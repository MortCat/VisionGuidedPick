"""Camera-only ROS stream boundary; shared semantic inference, C++ geometry/ID owner."""
import base64
from collections import Counter, deque
from copy import deepcopy
from dataclasses import dataclass
import io
import json
import math
from pathlib import Path
import subprocess
import sys
from statistics import median
import threading
import time


_TRANSIT_CONFIRMATIONS = 3
_BELT_SPEED_MPS = .06
_TRANSIT_ASSOCIATION_M = .075
_MAX_FRAME_GAP_NS = 250_000_000
# A single low-confidence, depth-less or missing gantry frame is bridged;
# two in a row restart the confirmation.
_CONFIRMATION_MISS_LIMIT = 1
_UPSTREAM_HINT_MAX_AGE_NS = 22_000_000_000
# Matches the second arm's hint age limit in ptc_vision.
_DOWNSTREAM_HINT_MAX_AGE_NS = 50_000_000_000
_WRIST_ACTIVE_INFERENCE_PERIOD_SEC = .12


def wrist_target_near(hint, wrist_epoch, source_ns, max_hint_age_ns, roi, lead_sec):
    measured = hint.pose.header.stamp
    age_ns = source_ns - (measured.sec * 10**9 + measured.nanosec)
    if hint.key.epoch != wrist_epoch or not 0 <= age_ns <= max_hint_age_ns:
        return False
    if roi is None:
        return True

    # The gantry anchors the horizontal belt trajectory. Wrist RGB-D measures
    # height before the existing 3D selection check.
    age = age_ns / 1e9
    position = hint.pose.pose.position
    velocity = hint.velocity_mps
    entry, exit_at = 0., lead_sec
    for axis, coordinate, speed in ((0, position.x, velocity.x), (1, position.y, velocity.y)):
        if not math.isfinite(coordinate) or not math.isfinite(speed):
            return False
        low, high = roi['bounds_min_m'][axis], roi['bounds_max_m'][axis]
        current = coordinate + speed * age
        if abs(speed) < 1e-9:
            if not low <= current <= high:
                return False
            continue
        first, second = (low-current)/speed, (high-current)/speed
        entry = max(entry, min(first, second))
        exit_at = min(exit_at, max(first, second))
        if entry > exit_at:
            return False
    return True


def wrist_inference_working(phase, wrist_epoch, source_ns, gantry_epoch, routed, frame_max_age_ns,
                            max_hint_age_ns, roi, lead_sec):
    if phase in (3, 4):
        return True
    routed_epoch, capture_ns, hints = routed
    return (phase == 1 and routed_epoch == gantry_epoch and source_ns is not None
            and 0 <= source_ns-capture_ns <= frame_max_age_ns
            and any(wrist_target_near(hint, wrist_epoch, source_ns, max_hint_age_ns, roi, lead_sec)
                    for hint in hints))


def measured_pick_area_exit(previous, current, roi):
    low, high = roi['bounds_min_m'], roi['bounds_max_m']
    return (low[0] <= previous[0] <= high[0] and low[0] <= current[0] <= high[0]
            and low[1] <= previous[1] <= high[1] < current[1])


def image_edge_margin(pixels):
    return max(1, round(pixels * .05))


def mask_near_image_edge(mask):
    height, width = mask.shape
    x, y = image_edge_margin(width), image_edge_margin(height)
    return (mask[:y, :].any() or mask[-y:, :].any()
            or mask[:, :x].any() or mask[:, -x:].any())


class GantryCloudHistory:
    """Short, capture-time history of measured surfaces within one Observe run."""

    WINDOW_NS = 350_000_000
    MAX_FRAMES = 4
    MAX_POINTS = 4096

    def __init__(self, support_z):
        self.support_z = support_z
        self.surfaces = {}

    def begin_frame(self, epoch, stamp_ns):
        self.surfaces = {
            key: frames for key, frames in self.surfaces.items()
            if key[0] == epoch and 0 <= stamp_ns - frames[-1][0] <= self.WINDOW_NS
        }

    def add(self, epoch, track_id, stamp_ns, cloud, *, near_edge=False):
        key = (epoch, track_id)
        if near_edge:
            # A clipped sighting ends this fusion window. The preview keeps its
            # last published revision until two interior scans are available.
            self.surfaces.pop(key, None)
            return
        import numpy as np

        points = np.asarray(cloud, dtype=np.float32).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > self.support_z)]
        if not len(points):
            return
        frames = self.surfaces.setdefault(key, deque(maxlen=self.MAX_FRAMES))
        if frames and stamp_ns <= frames[-1][0]:
            return
        frames.append((stamp_ns, points))
        while stamp_ns - frames[0][0] > self.WINDOW_NS:
            frames.popleft()

    def fused(self, epoch, track_id, stamp_ns, velocity):
        import numpy as np

        frames = self.surfaces.get((epoch, track_id))
        if not frames or len(frames) < 2 or frames[-1][0] != stamp_ns or velocity is None:
            return None
        if not np.isfinite(velocity).all():
            return None
        direction = np.asarray([velocity[0], velocity[1], 0.0], dtype=np.float32)
        aligned = np.concatenate([points + direction * ((stamp_ns - earlier) / 1e9)
                                  for earlier, points in frames])
        if len(aligned) > self.MAX_POINTS:
            aligned = aligned[np.linspace(0, len(aligned)-1, self.MAX_POINTS, dtype=int)]
        return frames[-1][1], aligned, len(frames)


class BinLoadVerifier:
    """Observe a stable net increase in support force after a keyed release."""

    GRAVITY_M_S2 = 9.81
    FRESH_SEC = 0.5
    MIN_GAIN_N = 0.20
    STABLE_SPREAD_N = 0.08
    SETTLE_NS = 750_000_000
    WINDOW_NS = 500_000_000
    BASELINE_LOOKBACK_NS = 1_000_000_000
    MAX_WAIT_SEC = 4.0
    OVERFLOW_WINDOW_NS = 2_000_000_000

    def __init__(self):
        self.samples = deque(maxlen=100)
        self.releases = deque(maxlen=64)
        self.pending = None
        self.pending_reports_miss = True
        self.watch_overflow = False
        self.peak_n = None
        self.carried = None
        self.last_drop_n = None
        self.last_release_stamp_ns = None
        self.events = []

    def sample(self, received, stamp_ns, force_n):
        if not math.isfinite(force_n):
            return
        if self.samples and stamp_ns < self.samples[-1][1]:
            self.samples.clear()
            self.carried = None
            self.last_drop_n = None
            self.last_release_stamp_ns = None
            if self.pending:
                self.finish('unverified', 'Bin load clock reset')
        if not self.samples or stamp_ns > self.samples[-1][1]:
            self.samples.append((received, stamp_ns, force_n))

    def stable_force(self, end_ns, start_ns=0):
        values = [(stamp, force) for _, stamp, force in self.samples
                  if max(start_ns, end_ns - self.WINDOW_NS) <= stamp <= end_ns]
        if (len(values) < 4 or values[-1][0] - values[0][0] < 300_000_000):
            return None
        forces = [force for _, force in values]
        if max(forces) - min(forces) > self.STABLE_SPREAD_N:
            return None
        return median(forces)

    def empty_after(self, now, tare_n, after_ns):
        current = self.samples[-1] if self.samples else None
        if not current or now - current[0] > self.FRESH_SEC or tare_n is None:
            return False
        force = self.stable_force(current[1], after_ns)
        return force is not None and abs(force - tare_n) <= self.STABLE_SPREAD_N

    def details(self, now, tare_n):
        current = self.samples[-1] if self.samples else None
        gross_n = (self.stable_force(current[1])
                   if current and now - current[0] <= self.FRESH_SEC else None)
        contents_g = None
        if (gross_n is not None and isinstance(tare_n, (int, float))
                and not isinstance(tare_n, bool) and math.isfinite(tare_n) and tare_n >= 0
                and gross_n >= tare_n - self.STABLE_SPREAD_N):
            contents_g = max(0.0, gross_n - tare_n) * 1000 / self.GRAVITY_M_S2
        last_drop_g = (None if self.last_drop_n is None
                       else self.last_drop_n * 1000 / self.GRAVITY_M_S2)
        return dict(contents_g=contents_g, last_drop_g=last_drop_g)

    def carry(self, now, key):
        """Record the bin load when a cycle starts carrying its object.

        An object that reaches the bin before the gripper opens, or after the
        grip is lost above the bin, is measured against this load.
        """
        if key in self.releases or (self.carried and self.carried[0] == key):
            return
        current = self.samples[-1] if self.samples else None
        baseline = None
        if current and now - current[0] <= self.FRESH_SEC:
            earliest = (self.last_release_stamp_ns + self.SETTLE_NS
                        if self.last_release_stamp_ns is not None else 0)
            baseline = self.stable_force(current[1], earliest)
        self.carried = (key, baseline)

    def ended(self, now, key):
        """Check the bin after a carrying cycle ended without a release."""
        if not self.carried or self.carried[0] != key or key in self.releases:
            return
        baseline = self.carried[1]
        self.carried = None
        self.releases.append(key)
        current = self.samples[-1] if self.samples else None
        if baseline is None or not current or self.pending:
            return
        self.last_release_stamp_ns = current[1]
        self.pending = (key, now, current[1], baseline)
        self.pending_reports_miss = False
        self.watch_overflow = False

    def release(self, now, key, near_rim=False):
        if key in self.releases:
            return
        self.releases.append(key)
        if self.pending:
            self.finish('unverified', 'Another release arrived before the load settled')
        self.pending_reports_miss = True
        self.watch_overflow = near_rim
        self.peak_n = None
        carried = self.carried[1] if self.carried and self.carried[0] == key else None
        self.carried = None
        current = self.samples[-1] if self.samples else None
        stamp_ns = current[1] if current else 0
        baseline = None
        if carried is not None and current and now - current[0] <= self.FRESH_SEC:
            baseline = carried
        elif current and now - current[0] <= self.FRESH_SEC:
            earliest = (self.last_release_stamp_ns + self.SETTLE_NS
                        if self.last_release_stamp_ns is not None else 0)
            # A brief disturbance as the arm approaches may spoil the final
            # window. Recover the latest recent plateau from this release only.
            for _, end_ns, _ in reversed(self.samples):
                if end_ns < max(earliest, stamp_ns - self.BASELINE_LOOKBACK_NS):
                    break
                baseline = self.stable_force(end_ns, earliest)
                if baseline is not None:
                    break
        if current:
            self.last_release_stamp_ns = stamp_ns
        self.pending = (key, now, stamp_ns, baseline)
        if baseline is None:
            self.finish('unverified', 'No stable bin load before release')
        else:
            self.events.append(self.event(key, 'placing', 'Checking bin load'))

    @staticmethod
    def event(key, state, reason, delta_n=None):
        return dict(epoch=key[0], cycle_id=key[1], track_id=key[2],
                    state=state, reason=reason, delta_n=delta_n)

    def finish(self, state, reason, delta_n=None):
        self.last_drop_n = delta_n
        self.events.append(self.event(self.pending[0], state, reason, delta_n))
        self.pending = None

    def results(self, now):
        if self.pending:
            _, started, stamp_ns, baseline = self.pending
            settled = self.samples[-1] if self.samples else None
            delta = None
            if settled and now - settled[0] <= self.FRESH_SEC:
                after = self.stable_force(settled[1], stamp_ns + self.SETTLE_NS)
                if after is not None:
                    delta = after - baseline
            expired = now - started >= self.MAX_WAIT_SEC
            monitoring = (self.watch_overflow and settled
                          and settled[1] - stamp_ns < self.OVERFLOW_WINDOW_NS and not expired)
            if self.watch_overflow and settled and now - settled[0] <= self.FRESH_SEC:
                if (delta is not None and self.peak_n is not None
                        and self.peak_n - delta >= self.MIN_GAIN_N and delta < self.MIN_GAIN_N):
                    self.finish('unverified', 'Bin overflow: near-rim load rose then fell', delta)
                    self.events[-1]['overflow'] = True
                else:
                    load = self.stable_force(settled[1], stamp_ns)
                    if load is not None and load-baseline >= self.MIN_GAIN_N:
                        self.peak_n = max(self.peak_n or 0, load-baseline)
            if self.pending and not monitoring and delta is not None and delta >= self.MIN_GAIN_N:
                self.finish('placed', 'Stable increase in bin load', delta)
            elif self.pending and expired:
                if self.pending_reports_miss:
                    self.finish('unverified', 'No sustained increase in bin load', delta)
                else:
                    # The cycle already reported its own failure.
                    self.pending = None
        events, self.events = self.events, []
        return events


@dataclass(frozen=True)
class TransitTrack:
    origin: tuple[int, float, float]
    deadline_ns: int
    candidate: dict
    confirmations: int
    samples: tuple = ()

    @property
    def confirmed(self):
        return self.confirmations >= _TRANSIT_CONFIRMATIONS


def update_transit_track(previous, candidate, association_distance_m=_TRANSIT_ASSOCIATION_M):
    measured_ns = int(candidate['measurement_stamp_ns'])
    x, y, _ = candidate['position_world_m']
    if previous is not None:
        last_ns = int(previous.candidate['measurement_stamp_ns'])
        if measured_ns <= last_ns:
            return previous
        # Allow the same bridged gantry frame as the pick confirmation.
        if not previous.confirmed and measured_ns - last_ns > _MAX_FRAME_GAP_NS * (1 + _CONFIRMATION_MISS_LIMIT):
            previous = None
    origin = previous.origin if previous is not None else (measured_ns, x, y)
    origin_ns, origin_x, origin_y = origin
    elapsed = (measured_ns - origin_ns) / 1e9
    # Late masks can absorb a neighboring block. Keep the last plausible
    # measurement without refreshing its stamp or counting an outlier.
    if math.hypot(x-origin_x, y-origin_y-_BELT_SPEED_MPS*elapsed) > association_distance_m:
        return previous
    count = min(previous.confirmations + 1, _TRANSIT_CONFIRMATIONS) if previous else 1
    samples = previous.samples if previous else ()
    samples = (*samples[-15:], (measured_ns, y))
    return TransitTrack(origin, measured_ns + 50_000_000_000, candidate, count, samples)


def measured_transit_speed(track, tracking):
    """Fit accepted RGB-D positions along the straight belt axis."""
    if not track.samples:
        return None
    latest = track.samples[-1][0]
    samples = [((stamp - latest) / 1e9, y) for stamp, y in track.samples
               if latest - stamp <= tracking['velocity_window_sec'] * 1e9]
    if (len(samples) < tracking['velocity_min_samples']
            or -samples[0][0] < tracking['velocity_min_span_sec']):
        return None
    mean_t = sum(t for t, _ in samples) / len(samples)
    mean_y = sum(y for _, y in samples) / len(samples)
    spread = sum((t - mean_t)**2 for t, _ in samples)
    speed = sum((t - mean_t) * (y - mean_y) for t, y in samples) / spread
    residual = math.sqrt(sum((y - mean_y - speed * (t - mean_t))**2
                             for t, y in samples) / len(samples))
    uncertainty = math.hypot(residual, tracking['max_position_stddev_m']) / math.sqrt(spread)
    if (speed <= 0 or not math.isfinite(uncertainty)
            or abs(speed - _BELT_SPEED_MPS) * (_UPSTREAM_HINT_MAX_AGE_NS / 1e9) > tracking['association_distance_m']
            or residual > tracking['max_velocity_fit_residual_m']
            or uncertainty > tracking['max_velocity_uncertainty_mps']):
        return None
    return speed, residual, uncertainty


def downstream_transit_hint(position, measured_ns, speed, wrist, now_ns, association_distance_m):
    """Prepare the second arm's search hint from the gantry record.

    The second arm extrapolates for up to _DOWNSTREAM_HINT_MAX_AGE_NS, so its
    belt speed must stay within the association distance over that horizon.
    A newer first-arm wrist measurement of the same ID replaces the gantry
    position and stamp, and the long gantry-to-wrist baseline sets the speed.
    Returns (position, measured_ns, speed, source).
    """
    limit = association_distance_m / (_DOWNSTREAM_HINT_MAX_AGE_NS / 1e9)
    bounded = min(max(speed, _BELT_SPEED_MPS - limit), _BELT_SPEED_MPS + limit)
    if wrist is None:
        return list(position), measured_ns, bounded, 'gantry'
    wrist_ns, wrist_position = wrist
    elapsed = (wrist_ns - measured_ns) / 1e9
    if not 0 < elapsed or wrist_ns > now_ns:
        return list(position), measured_ns, bounded, 'gantry'
    x, y = position[0], position[1] + speed * elapsed
    if math.hypot(wrist_position[0] - x, wrist_position[1] - y) > association_distance_m:
        return list(position), measured_ns, bounded, 'gantry'
    if elapsed >= 1:
        baseline = (wrist_position[1] - position[1]) / elapsed
        bounded = min(max(baseline, _BELT_SPEED_MPS - limit), _BELT_SPEED_MPS + limit)
    return list(wrist_position), wrist_ns, bounded, 'upstream_wrist'


def update_transit_routes(tracks, aliases, candidates, max_gap_sec, association_distance_m=_TRANSIT_ASSOCIATION_M):
    # Keep the published identity when the image tracker starts a new
    # fragment on the same trajectory. Never merge co-visible candidates.
    previous_routes = {key: track for key, track in tracks.items() if track.confirmed}
    visible_routes = set()
    new_routes = []
    for candidate in candidates:
        raw_key = candidate['track_key']
        key = aliases.get(raw_key, raw_key)
        previous = tracks.get(key)
        if key != raw_key:
            candidate = dict(candidate, track_key=key, ros_track_id=previous.candidate['ros_track_id'])
        updated = update_transit_track(previous, candidate, association_distance_m)
        tracks[key] = updated
        if updated is not previous:
            visible_routes.add(key)
        if updated.confirmed and not (previous and previous.confirmed):
            new_routes.append(key)

    joins = {}
    matches_per_route = Counter()
    for key in new_routes:
        candidate = tracks[key].candidate
        measured_ns = int(candidate['measurement_stamp_ns'])
        matches = []
        for old_key, old in previous_routes.items():
            if old_key in visible_routes:
                continue
            last_ns = int(old.candidate['measurement_stamp_ns'])
            if not .25 <= (measured_ns-last_ns)/1e9 <= max_gap_sec:
                continue
            canonical = dict(candidate, track_key=old_key, ros_track_id=old.candidate['ros_track_id'])
            refreshed = update_transit_track(old, canonical, association_distance_m)
            if refreshed is not old:
                matches.append((old_key, refreshed))
                matches_per_route[old_key] += 1
        if len(matches) == 1:
            joins[key] = matches[0]

    for key, (old_key, refreshed) in joins.items():
        if matches_per_route[old_key] == 1:
            tracks[old_key] = refreshed
            del tracks[key]
            aliases[key] = old_key


def display_bounds(points):
    import numpy as np

    points = np.asarray(points).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 20:
        return None
    # Keep the silhouette's extremes while dropping isolated depth spikes.
    sorted_points = np.sort(points, axis=0)
    trim = max(1, len(points) // 100)
    low, high = sorted_points[trim], sorted_points[-trim - 1]
    size = high - low
    # A gap between two well-supported surfaces is ambiguous, including when
    # the segmentation joined their projections. It must not anchor a box.
    flank = len(points) // 5
    gaps = np.diff(sorted_points[flank - 1:len(points) - flank + 1], axis=0)
    if np.any(gaps > np.maximum(.006, .20 * size)):
        return None
    padding = .03 * size
    return low - padding, high + padding


def project_display_box(center, half_size, camera_pose, mask_bounds, image_size):
    import numpy as np

    pose, k = camera_pose
    corners = np.array([[center[axis] + (half_size[axis] if bits & (1 << axis) else -half_size[axis])
                         for axis in range(3)] for bits in range(8)])
    corners = (corners - pose[:3, 3]) @ pose[:3, :3]
    if not np.all(corners[:, 2] > 0):
        return []
    pixel = np.column_stack((corners[:, 0] * k[0] / corners[:, 2] + k[2],
                             corners[:, 1] * k[4] / corners[:, 2] + k[5]))
    if not np.all(np.isfinite(pixel)):
        return []
    low, high = np.clip([pixel.min(axis=0), pixel.max(axis=0)], 0, image_size)
    mask_low, mask_high = mask_bounds
    mask_size = mask_high - mask_low
    allowance = np.maximum(2., .08 * mask_size)
    if (np.any(low > mask_low + allowance) or np.any(high < mask_high - allowance)
            or np.any(high - low > 1.65 * mask_size + 2. * allowance)):
        return []
    intersection = np.prod(np.maximum(0, np.minimum(high, mask_high) - np.maximum(low, mask_low)))
    union = np.prod(high - low) + np.prod(mask_size) - intersection
    if union <= 0 or intersection / union < .45:
        return []
    pixel = pixel.tolist()
    return [[pixel[i], pixel[i ^ (1 << axis)]] for i in range(8)
            for axis in range(3) if not i & (1 << axis)]


def remove_detached_mask_fragments(instance):
    """Remove small, distant islands before gantry depth and overlay use the mask."""
    import cv2
    import numpy as np

    mask = instance['mask']
    rows = np.flatnonzero(mask.any(axis=1))
    cols = np.flatnonzero(mask.any(axis=0))
    if not len(rows) or not len(cols):
        return instance
    region = np.s_[rows[0]:rows[-1]+1, cols[0]:cols[-1]+1]
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask[region].astype(np.uint8), connectivity=8)
    if count <= 2:
        return instance

    components = stats[1:]
    main_index = int(components[:, cv2.CC_STAT_AREA].argmax())
    x, y, width, height, area = components[main_index]
    left, top, widths, heights, areas = components.T
    gap_x = np.maximum(0, np.maximum(x - (left + widths), left - (x + width)))
    gap_y = np.maximum(0, np.maximum(y - (top + heights), top - (y + height)))
    # Keep nearby fragments and plausible second surfaces, including occluded parts.
    discard = (areas < .2 * area) & (np.hypot(gap_x, gap_y) > .25 * max(width, height))
    if not discard.any():
        return instance

    keep = np.r_[False, ~discard]
    cleaned = np.zeros_like(mask)
    cleaned[region] = keep[labels]
    bounds = components[~discard]
    return {**instance, 'mask': cleaned, 'bbox_xyxy': [
        float(cols[0] + bounds[:, 0].min()), float(rows[0] + bounds[:, 1].min()),
        float(cols[0] + (bounds[:, 0] + bounds[:, 2]).max()),
        float(rows[0] + (bounds[:, 1] + bounds[:, 3]).max()),
    ]}


def display_box(bounds, velocity, mask, camera_pose, previous, capture_ns, keep_size=False):
    import cv2
    import numpy as np

    # The anchor timestamp records accepted depth. RGB sightings never renew it.
    if previous is not None and not 0 <= capture_ns - previous[0] <= _MAX_FRAME_GAP_NS:
        previous = None
    if camera_pose is None:
        return [], previous
    rows = np.flatnonzero(mask.any(axis=1))
    cols = np.flatnonzero(mask.any(axis=0))
    if not len(rows) or not len(cols):
        return [], previous
    height, width = mask.shape
    mask_low, mask_high = np.array([cols[0], rows[0]]), np.array([cols[-1]+1, rows[-1]+1])
    mask_bounds = (mask_low, mask_high)
    crop = mask[rows[0]:rows[-1]+1, cols[0]:cols[-1]+1].astype(np.uint8)
    _, _, stats, _ = cv2.connectedComponentsWithStats(crop, connectivity=8)
    areas = stats[1:, cv2.CC_STAT_AREA]
    if np.count_nonzero(areas >= max(4, .2 * areas.max())) > 1:
        return [], previous
    image_size = np.array([width, height])
    margin = np.array([image_edge_margin(width), image_edge_margin(height)])
    near_edge = np.any(mask_low < margin) or np.any(mask_high > image_size - margin)
    if bounds is not None and not near_edge:
        minimum, maximum = bounds
        size = maximum - minimum
        sizes = previous[3] if previous else ()
        stable_size = np.median(sizes, axis=0) if keep_size and sizes else None
        complete = stable_size is None or np.all((size >= .7 * stable_size) & (size <= 1.4 * stable_size))
        if np.all(np.isfinite(size)) and np.all(size > 0) and complete:
            center = (minimum + maximum) * .5
            measured_edges = project_display_box(center, size * .5, camera_pose, mask_bounds, image_size)
            if measured_edges:
                # Keep learning from accepted depth after confirmation. A single
                # early fit must not fix this track's dimensions for its lifetime.
                sizes = sizes[-4:] + (size,)
                edges = project_display_box(center, np.median(sizes, axis=0) * .5,
                                            camera_pose, mask_bounds, image_size)
                if edges:
                    speed = np.asarray(velocity) if velocity is not None else np.zeros(3)
                    if not np.all(np.isfinite(speed)):
                        speed = np.zeros(3)
                    return edges, (capture_ns, center, speed, sizes)
    if previous is None:
        return [], None
    stamp_ns, center, speed, sizes = previous
    center = center + speed * ((capture_ns - stamp_ns) / 1e9)
    return project_display_box(center, np.median(sizes, axis=0) * .5,
                               camera_pose, mask_bounds, image_size), previous


def confirm_pick(previous, capture_ns, minimum_ns, qualified=True):
    """Return ((start_ns, last_frame_ns, misses) or None, confirmed)."""
    continuous = previous is not None and 0 <= capture_ns - previous[1] <= _MAX_FRAME_GAP_NS
    if not qualified:
        if continuous and previous[2] < _CONFIRMATION_MISS_LIMIT:
            return (previous[0], capture_ns, previous[2] + 1), False
        return None, False
    if not continuous:
        return (capture_ns, capture_ns, 0), False
    return (previous[0], capture_ns, 0), capture_ns - previous[0] >= minimum_ns


def joint_feedback_rows(msg):
    def finite_at(values, index):
        return values[index] if index < len(values) and math.isfinite(values[index]) else None

    joint_types = {f'panda_joint{i}': 'revolute' for i in range(1, 8)}
    joint_types.update({f'panda_finger_joint{i}': 'prismatic' for i in range(1, 3)})
    indices = {name: index for index, name in enumerate(msg.name)}
    return [dict(name=name, type=kind, position=finite_at(msg.position, indices[name]),
                 effort=finite_at(msg.effort, indices[name]))
            for name, kind in joint_types.items() if name in indices]


def feedback_timestamp_issue(clock_ns, stamp_ns, future_tolerance_ns, clock_step_ns):
    age_ns = clock_ns - stamp_ns
    # The feedback and /clock topics may deliver the same simulation step in either order.
    allowed_future_ns = max(future_tolerance_ns, min(clock_step_ns, 50_000_000) + 1_000)
    if age_ns < -allowed_future_ns:
        return f'Feedback timestamp is {-age_ns / 1e6:.1f} ms ahead of /clock'
    if age_ns > 1_000_000_000:
        return f'Feedback timestamp is {age_ns / 1e6:.1f} ms behind /clock'
    return None


# Observe's own outputs, relative to its node namespace. Resolved defaults stay
# /factory/observe/..., which the arm launcher and vision hint ages expect.
OBSERVE_NAMESPACE = 'factory/observe'
PICK_PREVIEW_TOPICS = {'arm_01': 'pick_preview_tracks', 'arm_02': 'pick_preview_tracks_arm_02'}
GANTRY_CLOUD_TOPIC = 'gantry_cloud'
# Arm-side names come from each selected profile's ros.endpoints.
_STATION_ENDPOINT_KEYS = ('tracks', 'frame', 'vision_status', 'inference_status', 'detections',
                          'task_status', 'task_command', 'gripper_status', 'arm_status',
                          'joint_states', 'rgb')
# Device/task topics without an endpoint key keep their suffix under the station namespace.
_STATION_SUFFIXES = {'bin_load': 'bin/load', 'bin_release': 'task/bin_release', 'bin_clear': 'bin/clear'}


def resolve_topic(namespace, name):
    namespace, name = namespace.strip('/'), str(name)
    if not namespace or not name or name.startswith('/') or name.startswith('~'):
        raise ValueError(f'Expected a station namespace and relative name, got {namespace!r}, {name!r}')
    return f'/{namespace}/{name}'


def _runtime_module(project, name):
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'observe_' + name, Path(project) / 'deployment/runtime' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def selected_profile_path(project, arm):
    """The deployment's selected arm profile (deployment/runtime/model_variant.py)."""
    return _runtime_module(project, 'model_variant').settings_file(project, arm)


def load_station_profile(project, arm):
    """Selected arm profile plus its product overlay, as the semantic arm launch uses."""
    import yaml
    source = selected_profile_path(project, arm)
    profile = yaml.safe_load(source.read_text(encoding='utf-8'))
    overlay = source.with_suffix('.product.yaml')
    if overlay.is_file():
        _runtime_module(project, 'semantic_settings').merge(
            profile, yaml.safe_load(overlay.read_text(encoding='utf-8')))
    return profile


def station_topics(profile):
    namespace = profile['runtime']['namespace']
    endpoints = profile['ros']['endpoints']
    missing = [key for key in _STATION_ENDPOINT_KEYS if key not in endpoints]
    if missing:
        raise ValueError('Arm profile lacks ros.endpoints: ' + ', '.join(missing))
    names = {key: endpoints[key] for key in _STATION_ENDPOINT_KEYS} | _STATION_SUFFIXES
    topics = {key: resolve_topic(namespace, name) for key, name in names.items()}
    topics['namespace'] = '/' + namespace.strip('/')
    return topics


def vision_subscribed(endpoints, station_namespace):
    """True when the arm's vision node subscribes to the given topic's endpoint list."""
    return any(endpoint.node_namespace == station_namespace and endpoint.node_name == 'vision'
               for endpoint in endpoints)


def run(model, descriptor, run_id, config, emit):
    import numpy as np
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.duration import Duration
    from sensor_msgs.msg import Image as RosImage, CameraInfo, JointState
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from geometry_msgs.msg import WrenchStamped
    from rosgraph_msgs.msg import Clock
    from tf2_ros import Buffer, TransformListener
    from PIL import Image
    import yaml
    project = Path(__file__).resolve().parents[4]
    from ptc_interfaces.msg import ArmStatus, CapabilityStatus, Detection, DetectionArray, EventKey, GripperStatus, ObjectCloudPreview, RgbdFrame, TaskStatus, TrackedObject, TrackedObjectArray
    from ptc_interfaces.srv import TaskCommand
    from std_srvs.srv import Trigger
    settings = yaml.safe_load((project / 'config/factory_demo/arm_01.yaml').read_text())
    # Gantry camera calibration above stays separate from the selected arm stations.
    station = {arm: station_topics(load_station_profile(project, arm)) for arm in ('arm_01', 'arm_02')}
    if len({topics['namespace'] for topics in station.values()}) != len(station):
        raise ValueError('Arm profiles must use distinct runtime.namespace values')
    camera = settings['vision']['tracking_camera']
    # Existing camera naming contract, not a second editable topic catalogue.
    prefix = camera['color_optical'].removesuffix('_color_optical_frame')
    world = settings['frames']['world']
    candidates=[project/'native_install/ptc_vision/lib/ptc_vision/observe_geometry',project/'native_install/lib/ptc_vision/observe_geometry']
    executable=next((path for path in candidates if path.is_file()),None)
    if executable is None: raise RuntimeError('Build the ptc_vision observe_geometry executable first')
    temporary_loss_sec = settings['tracking']['temporary_loss_timeout_sec']
    association_distance_m = settings['tracking']['association_distance_m']
    core = subprocess.Popen([str(executable), str(temporary_loss_sec),
                             str(_BELT_SPEED_MPS), str(association_distance_m)],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    commands = deque()
    def receive_commands():
        for line in sys.stdin:
            commands.append(json.loads(line))
        commands.append({'command':'stop_run','run_id':run_id})
    threading.Thread(target=receive_commands, daemon=True).start()
    rclpy.init()
    class ObserveStreamNode(Node):
        def __init__(self):
            super().__init__('observe_stream', namespace=OBSERVE_NAMESPACE)
            self.pick_preview_pub = self.create_publisher(
                TrackedObjectArray, PICK_PREVIEW_TOPICS['arm_01'], 5)
            self.downstream_preview_pub = self.create_publisher(
                TrackedObjectArray, PICK_PREVIEW_TOPICS['arm_02'], 5)
            self.gantry_cloud_pub = self.create_publisher(
                ObjectCloudPreview, GANTRY_CLOUD_TOPIC, qos_profile_sensor_data)
            self.upstream_claims = {}
            self.upstream_outcomes = {}
            self.upstream_released = set()
            self.upstream_gripper = None
            self.pending_rgb = deque(maxlen=5)
            self.wrist_frames = {}
            self.wrist_epochs = {}
            self.wrist_detections = {}
            self.wrist_tasks = {}
            self.upstream_wrist_tracks = {}
            self.downstream_pick_area = {}
            self.create_subscription(
                TrackedObjectArray, station['arm_02']['tracks'], self.update_downstream_tracks, 5)
            self.create_subscription(
                TrackedObjectArray, station['arm_01']['tracks'], self.update_upstream_tracks, 5)
            self.planned_grasps = {arm: deque(maxlen=20) for arm in ('arm_01', 'arm_02')}
            self.wrist_preview_frames = {}
            self.vision_status = {}
            self.inference_status = {}
            self.bin_load = {arm: BinLoadVerifier() for arm in ("arm_01", "arm_02")}
            for arm in ('arm_01', 'arm_02'):
                self.create_subscription(
                    WrenchStamped, station[arm]['bin_load'],
                    lambda msg, arm=arm: self.bin_load[arm].sample(
                        time.monotonic(), msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                        msg.wrench.force.z), qos_profile_sensor_data)
                self.create_subscription(
                    EventKey, station[arm]['bin_release'],
                    lambda msg, arm=arm: self.bin_release(arm, msg), 10)
                self.inference_status[arm] = self.create_publisher(
                    CapabilityStatus, station[arm]['inference_status'], 5)
                self.create_subscription(
                    CapabilityStatus, station[arm]['vision_status'],
                    lambda msg, arm=arm: self.vision_status.__setitem__(arm, msg), 5)
                self.wrist_detections[arm] = self.create_publisher(
                    DetectionArray, station[arm]['detections'], 5)
                self.create_subscription(
                    RgbdFrame, station[arm]['frame'],
                    lambda msg, arm=arm: self.update_wrist_frame(arm, msg),
                    qos_profile_sensor_data)
                self.create_subscription(
                    TaskStatus, station[arm]['task_status'],
                    lambda msg, arm=arm: self.update_task(arm, msg), 5)
                self.create_subscription(
                    GripperStatus, station[arm]['gripper_status'],
                    lambda msg, arm=arm: self.update_gripper(arm, msg), 5)
            self.depth = deque(maxlen=8)
            self.info = {}
            self.sequence = 0
            self.received_count = 0
            self.input_times = deque(maxlen=128)
            self.last_rgb_stamp = None
            self.dropped = 0
            self.epoch = 0
            self.clock_ns = None
            self.tf = Buffer(cache_time=Duration(seconds=5))
            self.listener = TransformListener(self.tf, self)
            self.create_subscription(RosImage, '/' + prefix + '/color/image_raw', self.rgb, qos_profile_sensor_data)
            self.create_subscription(RosImage, '/' + prefix + '/depth/image_raw', lambda msg:self.depth.append(msg), qos_profile_sensor_data)
            for kind in ('color','depth'):
                self.create_subscription(CameraInfo, '/' + prefix + '/' + kind + '/camera_info', lambda msg,k=kind:self.info.__setitem__(k,msg), qos_profile_sensor_data)
            self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        def update_wrist_frame(self, arm, msg):
            self.wrist_frames[arm] = msg
            self.wrist_preview_frames[arm] = (msg, time.monotonic())
            self.wrist_epochs[arm] = msg.key.epoch

        def bin_release(self, arm, msg):
            task, seen = self.wrist_tasks.get(arm, (None, 0.0))
            now = time.monotonic()
            rim = config.get('bin_rim_height_m', {}).get(arm)
            near_rim = False
            if task is not None and now-seen <= 1 and rim is not None:
                height_stamp = task.bin_surface_stamp.sec*10**9 + task.bin_surface_stamp.nanosec
                same_cycle = all(getattr(task.key, k) == getattr(msg, k)
                                 for k in ('epoch', 'cycle_id', 'track_id'))
                near_rim = (same_cycle and height_stamp > 0 and self.clock_ns is not None
                            and 0 <= self.clock_ns-height_stamp <= 3*10**9
                            and task.bin_surface_z_m >= rim-config.get('bin_rim_margin_m', .025))
            self.bin_load[arm].release(now, (str(msg.epoch), str(msg.cycle_id), str(msg.track_id)), near_rim)

        def update_upstream_tracks(self, msg):
            # First-arm wrist measurements of shared IDs re-anchor the second arm's hint.
            for item in msg.objects:
                if item.phase != TrackedObject.TRACKING or not item.velocity_valid or not item.key.track_id:
                    continue
                measured = item.pose.header.stamp.sec * 1000000000 + item.pose.header.stamp.nanosec
                previous = self.upstream_wrist_tracks.get(item.key.track_id)
                if previous is None or previous[0] != item.key.epoch or measured > previous[1]:
                    p = item.pose.pose.position
                    self.upstream_wrist_tracks[item.key.track_id] = (item.key.epoch, measured, [p.x, p.y, p.z])

        def update_downstream_tracks(self, msg):
            now = time.monotonic()
            tasks = [self.wrist_tasks.get(arm, (None, 0.)) for arm in ('arm_01', 'arm_02')]
            if any(task is None or not task.continuous_operation or now-seen >= 1.
                   for task, seen in tasks):
                self.downstream_pick_area.clear()
                return
            if self.clock_ns is None or msg.header.frame_id != world:
                return
            roi = arm_settings['arm_02']['task'].get('target_selection_roi')
            if roi is None:
                return
            for item in msg.objects:
                track_id = item.key.track_id
                measured_ns = stamp(item.pose)
                if (not track_id or track_id >> 32 != int(run_id[:8], 16)
                        or item.phase != TrackedObject.TRACKING
                        or item.key.epoch != self.wrist_epochs.get('arm_02')
                        or item.key.epoch != tasks[1][0].key.epoch
                        or not 0 <= self.clock_ns-measured_ns <= vision_max_age_ns['arm_02']):
                    continue
                point = item.pose.pose.position
                current = (point.x, point.y)
                previous = self.downstream_pick_area.get(track_id)
                if previous is not None:
                    epoch, last_ns, last = previous
                    if epoch == self.epoch and measured_ns <= last_ns:
                        continue
                    if (epoch == self.epoch and measured_ns-last_ns <= vision_max_age_ns['arm_02']
                            and measured_pick_area_exit(last, current, roi)
                            and not any(task.key.track_id == track_id and task.phase >= 3
                                        for task, _ in tasks)):
                        emit(dict(event='pick_area_exit', run_id=run_id, epoch=str(self.epoch),
                                  track_id=str(track_id)))
                self.downstream_pick_area[track_id] = (self.epoch, measured_ns, current)
                if len(self.downstream_pick_area) > 128:
                    del self.downstream_pick_area[next(iter(self.downstream_pick_area))]

        def update_gripper(self, arm, msg):
            if arm == 'arm_01':
                self.upstream_gripper = (msg, time.monotonic())

        def update_task(self, arm, msg):
            self.wrist_tasks[arm] = (msg, time.monotonic())
            if not msg.continuous_operation:
                self.downstream_pick_area.clear()
            if msg.key.track_id and msg.grasp_possession == TaskStatus.POSSESSION_HOLDING:
                self.bin_load[arm].carry(time.monotonic(), (
                    str(msg.key.epoch), str(msg.key.cycle_id), str(msg.key.track_id)))
            ended = msg.last_completed_key
            if msg.last_outcome and ended.track_id:
                self.bin_load[arm].ended(time.monotonic(), (
                    str(ended.epoch), str(ended.cycle_id), str(ended.track_id)))
            history = self.planned_grasps[arm]
            if msg.planned_grasp_center.header.frame_id:
                if history and stamp(msg.planned_grasp_center) < stamp(history[-1].planned_grasp_center):
                    history.clear()
                history.append(msg)
            else:
                history.clear()
            if arm != 'arm_01':
                return
            key = msg.key
            if key.track_id:
                identity = (key.epoch, key.cycle_id)
                if self.upstream_claims.get(key.track_id) != identity:
                    self.upstream_claims[key.track_id] = identity
                    self.upstream_released.discard(key.track_id)
            completed = msg.last_completed_key
            if msg.last_outcome and completed.track_id:
                identity = (completed.epoch, completed.cycle_id, completed.track_id)
                self.upstream_outcomes[identity] = msg.last_outcome
                if len(self.upstream_outcomes) > 128:
                    del self.upstream_outcomes[next(iter(self.upstream_outcomes))]

        def clock(self, msg):
            value = msg.clock.sec * 1000000000 + msg.clock.nanosec
            if self.clock_ns is not None and value < self.clock_ns:
                self.epoch += 1
                self.pending_rgb.clear()
                self.depth.clear()
                self.wrist_frames.clear()
                self.wrist_epochs.clear()
                self.vision_status.clear()
                self.last_rgb_stamp = None
            self.clock_ns = value
        def rgb(self, msg):
            value=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            if self.last_rgb_stamp is not None and value<self.last_rgb_stamp:
                self.epoch+=1
                self.depth.clear()
                self.wrist_frames.clear()
                self.wrist_epochs.clear()
                self.pending_rgb.clear()
            self.last_rgb_stamp=value
            self.input_times.append(time.monotonic())
            if len(self.pending_rgb) == self.pending_rgb.maxlen: self.dropped += 1
            self.sequence += 1
            self.received_count += 1
            self.pending_rgb.append((msg, self.sequence, self.epoch, time.monotonic()))
    class FeedbackNode(Node):
        def __init__(self):
            super().__init__('auto_pick_feedback', namespace='factory/observer')
            self.lock = threading.Lock()
            self.clock_ns = None
            self.clock_seen = 0.0
            self.clock_step_ns = 0
            self.samples = {}
            self.joint_history = {arm: deque(maxlen=8) for arm in ('arm_01', 'arm_02')}
            self.wrist_rgb = {}
            self.create_subscription(Clock, '/clock', self.update_clock, qos_profile_sensor_data)
            for arm in ('arm_01', 'arm_02'):
                self.create_subscription(
                    ArmStatus, station[arm]['arm_status'],
                    lambda msg, arm=arm: self.update_sample(arm, 'arm', msg), 1)
                self.create_subscription(
                    GripperStatus, station[arm]['gripper_status'],
                    lambda msg, arm=arm: self.update_sample(arm, 'gripper', msg), 1)
                self.create_subscription(
                    JointState, station[arm]['joint_states'],
                    lambda msg, arm=arm: self.update_joints(arm, msg), qos_profile_sensor_data)
                self.create_subscription(
                    RosImage, station[arm]['rgb'],
                    lambda msg, arm=arm: self.update_wrist_rgb(arm, msg), qos_profile_sensor_data)

        def update_wrist_rgb(self, arm, msg):
            with self.lock:
                self.wrist_rgb[arm] = (msg, time.monotonic())

        def latest_wrist_rgb(self, arm):
            with self.lock:
                return self.wrist_rgb.get(arm, (None, 0.0))

        def update_clock(self, msg):
            value = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
            with self.lock:
                if self.clock_ns is not None:
                    step = value - self.clock_ns
                    if step < 0:
                        self.samples.clear()
                        for history in self.joint_history.values():
                            history.clear()
                        self.wrist_rgb.clear()
                        self.clock_step_ns = 0
                    elif 0 < step <= 50_000_000:
                        self.clock_step_ns = step
                self.clock_ns = value
                self.clock_seen = time.monotonic()

        def update_sample(self, arm, device, msg):
            with self.lock:
                self.samples[arm, device] = (msg, time.monotonic())

        def update_joints(self, arm, msg):
            with self.lock:
                self.joint_history[arm].append((msg, time.monotonic()))

        def snapshot(self, arm):
            with self.lock:
                return (self.clock_ns, self.clock_seen, self.clock_step_ns,
                        self.samples.get((arm, 'gripper'), (None, 0.0)),
                        self.samples.get((arm, 'arm'), (None, 0.0)),
                        tuple(self.joint_history[arm]))

    node = ObserveStreamNode()
    feedback_node = FeedbackNode()
    feedback_executor = SingleThreadedExecutor()
    feedback_executor.add_node(feedback_node)
    task_clients = {arm: node.create_client(TaskCommand, station[arm]['task_command'])
                    for arm in node.wrist_detections}
    task_requests = {}
    next_task_status = 0.0

    preview_publishers = {'arm_01': node.pick_preview_pub, 'arm_02': node.downstream_preview_pub}

    def semantic_arm_ready(arm):
        # Inspect the publisher's resolved (remapped) topic, not a fixed name.
        return (task_clients[arm].service_is_ready() and vision_subscribed(
            node.get_subscriptions_info_by_topic(preview_publishers[arm].topic_name),
            station[arm]['namespace']))

    clear_clients = {arm: node.create_client(Trigger, station[arm]['bin_clear']) for arm in task_clients}
    clear_requests = {}
    pause_requests = {}
    next_pause = {}
    state_file = project / 'logs/observe/bin_overflow.json'
    try:
        recorded = json.loads(state_file.read_text(encoding='utf-8'))
        overflow = {arm for arm in task_clients if arm in recorded}
    except FileNotFoundError:
        overflow = set()

    def save_overflow():
        state_file.parent.mkdir(parents=True, exist_ok=True)
        temp = state_file.with_suffix('.tmp')
        temp.write_text(json.dumps(sorted(overflow)), encoding='utf-8')
        temp.replace(state_file)

    def bin_clear_available(arm, task, seen):
        if (arm in clear_requests or arm in task_requests or task is None or time.monotonic()-seen > 1.0
                or task.phase != 0 or task.continuous_operation or task.home_state in (1, 2, 3)
                or node.bin_load[arm].pending is not None
                or not clear_clients[arm].service_is_ready()
                or config.get('bin_tare_n', {}).get(arm) is None):
            return False
        feedback = device_feedback(arm, task)
        gripper, motor = feedback['gripper'], feedback['arm_feedback']
        return bool(gripper['fresh'] and gripper['ready'] and gripper['stopped']
                    and gripper['possession'] == 'empty'
                    and motor['fresh'] and motor['ready'] and motor['stopped'])

    def bin_clear_reply(arm, request_id, accepted, message):
        del clear_requests[arm]
        emit(dict(event='pick_reply', arm=arm, request_id=request_id, accepted=accepted, message=message))

    def advance_bins(now):
        for arm, (request_id, sent, future, after_ns) in list(clear_requests.items()):
            load = node.bin_load[arm]
            if future is not None and future.done():
                try:
                    reply = future.result()
                except Exception as error:
                    bin_clear_reply(arm, request_id, False, str(error))
                    continue
                if not reply.success:
                    bin_clear_reply(arm, request_id, False, reply.message)
                    continue
                after_ns = load.samples[-1][1] + load.SETTLE_NS if load.samples else 0
                clear_requests[arm] = (request_id, now, None, after_ns)
            elif future is None and load.empty_after(now, config['bin_tare_n'][arm], after_ns):
                if arm in overflow:
                    overflow.remove(arm)
                    save_overflow()
                bin_clear_reply(arm, request_id, True, 'Bin cleared and empty weight verified')
            elif now - sent >= (10 if future is None else 5):
                if future is not None:
                    future.cancel()
                bin_clear_reply(arm, request_id, False, 'Could not verify an empty bin')
        for arm in overflow:
            task, seen = node.wrist_tasks.get(arm, (None, 0.0))
            pending = pause_requests.get(arm)
            if pending is not None:
                sent, future = pending
                if future.done() or now - sent >= 5:
                    if not future.done():
                        future.cancel()
                    del pause_requests[arm]
            if (task is not None and now-seen <= 1 and task.continuous_operation
                    and arm not in pause_requests and arm not in task_requests
                    and now >= next_pause.get(arm, 0) and task_clients[arm].service_is_ready()):
                req = TaskCommand.Request()
                req.command = TaskCommand.Request.PAUSE_CONTINUOUS
                pause_requests[arm] = (now, task_clients[arm].call_async(req))
                next_pause[arm] = now + 1

    count = 0
    started = time.monotonic()
    last_inference = 0
    model_frames = deque(maxlen=8)
    last_health = 0
    output_times = deque(maxlen=128)
    def rate(samples):
        recent=[value for value in samples if value>=time.monotonic()-2]
        return (len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 and recent[-1]>recent[0] else 0.0
    emit(dict(event='started', run_id=run_id))

    def stamp(msg): return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

    def device_feedback(arm, task):
        expected_epoch = node.wrist_epochs.get(arm) or (task.key.epoch if task else None)
        future_tolerance_ns = settings['time']['sensor_future_tolerance_sec'] * 1e9
        clock_ns, clock_seen, clock_step_ns, grip_sample, arm_sample, joint_history = feedback_node.snapshot(arm)
        now = time.monotonic()

        def feedback_issue(sample, *, has_epoch=True):
            msg, seen = sample
            if msg is None:
                return 'No feedback received'
            if now - seen > 1.0:
                return 'No new feedback for over 1 s'
            if has_epoch and msg.key.epoch != expected_epoch:
                return 'Feedback belongs to a different simulation epoch'
            if clock_ns is None or now - clock_seen > 1.0:
                return 'Simulation clock unavailable'
            return feedback_timestamp_issue(clock_ns, stamp(msg), future_tolerance_ns, clock_step_ns)

        grip_issue = feedback_issue(grip_sample)
        arm_issue = feedback_issue(arm_sample)
        grip_fresh = grip_issue is None
        arm_fresh = arm_issue is None
        grip = grip_sample[0] if grip_fresh else None
        motor = arm_sample[0] if arm_fresh else None
        possession = 'unknown'
        if grip and grip.ready:
            if grip.empty_confirmed and not grip.contact_confirmed:
                possession = 'empty'
            elif (task and task.key.epoch == grip.key.epoch
                  and task.grasp_possession == TaskStatus.POSSESSION_HOLDING and grip.contact_confirmed):
                possession = 'holding'
        # A newer sample can reach this subscriber before its /clock update.
        # Display an earlier eligible measurement until the clock catches up.
        joints = None
        joint_issue = feedback_issue(joint_history[-1] if joint_history else (None, 0.0), has_epoch=False)
        for sample in reversed(joint_history):
            if feedback_issue(sample, has_epoch=False) is None:
                joints, joint_issue = sample[0], None
                break
        return dict(
            joint_feedback=dict(fresh=joint_issue is None, issue=joint_issue,
                                joints=joint_feedback_rows(joints) if joints is not None else []),
            gripper=dict(fresh=grip_fresh, issue=grip_issue, ready=bool(grip and grip.ready),
                         stopped=bool(grip and grip.stopped), width_m=grip.width_m if grip and math.isfinite(grip.width_m) else None,
                         possession=possession),
            arm_feedback=dict(fresh=arm_fresh, issue=arm_issue, ready=bool(motor and motor.ready),
                              command_ready=bool(motor and motor.command_ready),
                              stopped=bool(motor and motor.stopped)),
        )
    def transform(target, source, rgb):
        if target == source: return np.eye(4)
        tf = node.tf.lookup_transform(target, source, Time.from_msg(rgb.header.stamp), timeout=Duration(seconds=0)).transform
        x,y,z,w = tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w
        matrix = np.eye(4)
        matrix[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
        matrix[:3,3] = [tf.translation.x,tf.translation.y,tf.translation.z]
        return matrix
    def planned_grasp_pixels(arm, rgbd, task):
        frame, info = rgbd.color, rgbd.intrinsics
        if (rgbd.key.epoch != task.key.epoch or
                (info.width, info.height) != (frame.width, frame.height)):
            return None
        identity = lambda msg: (msg.key.epoch, msg.key.cycle_id, msg.key.track_id)
        plan = min((msg for msg in node.planned_grasps[arm] if identity(msg) == identity(task)),
                   key=lambda msg: abs(stamp(msg.planned_grasp_center) - stamp(frame)), default=None)
        if plan is None or abs(stamp(plan.planned_grasp_center) - stamp(frame)) > 100_000_000:
            return None
        capture = rgbd.camera_world
        if (capture.header.frame_id != plan.planned_grasp_center.header.frame_id
                or stamp(capture) != stamp(frame)):
            return None
        origin = capture.pose.position
        q = capture.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        camera_to_world = np.asarray([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])
        points = [plan.planned_grasp_center.point, *plan.planned_grasp_jaws]
        delta = np.asarray([(p.x-origin.x, p.y-origin.y, p.z-origin.z) for p in points])
        optical = delta @ camera_to_world
        if not np.isfinite(optical).all() or (optical[:, 2] <= 0).any():
            return None
        fx, fy = info.k[0], info.k[4]
        if not (fx > 0 and fy > 0):
            return None
        u = (optical[:, 0] * fx / optical[:, 2] + info.k[2]) / frame.width
        v = (optical[:, 1] * fy / optical[:, 2] + info.k[5]) / frame.height
        if not (0 <= u[0] <= 1 and 0 <= v[0] <= 1):
            return None
        return dict(track_id=str(plan.key.track_id), cycle_id=str(plan.key.cycle_id),
                    epoch=str(plan.key.epoch), points=np.column_stack((u, v)).tolist())

    def prepare_observe_frame(msg):
        if msg.encoding not in ('rgb8','bgr8') or msg.step < msg.width*3: raise ValueError('Unsupported gantry RGB encoding/stride')
        rgb = np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        return rgb.copy() if msg.encoding=='rgb8' else rgb[:,:,::-1].copy()
    def matching_depth(frame):
        if not node.depth:
            return None
        rgb_stamp = stamp(frame)
        depth = min(node.depth, key=lambda value: abs(stamp(value) - rgb_stamp))
        return depth if abs(stamp(depth) - rgb_stamp) / 1e9 <= settings['vision']['rgb_depth_sync_tolerance_sec'] else None

    def extract_target_surface(frame, instances, depth):
        if not node.depth or 'color' not in node.info or 'depth' not in node.info:
            return [[] for _ in instances], None, 'depth_unavailable'
        if depth is None:
            return [[] for _ in instances], None, 'depth_unsynchronized'
        if depth.encoding not in ('32FC1','16UC1'): return [[] for _ in instances],None,'depth_encoding_unavailable'
        try:
            color_world = transform(world,frame.header.frame_id,frame)
            depth_color = transform(frame.header.frame_id,depth.header.frame_id,frame)
        except Exception:
            return [[] for _ in instances],None,'tf_unavailable_at_capture'
        if not instances: return [],(color_world,node.info['color'].k),'available'
        dtype = np.dtype(('>' if depth.is_bigendian else '<') + ('f4' if depth.encoding=='32FC1' else 'u2'))
        values = np.frombuffer(depth.data,dtype=dtype).reshape(depth.height,depth.step//dtype.itemsize)[:,:depth.width].astype(float)
        if depth.encoding=='16UC1': values *= .001
        y,x = np.nonzero(np.isfinite(values)&(values>0))
        if len(x)>config['max_depth_points']:
            select=np.linspace(0,len(x)-1,config['max_depth_points'],dtype=int);x=x[select];y=y[select]
        z = values[y,x]
        intrinsics = node.info['depth'].k
        points = np.column_stack(((x-intrinsics[2])*z/intrinsics[0],(y-intrinsics[5])*z/intrinsics[4],z))
        points = points @ depth_color[:3,:3].T + depth_color[:3,3]
        positive=points[:,2]>0;points=points[positive]
        k=node.info['color'].k
        u=np.rint(points[:,0]*k[0]/points[:,2]+k[2]).astype(int)
        v=np.rint(points[:,1]*k[4]/points[:,2]+k[5]).astype(int)
        inside=(u>=0)&(u<frame.width)&(v>=0)&(v<frame.height)
        points=points[inside];u=u[inside];v=v[inside]
        # Nearest point per RGB pixel is the registered visible depth, not a hidden surface.
        order=np.argsort(points[:,2]);indices=v[order]*frame.width+u[order]
        _,first=np.unique(indices,return_index=True);order=order[first]
        points=points[order];u=u[order];v=v[order]
        output=[]
        for instance in instances:
            selected=points[instance['mask'][v,u]]
            if len(selected)>config['max_object_points']:
                selected=selected[np.linspace(0,len(selected)-1,config['max_object_points'],dtype=int)]
            output.append((selected @ color_world[:3,:3].T + color_world[:3,3]).tolist())
        return output,(color_world,k),'available'
    display_boxes = {}
    gantry_clouds = GantryCloudHistory(settings['vision']['foreground']['support_plane_z_m'] + .003)
    candidate_streaks = {}
    annotated_at = {}
    confirmation_sec = float(config.get('pick_confirmation_sec', 1.0))
    if not math.isfinite(confirmation_sec) or confirmation_sec <= 0:
        raise ValueError('pick_confirmation_sec must be finite and positive')
    confirmation_ns = round(confirmation_sec * 1e9)
    transit_tracks = {}
    transit_aliases = {}
    downstream_hint_sources = {}
    # Keep the pickup workspace separate from the gantry camera calibration.
    arm_settings = {
        arm: yaml.safe_load(selected_profile_path(project, arm).read_text())
        for arm in ('arm_01', 'arm_02')
    }
    workspace = arm_settings['arm_01']['motion']['workspace']
    entry_y = workspace['bounds_min_m'][1]
    wrist_classes = {
        arm: settings['inspection']['inference']['object_class_ids'][0]
        for arm, settings in arm_settings.items()
    }
    last_wrist_inference = dict.fromkeys(wrist_classes, 0.0)
    wrist_inference_hints = {}
    wrist_hint_age_ns = {'arm_01': _UPSTREAM_HINT_MAX_AGE_NS,
                         'arm_02': _DOWNSTREAM_HINT_MAX_AGE_NS}
    wrist_idle_hz = float(config.get('wrist_idle_inference_hz', 2.0))
    wrist_lead_sec = float(config.get('wrist_inference_lead_sec', 2.0))
    if not math.isfinite(wrist_idle_hz) or not 0 < wrist_idle_hz <= 1/_WRIST_ACTIVE_INFERENCE_PERIOD_SEC:
        raise ValueError('wrist_idle_inference_hz must be positive and at most the working rate')
    if not math.isfinite(wrist_lead_sec) or wrist_lead_sec <= 0:
        raise ValueError('wrist_inference_lead_sec must be finite and positive')
    wrist_idle_period = 1/wrist_idle_hz
    vision_max_age_ns = {
        arm: round(settings['vision']['frame_max_age_sec'] * 1e9)
        for arm, settings in arm_settings.items()
    }

    def update_upstream_release():
        if not node.upstream_gripper:
            return
        grip, grip_seen = node.upstream_gripper
        if time.monotonic()-grip_seen >= 1 or not grip.ready or not grip.stopped or not grip.empty_confirmed:
            return
        for track_id, (epoch, cycle) in node.upstream_claims.items():
            if track_id in node.upstream_released or grip.key.epoch != epoch:
                continue
            if node.upstream_outcomes.get((epoch, cycle, track_id)) in (3, 4, 5):
                node.upstream_released.add(track_id)

    def infer_wrist(arm, frame):
        result = DetectionArray()
        result.key = frame.key
        result.frame_id = frame.frame_id
        result.capture_stamp = frame.color.header.stamp
        result.model_version = descriptor['model_version']
        result.decision_source = 'model'
        result.complete = True
        image = Image.fromarray(prepare_observe_frame(frame.color))
        for instance in model.predict(image, max(0.5, config['confidence_threshold']))[:32]:
            if instance['confidence'] <= max(0.5, config['confidence_threshold']):
                continue
            detection = Detection()
            detection.class_id = wrist_classes[arm]
            detection.confidence = instance['confidence']
            detection.bbox_xyxy = list(instance['bbox_xyxy'])
            mask_bytes = io.BytesIO()
            Image.fromarray(np.asarray(instance['mask'], dtype=np.uint8) * 255).save(mask_bytes, format='PNG')
            detection.mask.format = 'png'
            detection.mask.data = mask_bytes.getvalue()
            result.detections.append(detection)
        node.wrist_detections[arm].publish(result)


    def encode(image, kind):
        stream=io.BytesIO();image.save(stream,format=kind,**({'quality':82} if kind=='JPEG' else {}))
        return base64.b64encode(stream.getvalue()).decode()
    wrist_preview_received = {}
    feedback_thread = threading.Thread(target=feedback_executor.spin, name='auto-pick-feedback')
    feedback_thread.start()
    try:
        while rclpy.ok():
            if commands:
                command=commands.popleft()
                if command.get('command')=='stop_run' and command.get('run_id')==run_id: break
                if command.get('command') == 'pick_clear_bin' and command.get('run_id') == run_id:
                    arm = command['arm']
                    task, seen = node.wrist_tasks.get(arm, (None, 0.0))
                    request_id = command['request_id']
                    if bin_clear_available(arm, task, seen):
                        clear_requests[arm] = (request_id, time.monotonic(),
                                               clear_clients[arm].call_async(Trigger.Request()), 0)
                    else:
                        emit(dict(event='pick_reply', arm=arm, request_id=request_id, accepted=False,
                                  message='Clear Bin requires live feedback and a stopped, empty, idle arm'))
                if command.get('command') in (
                    'pick_start', 'pick_continuous', 'pick_pause', 'pick_cancel', 'pick_home'
                ) and command.get('run_id') == run_id:
                    request_id = command['request_id']
                    arm = command.get('arm', 'arm_01')
                    task_status, task_seen = node.wrist_tasks.get(arm, (None, 0.0))
                    starting = command['command'] in ('pick_start', 'pick_continuous')
                    homing = command['command'] == 'pick_home'
                    # Return Home is a reset. It needs only the arm's command
                    # service and waits behind a request that is still in flight.
                    if arm in clear_requests or (starting and arm in overflow):
                        emit(dict(event='pick_reply', arm=arm, request_id=request_id, accepted=False,
                                  message='Wait for Clear Bin or clear the overflow before starting'))
                    elif homing and arm in task_requests:
                        commands.append(command)
                    elif (not homing and (arm in task_requests or not semantic_arm_ready(arm))) or (
                            homing and not task_clients[arm].service_is_ready()) or (starting and (
                        task_status is None or time.monotonic()-task_seen > 1 or task_status.phase != 0
                        or task_status.continuous_operation
                        or not node.input_times or time.monotonic()-node.input_times[-1] > config['stale_after_sec']
                    )):
                        emit(dict(event='pick_reply', arm=arm, request_id=request_id, accepted=False,
                                  message=f'{arm} semantic runtime is unavailable or busy'))
                    else:
                        request = TaskCommand.Request()
                        request.command = {
                            'pick_start': TaskCommand.Request.START,
                            'pick_continuous': TaskCommand.Request.START_CONTINUOUS,
                            'pick_pause': TaskCommand.Request.PAUSE_CONTINUOUS,
                            'pick_cancel': TaskCommand.Request.CANCEL,
                            'pick_home': TaskCommand.Request.RETURN_HOME,
                        }[command['command']]
                        task_requests[arm] = (request_id, time.monotonic(), task_clients[arm].call_async(request))
            rclpy.spin_once(node, timeout_sec=.01)
            now=time.monotonic()
            for arm, (request_id, sent, future) in list(task_requests.items()):
                if future.done():
                    del task_requests[arm]
                    try:
                        reply = future.result()
                        emit(dict(event='pick_reply', arm=arm, request_id=request_id, accepted=reply.accepted,
                                  message=reply.message))
                    except Exception as error:
                        emit(dict(event='pick_reply', arm=arm, request_id=request_id, accepted=None, message=str(error)))
                elif now-sent > 5:
                    del task_requests[arm]
                    future.cancel()
                    emit(dict(event='pick_reply', arm=arm, request_id=request_id, accepted=None,
                              message='ROS reply timed out. Check live task status before retrying.'))
            advance_bins(now)
            if now >= next_task_status:
                next_task_status = now + 0.25
                for arm in task_clients:
                    vision = node.vision_status.get(arm)
                    if vision and node.clock_ns is not None:
                        status = deepcopy(vision)
                        source_stamp = stamp(vision)
                        max_age = vision_max_age_ns[arm]
                        # A completed inference may precede delivery of its clock
                        # tick. Publish the latest eligible capture in this epoch.
                        model_stamp = next((captured for epoch, captured in reversed(model_frames)
                                            if epoch == node.epoch and 0 <= node.clock_ns - captured < max_age), None)
                        status.ready = (model_stamp is not None and abs(node.clock_ns - source_stamp) <= max_age)
                        if status.ready:
                            status.header.stamp.sec, status.header.stamp.nanosec = divmod(model_stamp, 10**9)
                        status.model_version = descriptor['model_version']
                        status.message = 'Observe model active' if status.ready else 'Waiting for fresh gantry inference'
                        node.inference_status[arm].publish(status)
                    task, seen = node.wrist_tasks.get(arm, (None, 0.0))
                    active = task if now-seen <= 1 else None
                    feedback = device_feedback(arm, active)
                    for placement in node.bin_load[arm].results(now):
                        if placement.get('overflow') and arm not in overflow:
                            overflow.add(arm)
                            save_overflow()
                        emit(dict(event='bin_placement', arm=arm, placement=placement))
                    last_result = None
                    if active and active.last_outcome:
                        key = active.last_completed_key
                        last_result = dict(epoch=str(key.epoch), cycle_id=str(key.cycle_id),
                                           track_id=str(key.track_id), outcome=active.last_outcome)
                    emit(dict(event='pick_status', arm=arm, available=bool(active and semantic_arm_ready(arm)),
                              task=None if active is None else dict(
                                  epoch=str(active.key.epoch), cycle_id=str(active.key.cycle_id),
                                  track_id=str(active.key.track_id), phase=active.phase, reason=active.reason,
                                  home_state=active.home_state, home_message=active.home_message,
                                  planned_grasp_active=bool(active.planned_grasp_center.header.frame_id),
                                  continuous_operation=active.continuous_operation, last_result=last_result),
                              bin_load=dict(node.bin_load[arm].details(now, config.get('bin_tare_n', {}).get(arm)),
                                            overflow=arm in overflow, clearing=arm in clear_requests,
                                            clear_available=bin_clear_available(arm, active, seen)),
                              **feedback))
                    frame, received = feedback_node.latest_wrist_rgb(arm)
                    grasp = None
                    rgbd, rgbd_received = node.wrist_preview_frames.get(arm, (None, 0.))
                    if (active and active.planned_grasp_center.header.frame_id
                            and rgbd is not None and now - rgbd_received <= .5):
                        grasp = planned_grasp_pixels(arm, rgbd, active)
                        if grasp is not None:
                            frame, received = rgbd.color, rgbd_received
                    if frame is not None and time.monotonic() - received <= config['stale_after_sec']:
                        if wrist_preview_received.get(arm) != received:
                            wrist_preview_received[arm] = received
                            try:
                                preview = Image.fromarray(prepare_observe_frame(frame))
                                height = config['preview_height_px']
                                preview.thumbnail((max(1, round(preview.width * height / preview.height)), height))
                                emit(dict(event='wrist_preview', run_id=run_id, arm=arm,
                                          grasp=grasp,
                                          jpeg=encode(preview, 'JPEG')))
                            except (ValueError, OSError):
                                pass
            for arm, (wrist_task, seen) in node.wrist_tasks.items():
                wrist_frame = node.wrist_frames.get(arm)
                if (now-seen >= 1 or wrist_task.phase not in (1, 3, 4) or wrist_frame is None
                        or now-last_wrist_inference[arm] < _WRIST_ACTIVE_INFERENCE_PERIOD_SEC
                        or not node.wrist_detections[arm].get_subscription_count()):
                    continue
                working = wrist_inference_working(
                    wrist_task.phase, wrist_frame.key.epoch, node.clock_ns, node.epoch,
                    wrist_inference_hints.get(arm, (None, 0, ())), vision_max_age_ns[arm],
                    wrist_hint_age_ns[arm], arm_settings[arm]['task'].get('target_selection_roi'),
                    wrist_lead_sec)
                period = _WRIST_ACTIVE_INFERENCE_PERIOD_SEC if working else wrist_idle_period
                if now-last_wrist_inference[arm] >= period:
                    last_wrist_inference[arm] = now
                    infer_wrist(arm, node.wrist_frames.pop(arm))
            if not node.pending_rgb or now-last_inference < 1/config['inference_hz']:
                if now-last_health>=1:
                    emit(dict(event='health',health=dict(source_state='waiting' if count==0 else 'waiting_for_new_frame',clock_epoch=str(node.epoch),input_hz=rate(node.input_times),inference_hz=rate(output_times))))
                    last_health=now
                continue
            selected = None
            depth = None
            for candidate in reversed(node.pending_rgb):
                depth = matching_depth(candidate[0])
                if depth is not None:
                    selected = candidate
                    break
            if selected is None:
                # Give delayed depth one bounded chance to pair. Keep the
                # RGB preview responsive when the depth stream is missing.
                selected = next((candidate for candidate in reversed(node.pending_rgb)
                                 if now - candidate[3] >= .2), None)
                if selected is None:
                    continue
            frame, sequence, epoch, received = selected
            while node.pending_rgb and node.pending_rgb[0][1] <= sequence:
                _, discarded_sequence, _, _ = node.pending_rgb.popleft()
                if discarded_sequence != sequence:
                    node.dropped += 1
            last_inference = now
            rgb=prepare_observe_frame(frame)
            image=Image.fromarray(rgb)
            inference_started=time.monotonic()
            instances = [remove_detached_mask_fragments(instance)
                         for instance in model.predict(image, config['confidence_threshold'])]
            model_frames.append((epoch, stamp(frame)))
            inference_ms=(time.monotonic()-inference_started)*1000
            points,camera_pose,reason=extract_target_surface(frame,instances,depth)
            payload=dict(epoch=str(epoch),stamp_sec=stamp(frame)/1e9,objects=[dict(label=obj['label'],bbox_xyxy=obj['bbox_xyxy'],points_world=cloud) for obj,cloud in zip(instances,points)])
            core.stdin.write(json.dumps(payload)+'\n');core.stdin.flush()
            tracks=json.loads(core.stdout.readline())
            overlay=np.zeros((frame.height,frame.width,4),dtype=np.uint8)
            objects=[]
            capture_ns = stamp(frame)
            display_boxes = {key: value for key, value in display_boxes.items()
                             if key[0] == epoch and 0 <= capture_ns - value[0] <= _MAX_FRAME_GAP_NS}
            annotated_at = {key: seen for key, seen in annotated_at.items()
                            if 0 <= capture_ns - seen <= _MAX_FRAME_GAP_NS}
            gantry_clouds.begin_frame(epoch, capture_ns)
            for instance,track,cloud in zip(instances,tracks,points):
                mask = instance['mask']
                key = (epoch, track['track_id'])
                gantry_clouds.add(*key, capture_ns, cloud, near_edge=mask_near_image_edge(mask))
                track_key = f'{run_id}:{epoch}:' + track.pop('track_id')
                edges, cached_box = display_box(display_bounds(cloud), track['velocity_world_mps'], mask,
                                                camera_pose, display_boxes.get(key), capture_ns,
                                                keep_size=track_key in annotated_at)
                if cached_box is not None:
                    display_boxes[key] = cached_box
                objects.append(dict(track_key=track_key,label=instance['label'],confidence=instance['confidence'],bbox_xyxy=instance['bbox_xyxy'],measurement_stamp_ns=str(stamp(frame)),geometry_reason=(reason if reason!='available' else 'no_valid_mask_depth') if track['position_world_m'] is None else 'available',box_edges_rgb_pixels=edges,box_estimated=bool(edges and cached_box[0] != capture_ns),**track))
            size=(max(1,round(frame.width*config['preview_height_px']/frame.height)),config['preview_height_px'])
            image=image.resize(size)
            count+=1
            output_times.append(time.monotonic())
            snapshot=dict(run_id=run_id,model_version=descriptor['model_version'],clock_epoch=str(epoch),frame_key=f'{run_id}_{epoch}_{sequence}',capture_stamp_ns=str(stamp(frame)),original_size=dict(width=frame.width,height=frame.height),preview_size=dict(width=size[0],height=size[1]),objects=objects)
            next_streaks = {}
            candidates = []
            for obj in objects:
                key = obj['track_key']
                qualified = (obj['confidence'] > max(0.5, config['confidence_threshold'])
                             and obj['geometry_reason'] == 'available')
                streak, obj['pick_confirmed'] = confirm_pick(
                    candidate_streaks.get(key), capture_ns, confirmation_ns, qualified)
                if streak is not None:
                    next_streaks[key] = streak
                if obj['pick_confirmed'] and obj.get('velocity_world_mps') is not None:
                    candidates.append(dict(
                        track_key=key, confidence=obj['confidence'],
                        position_world_m=obj['position_world_m'],
                        velocity_world_mps=obj['velocity_world_mps'],
                        measurement_stamp_ns=obj['measurement_stamp_ns'],
                        valid_for_sec=0.25,
                    ))
            present = {obj['track_key'] for obj in objects}
            for key, previous in candidate_streaks.items():
                if key not in present:
                    streak, _ = confirm_pick(previous, capture_ns, confirmation_ns, False)
                    if streak is not None:
                        next_streaks[key] = streak
            candidate_streaks = next_streaks
            for instance, obj in zip(instances, objects):
                key = obj['track_key']
                obj['display_confirmed'] = obj['pick_confirmed'] or key in annotated_at
                if obj['display_confirmed']:
                    overlay[instance['mask']] = [50, 230, 150, 105]
                    annotated_at[key] = capture_ns
                else:
                    obj['box_edges_rgb_pixels'] = []
                    obj['box_estimated'] = False
            alpha = Image.fromarray(overlay).resize(size, Image.Resampling.NEAREST)
            annotated_count = sum(obj['display_confirmed'] for obj in objects)
            snapshot['message'] = f'{annotated_count} target(s)' if annotated_count else 'No confirmed targets'
            if reason != 'available' and annotated_count:
                snapshot['message'] += ' · 2D only'
            snapshot['pick_candidates'] = candidates
            preview_tracks = TrackedObjectArray()
            preview_tracks.header.frame_id = settings['frames']['world']
            preview_tracks.header.stamp = frame.header.stamp
            downstream_tracks = TrackedObjectArray()
            downstream_tracks.header = preview_tracks.header
            update_upstream_release()
            for candidate in candidates[:256]:
                local_id = int(candidate['track_key'].rsplit(':', 1)[1])
                candidate['ros_track_id'] = str((int(run_id[:8], 16) << 32) | local_id)
                x, y, _ = candidate['position_world_m']
                # Straight-belt travel is a search prior; wrist RGB-D supplies measured velocity.
                vx, vy = 0.0, _BELT_SPEED_MPS
                if vy > 0 and y < entry_y:
                    seconds = (entry_y - y) / vy
                    entry_x = x + vx * seconds
                    if workspace['bounds_min_m'][0] <= entry_x <= workspace['bounds_max_m'][0]:
                        candidate['arm_01_arrival_preview'] = {
                            'seconds': seconds, 'entry_xy_world_m': [entry_x, entry_y]}

            # Preserve identity for the wrist search without refreshing measurement time.
            transit_tracks = {
                key: track for key, track in transit_tracks.items()
                if key.startswith(f'{run_id}:{epoch}:')
                and int(track.candidate['measurement_stamp_ns']) <= stamp(frame) <= track.deadline_ns
            }
            transit_aliases = {raw_key: key for raw_key, key in transit_aliases.items() if key in transit_tracks}
            eligible = [item for item in candidates if 0 < item.get('arm_01_arrival_preview', {}).get('seconds', 0) <= 20]
            update_transit_routes(transit_tracks, transit_aliases, eligible, temporary_loss_sec, association_distance_m)
            # A duplicate mask may qualify for the preview briefly. Require a
            # continuing gantry trajectory before it can compete for an arm ID.
            confirmed_transits = [track for track in transit_tracks.values() if track.confirmed]
            # These records are search hints. Their pose stamp stays at the
            # gantry measurement, even while the array heartbeat advances.
            active_ids = {int(track.candidate['ros_track_id']) for track in confirmed_transits}
            node.upstream_claims = {track_id: identity for track_id, identity in node.upstream_claims.items()
                                    if track_id in active_ids}
            node.upstream_released.intersection_update(active_ids)
            node.upstream_wrist_tracks = {track_id: item for track_id, item in node.upstream_wrist_tracks.items()
                                          if track_id in active_ids}
            downstream_hint_sources = {track_id: source for track_id, source in downstream_hint_sources.items()
                                       if track_id in active_ids}
            upstream_task, upstream_seen = node.wrist_tasks.get('arm_01', (None, 0.0))
            upstream_live = upstream_task is not None and time.monotonic()-upstream_seen < 1
            for track in confirmed_transits[-32:]:
                candidate = track.candidate
                hint = TrackedObject()
                hint.key.settings_revision = descriptor["model_version"]
                hint.key.track_id = int(candidate['ros_track_id'])
                hint.key.sequence = sequence
                hint.phase = TrackedObject.CANDIDATE
                hint.pose.header.frame_id = preview_tracks.header.frame_id
                measured_ns = int(candidate['measurement_stamp_ns'])
                hint.pose.header.stamp.sec, hint.pose.header.stamp.nanosec = divmod(measured_ns, 10**9)
                hint.pose.pose.orientation.w = 1.0
                position = hint.pose.pose.position
                position.x, position.y, position.z = candidate['position_world_m']
                velocity = hint.velocity_mps
                velocity.y = _BELT_SPEED_MPS
                hint.velocity_valid = True
                measured_speed = measured_transit_speed(track, settings['tracking'])
                if measured_speed is not None:
                    velocity.y, hint.velocity_fit_residual_m, hint.velocity_uncertainty_mps = measured_speed
                    hint.confirmations = track.confirmations
                age_ns = stamp(frame) - measured_ns
                # Observe can restart independently. The receiving vision
                # nodes assign ROS epochs, which are separate from UI epochs.
                upstream_epoch = node.wrist_epochs.get('arm_01')
                downstream_epoch = node.wrist_epochs.get('arm_02')
                if upstream_epoch and age_ns <= _UPSTREAM_HINT_MAX_AGE_NS and hint.key.track_id not in node.upstream_released:
                    hint.key.epoch = upstream_epoch
                    preview_tracks.objects.append(hint)
                if downstream_epoch and upstream_live and (hint.key.track_id not in node.upstream_claims
                                      or hint.key.track_id in node.upstream_released):
                    hint = deepcopy(hint)
                    hint.key.epoch = downstream_epoch
                    wrist = node.upstream_wrist_tracks.get(hint.key.track_id)
                    wrist = wrist[1:] if wrist and wrist[0] == upstream_epoch else None
                    anchor, anchor_ns, speed, source = downstream_transit_hint(
                        candidate['position_world_m'], measured_ns, velocity.y, wrist,
                        stamp(frame), association_distance_m)
                    hint.velocity_mps.y = speed
                    hint.pose.header.stamp.sec, hint.pose.header.stamp.nanosec = divmod(anchor_ns, 10**9)
                    anchored = hint.pose.pose.position
                    anchored.x, anchored.y, anchored.z = anchor
                    if source != downstream_hint_sources.get(hint.key.track_id):
                        downstream_hint_sources[hint.key.track_id] = source
                        node.get_logger().info(
                            f'downstream hint: track={hint.key.track_id} source={source} '
                            f'stamp_ns={anchor_ns} speed_mps={speed:.5f} gantry_speed_mps={velocity.y:.5f}')
                    downstream_tracks.objects.append(hint)
            node.pick_preview_pub.publish(preview_tracks)
            node.downstream_preview_pub.publish(downstream_tracks)
            wrist_inference_hints['arm_01'] = (epoch, capture_ns, preview_tracks.objects)
            wrist_inference_hints['arm_02'] = (epoch, capture_ns, downstream_tracks.objects)
            cloud_header = deepcopy(frame.header)
            cloud_header.frame_id = world
            for obj in objects:
                if not obj['display_confirmed']:
                    continue
                raw_key = obj['track_key']
                route_key = transit_aliases.get(raw_key, raw_key)
                raw_id = int(raw_key.rsplit(':', 1)[1])
                display_id = int(route_key.rsplit(':', 1)[1])
                prefix = int(run_id[:8], 16) << 32
                obj.update(ros_track_id=str(prefix | display_id), display_id=str(display_id),
                           raw_ros_track_id=str(prefix | raw_id))
                fused = gantry_clouds.fused(epoch, str(raw_id), capture_ns, obj['velocity_world_mps'])
                if fused is None:
                    continue
                latest, accumulated, scan_count = fused
                cloud_key = EventKey(epoch=epoch+1, track_id=prefix | display_id, sequence=sequence,
                                     settings_revision=descriptor['model_version'])
                node.gantry_cloud_pub.publish(ObjectCloudPreview(
                    header=cloud_header, key=cloud_key, revision=sequence, scan_count=scan_count,
                    source_camera='gantry', reason='capture-time belt translation',
                    latest=create_cloud_xyz32(cloud_header,latest),
                    accumulated=create_cloud_xyz32(cloud_header,accumulated)))
                def preview_points(points):
                    indices = np.linspace(0, len(points)-1, min(len(points), 450), dtype=int)
                    return points[indices].astype(float).round(4).tolist()

                emit(dict(event='gantry_surface', run_id=run_id, epoch=str(epoch),
                          track_id=str(prefix | display_id), scans=scan_count, revision=sequence,
                          capture_stamp_ns=str(capture_ns),
                          latest=preview_points(latest), accumulated=preview_points(accumulated)))
            snapshot['transit_tracks'] = [
                dict(track.candidate, state='awaiting_wrist_confirmation',
                     expires_stamp_ns=str(track.deadline_ns))
                for track in confirmed_transits
            ]


            health=dict(source_state='live',depth_state=reason,tf_state='available' if camera_pose else 'unavailable',input_hz=rate(node.input_times),inference_hz=rate(output_times),inference_ms=inference_ms,dropped_frame_count=node.dropped,clock_epoch=str(epoch),processing_age_sec=time.monotonic()-received)
            emit(dict(event='frame',run_id=run_id,snapshot=snapshot,rgb=encode(image,'JPEG'),mask=encode(alpha,'PNG'),health=health))
    finally:
        feedback_executor.shutdown()
        feedback_thread.join()
        feedback_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        core.stdin.close()
        try: core.wait(timeout=3)
        except subprocess.TimeoutExpired: core.terminate();core.wait()
        core.stdout.close()
