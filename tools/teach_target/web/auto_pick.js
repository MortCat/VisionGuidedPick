'use strict';

const phases = ['Idle', 'Observing', '—', 'Preparing pick', 'Picking', 'Transporting', 'Placing', 'Reporting', 'Recovering', 'Fault'];
const outcomes = ['—', 'Placement unverified', 'No defect', 'Inconclusive', 'Canceled', 'Failed'];
const possessionLabels = {empty: 'Empty', holding: 'Holding', unknown: 'Unknown'};
const homeInProgress = [1, 2, 3];
const commandRoutes = {start: 'continuous', stop: 'cancel', home: 'home', clearBin: 'clear-bin'};
const commandLabels = {pick_continuous: 'Start', pick_cancel: 'Stop', pick_home: 'Return Home', pick_clear_bin: 'Clear Bin'};
const source = document.getElementById('source');
const cards = {};
let runId, pollBusy = false;

async function request(path, options) {
    const response = await fetch(path, {cache: 'no-store', ...options});
    const data = await response.json();
    if (!response.ok) throw Error(data.error?.message || `HTTP ${response.status}`);
    return data;
}

const torqueBarRangeNm = 30;
const binContentDisplayGraceMs = 2000;

function createJointRows(fields) {
    return Array.from({length: 9}, (_, index) => {
        const linear = index >= 7;
        const number = linear ? index - 6 : index + 1;
        const body = fields[linear ? 'finger-joint-rows' : 'arm-joint-rows'];
        const row = body.insertRow();
        const heading = document.createElement('th');
        heading.scope = 'row';
        heading.textContent = linear ? (number === 1 ? 'Left Finger' : 'Right Finger') : `J${number}`;
        row.append(heading);

        const position = row.insertCell();
        const effortCell = row.insertCell();
        const effort = document.createElement('span');
        effort.textContent = position.textContent = '—';
        effortCell.append(effort);

        let fill = null;
        if (!linear) {
            const bar = document.createElement('span');
            bar.className = 'torque-bar';
            bar.setAttribute('aria-hidden', 'true');
            fill = document.createElement('span');
            bar.append(fill);
            effortCell.append(bar);
        }

        return {name: `panda_${linear ? 'finger_' : ''}joint${number}`, linear, position, effort, fill};
    });
}

function renderDetails(card) {
    if (card.fields['joint-details'].hidden) return;
    const contents = card.lastBinContent;
    const drop = card.binLoad?.last_drop_g;
    const binReadingDelayed = contents && performance.now() - contents.receivedAt >= binContentDisplayGraceMs;
    card.fields['bin-contents'].textContent = contents
        ? `${Math.round(contents.grams)} g${binReadingDelayed ? ' (last known)' : ''}` : 'Unavailable';
    const roundedDrop = Math.round(drop);
    card.fields['last-drop'].textContent = Number.isFinite(drop)
        ? `${roundedDrop > 0 ? '+' : ''}${roundedDrop} g` : 'Unavailable';
    const feedback = card.jointFeedback;
    const joints = feedback?.fresh ? feedback.joints : [];
    const byName = new Map(joints.map(joint => [joint.name, joint]));
    const status = card.fields['joint-status'];
    const delayed = feedback?.fresh === false && feedback.issue && feedback.issue !== 'No feedback received';
    const label = joints.length ? 'Live' : delayed ? 'Delayed' : 'No feedback';
    if (status.textContent !== label) status.textContent = label;
    status.dataset.state = label;
    status.title = feedback?.issue || '';

    for (const {name, linear, position, effort, fill} of card.jointRows) {
        const joint = byName.get(name);
        position.textContent = Number.isFinite(joint?.position)
            ? (joint.position * (linear ? 1000 : 180 / Math.PI)).toFixed(2) : '—';
        const value = joint?.effort;
        effort.textContent = Number.isFinite(value)
            ? `${value > 0 ? '+' : ''}${value.toFixed(2)}` : '—';
        if (fill) {
            fill.dataset.direction = value < 0 ? 'negative' : 'positive';
            const fraction = Number.isFinite(value) ? Math.min(1, Math.abs(value) / torqueBarRangeNm) : 0;
            fill.style.width = `${fraction * 50}%`;
        }
    }
}

function renderGrasp(card) {
    const fields = card.fields;
    const image = fields.image;
    const preview = card.preview;
    const points = preview?.live && preview.frame_id === card.loadedFrameId && preview.grasp?.points;
    const show = fields.view.value === 'rgb' && points?.length === 3 && !image.hidden;
    fields['grasp-overlay'].toggleAttribute('hidden', !show);
    fields['grasp-label'].hidden = !show;
    if (!show || card.graspFrameId === preview.frame_id) return;
    card.graspFrameId = preview.frame_id;

    const {naturalWidth: width, naturalHeight: height} = image;
    fields['grasp-overlay'].setAttribute('viewBox', `0 0 ${width} ${height}`);
    const [[x, y], [ax, ay], [bx, by]] = points.map(([u, v]) => [u * width, v * height]);
    const span = Math.hypot(bx - ax, by - ay);
    const dx = span ? -(by - ay) * 6 / span : 0;
    const dy = span ? (bx - ax) * 6 / span : 0;
    fields['grasp-jaws'].setAttribute('d', `M${ax} ${ay} L${bx} ${by} M${ax-dx} ${ay-dy} l${2*dx} ${2*dy} M${bx-dx} ${by-dy} l${2*dx} ${2*dy}`);
    fields['grasp-center'].setAttribute('d', `M${x-5} ${y} h10 M${x} ${y-5} v10`);
}

function renderArm(card, pick, observe) {
    card.jointFeedback = pick.joint_feedback;
    card.binLoad = pick.bin_load;
    const contents = card.binLoad?.contents_g;
    if (Number.isFinite(contents)) {
        card.lastBinContent = {grams: contents, receivedAt: performance.now()};
    }
    renderDetails(card);
    const {task, gripper, arm_feedback: feedback, wrist_camera: camera} = pick;
    if (task) card.lastTask = task;
    if (gripper?.fresh) card.lastGripper = gripper;
    if (feedback?.fresh) card.lastArm = feedback;
    const displayedTask = task || card.lastTask;
    const shownGrip = card.lastGripper;
    const shownArm = card.lastArm;
    const showCloud = card.fields.view.value === 'cloud';
    card.preview = camera;
    card.fields.image.hidden = showCloud || !card.loadedFrameId;
    card.fields.cloud.hidden = !showCloud;
    if (showCloud) {
        const revision = (runId && pick.gantry_cloud_revision) || null;
        if (revision !== card.cloudRevision || (!revision && pick.gantry_cloud)
                || (pick.gantry_cloud && !card.cloud)) {
            card.cloud = runId && pick.gantry_cloud;
            card.cloudRevision = revision;
            renderCloud(card.fields.cloud, card.cloud);
        }
        const cloud = card.cloud;
        card.fields.camera.textContent = cloud ? `${cloud.scans} scans · Last gantry capture`
            : [3, 4, 5, 6].includes(task?.phase) && task.track_id !== '0'
                ? 'No gantry point cloud for this target' : 'Waiting for target';
    } else {
        card.fields.camera.textContent = camera?.live ? 'Live'
            : card.fields.image.hidden ? 'Waiting for camera' : 'Last frame · Camera delayed';
        if (camera?.frame_id && camera.frame_id !== card.frameId && !card.imagePending) {
            card.frameId = camera.frame_id;
            card.imagePending = true;
            card.fields.image.src = `/api/auto-pick/wrist-preview?arm=${card.arm}&frame=${camera.frame_id}`;
        }
    }
    renderGrasp(card);
    const busyHome = homeInProgress.includes(task?.home_state);
    const locked = card.busy || !runId || !pick.available || pick.command?.pending;
    const running = task?.continuous_operation || task?.phase !== 0;
    const bin = pick.bin_load;
    const clear = pick.command?.command === 'pick_clear_bin' ? pick.command : null;
    const clearing = bin?.clearing || clear?.pending;
    card.actions.start.disabled = locked || busyHome || clearing || bin?.overflow
        || observe.source_state !== 'live' || running;
    card.actions.clearBin.disabled = locked || busyHome || clearing || !bin?.clear_available;
    card.fields['bin-status'].textContent = clearing ? 'Clearing · checking bin weight…'
        : clear?.accepted === false ? clear.message
        : bin?.overflow ? (running ? 'Overflow · pausing after current task' : 'Overflow · clear bin to resume')
        : clear?.accepted ? clear.message : '';
    card.fields['bin-status'].dataset.overflow = String(Boolean(bin?.overflow));
    card.actions.stop.disabled = locked || (!running && !busyHome);
    // Return Home is a reset and stays available while Observe is running.
    card.actions.home.disabled = card.busy || !runId || busyHome || clearing;
    const last = ' (last known)';
    card.fields.phase.textContent = displayedTask
        ? phases[displayedTask.phase] + (task ? '' : last) : '—';
    card.fields.track.textContent = displayedTask?.track_id && displayedTask.track_id !== '0' ? displayedTask.track_id : '—';
    card.fields.gripper.textContent = shownGrip
        ? (possessionLabels[shownGrip.possession] || 'Unknown') + (gripper?.fresh ? '' : last) : '—';
    card.fields.opening.textContent = Number.isFinite(shownGrip?.width_m)
        ? `${(shownGrip.width_m * 1000).toFixed(1)} mm${gripper?.fresh ? '' : last}` : '—';
    card.fields.feedback.textContent = shownArm
        ? (!shownArm.ready ? 'Not ready' : shownArm.stopped ? 'Stopped' : 'Moving')
            + (feedback?.fresh ? '' : last) : '—';
    const outcome = displayedTask?.last_result;
    const placement = pick.placement;
    const sameCycle = placement && (!outcome || ['epoch', 'cycle_id', 'track_id'].every(
        key => placement[key] === outcome[key]));
    card.fields.result.textContent = sameCycle
        ? {placing: 'Checking bin load', placed: 'Placed', unverified: 'Placement unverified'}[placement.state]
        : outcomes[outcome?.outcome ?? pick.outcome] || '—';
    card.fields.result.title = sameCycle
        ? [outcomes[outcome?.outcome], placement.reason,
            Number.isFinite(placement.delta_n) ? `${placement.delta_n.toFixed(3)} N` : ''].filter(Boolean).join(' · ')
        : '';
    const label = commandLabels[pick.command?.command];
    let status = card.error || '';
    if (!status && card.busy) status = 'Sending…';
    if (!status && label && pick.command.pending) status = `${label} pending…`;
    if (!status && label && pick.command.accepted === false) status = `${label} rejected`;
    if (!status && label && pick.command.accepted === null) status = `${label} unconfirmed`;
    if (!status && bin?.overflow) status = 'Bin overflow';
    if (!status && busyHome) status = 'Returning home…';
    if (!status && task?.home_state === 5) status = 'Return Home failed';
    if (!status && !task) status = 'Arm status unavailable';
    if (!status && (!feedback?.fresh || !gripper?.fresh)) status = 'Feedback unavailable';
    card.fields.status.textContent = status;
    card.fields.status.title = card.errorDetail || pick.command?.message || feedback?.issue || gripper?.issue || '';
}

async function refresh() {
    if (pollBusy) return;
    pollBusy = true;
    try {
        const observe = await request('/api/observe/status');
        const liveRun = ['observing', 'waiting_for_frame'].includes(observe.run?.state);
        runId = liveRun ? observe.run.run_id : null;
        source.textContent = runId ? `Gantry camera · ${observe.source_state}` : 'Start Observe first';
        await Promise.all(Object.entries(cards).map(async ([arm, card]) => {
            try {
                const query = new URLSearchParams({arm});
                if (card.fields.view.value === 'cloud') {
                    query.set('point_cloud', 'true');
                    if (card.cloudRevision) query.set('cloud_revision', card.cloudRevision);
                }
                renderArm(card, await request(`/api/auto-pick/status?${query}`), observe);
            } catch {
                renderArm(card, {}, observe);
            }
        }));
    } catch (error) {
        runId = null;
        source.textContent = error.message;
        for (const card of Object.values(cards)) renderArm(card, {}, {});
    } finally {
        pollBusy = false;
    }
}

async function command(arm, action) {
    const card = cards[arm];
    if (!runId || card.busy || card.actions[action].disabled) return;
    card.busy = true;
    card.error = '';
    card.errorDetail = '';
    card.fields.status.textContent = 'Sending…';
    for (const button of Object.values(card.actions)) button.disabled = true;
    try {
        const key = Array.from(crypto.getRandomValues(new Uint8Array(16)), value => value.toString(16).padStart(2, '0')).join('');
        await request(`/api/auto-pick/${commandRoutes[action]}?arm=${arm}`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json', 'Idempotency-Key': key},
            body: JSON.stringify({run_id: runId}),
        });
    } catch (error) {
        card.error = `${card.actions[action].textContent} request failed`;
        card.errorDetail = error.message;
    } finally {
        card.busy = false;
        refresh();
    }
}

for (const [arm, label] of Object.entries({arm_01: 'Arm 1', arm_02: 'Arm 2'})) {
    const element = document.getElementById('pick-arm-template').content.firstElementChild.cloneNode(true);
    const fields = Object.fromEntries(Array.from(element.querySelectorAll('[data-field]'), field => [field.dataset.field, field]));
    const actions = Object.fromEntries(Array.from(element.querySelectorAll('[data-action]'), button => [button.dataset.action, button]));
    fields.title.textContent = label;
    for (const [action, button] of Object.entries(actions)) button.onclick = () => command(arm, action);
    const card = cards[arm] = {arm, fields, actions, busy: false};
    const toggle = fields['details-toggle'];
    const details = fields['joint-details'];
    details.id = `${arm}-joint-details`;
    toggle.setAttribute('aria-controls', details.id);
    card.jointRows = createJointRows(fields);
    toggle.onclick = () => {
        details.hidden = !details.hidden;
        toggle.setAttribute('aria-expanded', String(!details.hidden));
        renderDetails(card);
    };
    fields.view.onchange = () => {
        fields.image.hidden = fields.view.value === 'cloud' || !card.loadedFrameId;
        fields.cloud.hidden = fields.view.value !== 'cloud';
        renderGrasp(card);
        refresh();
    };
    fields.image.onload = () => {
        card.imagePending = false;
        card.loadedFrameId = card.frameId;
        fields.image.hidden = fields.view.value === 'cloud';
        renderGrasp(card);
    };
    fields.image.onerror = () => {
        card.imagePending = false;
        card.loadedFrameId = null;
        fields.image.hidden = true;
        renderGrasp(card);
    };
    document.getElementById('pick-arms').append(element);
}
refresh();
setInterval(refresh, 250);
