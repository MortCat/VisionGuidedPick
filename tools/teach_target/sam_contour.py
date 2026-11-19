"""Single SAM 2 + boundary GrabCut annotation pipeline and its owned worker."""
import contextlib
import importlib.util
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]


class ContourUnavailable(RuntimeError):
    pass


class ContourClient:
    def __init__(self, config):
        self.config = config
        self.lock = threading.Lock()
        self.process = None

    def close(self):
        with self.lock:
            self._close()

    def _close(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
            self.process.stdin.close()
            self.process.stdout.close()
            self.process = None

    def propose(self, path, prompt):
        with self.lock:
            if self.process is None or self.process.poll() is not None:
                self._close()
                python = self.config['python']
                if not Path(python).is_file():
                    raise ContourUnavailable('Annotation Python is missing; run setup_sam.py')
                log = ROOT / 'logs/teach_target/sam-worker.log'
                log.parent.mkdir(parents=True, exist_ok=True)
                env = os.environ.copy()
                env.pop('PYTHONPATH', None)
                env.pop('PYTHONHOME', None)
                self.responses = queue.Queue()
                with log.open('a') as stream:
                    self.process = subprocess.Popen(
                        [python, '-u', str(Path(__file__).resolve()), '--worker'],
                        cwd=ROOT, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=stream, text=True, encoding='utf-8', start_new_session=True)
                output = self.process.stdout
                responses = self.responses
                def read():
                    try:
                        for line in output:
                            responses.put(json.loads(line))
                    except (ValueError, OSError) as error:
                        responses.put({'error': str(error)})
                    finally:
                        responses.put({'error': 'Annotation worker exited; inspect logs/teach_target/sam-worker.log'})
                threading.Thread(target=read, daemon=True).start()
            started = time.monotonic()
            try:
                self.process.stdin.write(json.dumps({'path': str(path), 'prompt': prompt, 'config': self.config}) + '\n')
                self.process.stdin.flush()
                response = self.responses.get(timeout=self.config['timeout_sec'])
            except (BrokenPipeError, OSError, queue.Empty) as error:
                self._close()
                raise ContourUnavailable('SAM did not respond; inspect logs/teach_target/sam-worker.log') from error
            if 'invalid_prompt' in response:
                raise ValueError(response['invalid_prompt'])
            if 'error' in response:
                self._close()
                raise ContourUnavailable(response['error'])
            response['elapsed_ms'] = round((time.monotonic() - started) * 1000, 2)
            return response


def refine_boundary(rgb, initial, prompt, config):
    import cv2
    import numpy as np
    mask = initial.astype('uint8')
    radius = config['boundary_radius_px']
    kernel = np.ones((radius * 2 + 1, radius * 2 + 1), np.uint8)
    inside = cv2.erode(mask, kernel) > 0
    outside = cv2.dilate(mask, kernel) == 0
    labels = np.where(mask, cv2.GC_PR_FGD, cv2.GC_PR_BGD).astype(np.uint8)
    labels[inside] = cv2.GC_FGD
    labels[outside] = cv2.GC_BGD
    for key, value in [('positive_points', cv2.GC_FGD), ('negative_points', cv2.GC_BGD)]:
        for x, y in prompt.get(key, []):
            cv2.circle(labels, (round(x), round(y)), config['seed_radius_px'], value, -1)
    if not inside.any() or not outside.any():
        return mask, 'sam_only_no_boundary_samples'
    cv2.setNumThreads(config['threads'])
    cv2.grabCut(rgb, labels, None, np.zeros((1, 65), np.float64),
                np.zeros((1, 65), np.float64), config['iterations'], cv2.GC_INIT_WITH_MASK)
    return np.isin(labels, [cv2.GC_FGD, cv2.GC_PR_FGD]).astype(np.uint8), 'boundary_refined'


def worker():
    import math
    import traceback
    predictor = None
    cached_image = None
    for line in sys.stdin:
        try:
            request = json.loads(line)
            cfg = request['config']
            with contextlib.redirect_stdout(sys.stderr):
                for key in ['dependencies', 'source']:
                    path = str((ROOT / cfg[key]).resolve())
                    if path not in sys.path:
                        sys.path.insert(0, path)
                import torch
                import numpy as np
                from PIL import Image
                spec = importlib.util.spec_from_file_location('contour_rle', Path(__file__).with_name('contour.py'))
                rle = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(rle)
                path = Path(request['path'])
                rgb = np.array(Image.open(path).convert('RGB'), copy=True)
                height, width = rgb.shape[:2]
                prompt = request['prompt']
                points, labels = [], []
                for key, label in [('positive_points', 1), ('negative_points', 0)]:
                    values = prompt.get(key, [])
                    if not isinstance(values, list) or len(values) > 128:
                        raise ValueError('Too many correction points')
                    for point in values:
                        if not isinstance(point, list) or len(point) != 2 or any(type(v) not in (int, float) or not math.isfinite(v) for v in point):
                            raise ValueError('Correction points must be finite image coordinates')
                        if not (0 <= point[0] < width and 0 <= point[1] < height):
                            raise ValueError('Correction point is outside the image')
                        points.append(point)
                        labels.append(label)
                box = prompt.get('bbox_xyxy')
                if box is None and 'initial_mask' in prompt:
                    prior = rle.decode_rle(prompt['initial_mask'], width, height)
                    y, x = np.where(prior)
                    if len(x):
                        box = [max(0, int(x.min()) - 12), max(0, int(y.min()) - 12), min(width, int(x.max()) + 13), min(height, int(y.max()) + 13)]
                if not isinstance(box, list) or len(box) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) for v in box):
                    raise ValueError('Draw a rough box around the target first')
                if not (0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height):
                    raise ValueError('Box must fit inside the original image')
                if predictor is None:
                    from sam2.build_sam import build_sam2
                    from sam2.sam2_image_predictor import SAM2ImagePredictor
                    checkpoint = ROOT / cfg['checkpoint']
                    if not checkpoint.is_file():
                        raise RuntimeError('SAM checkpoint is missing; run setup_sam.py')
                    torch.set_num_threads(cfg['threads'])
                    predictor = SAM2ImagePredictor(build_sam2(cfg['model_config'], str(checkpoint), device=cfg['device']))
                identity = (str(path.resolve()), path.stat().st_mtime_ns, path.stat().st_size)
                reused = identity == cached_image
                sam_started = time.monotonic()
                with torch.inference_mode():
                    if not reused:
                        predictor.set_image(rgb)
                        cached_image = identity
                    masks, scores, _ = predictor.predict(
                        point_coords=np.asarray(points, np.float32) if points else None,
                        point_labels=np.asarray(labels, np.int32) if points else None,
                        box=np.asarray(box, np.float32), multimask_output=True)
                mask = masks[int(np.argmax(scores))].astype(np.uint8)
                if not mask.any():
                    raise ValueError('No object found; add an Include point inside the target')
                sam_ms = round((time.monotonic() - sam_started) * 1000, 2)
                refined, status = refine_boundary(rgb, mask, prompt, cfg)
                response = {'mask': rle.encode_rle(refined), 'sam_mask': rle.encode_rle(mask),
                            'approved': False, 'method': 'sam2_grabcut', 'embedding_reused': reused,
                            'sam_ms': sam_ms, 'refinement': status}
            print(json.dumps(response), flush=True)
        except ValueError as error:
            print(json.dumps({'invalid_prompt': str(error)}), flush=True)
        except Exception as error:
            traceback.print_exc(file=sys.stderr)
            print(json.dumps({'error': 'SAM annotation unavailable: ' + str(error)}), flush=True)


if __name__ == '__main__':
    worker()
