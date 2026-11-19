"""CPU regression for the boundary-only refinement contract; no model download."""
from pathlib import Path
import importlib.util
import numpy as np
import cv2

spec = importlib.util.spec_from_file_location('sam_contour', Path(__file__).with_name('sam_contour.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
mask = np.zeros((180, 240), np.uint8)
mask[40:140, 50:190] = 1
mask[60:90, 90:110] = 0
rgb = np.full((180, 240, 3), 25, np.uint8)
rgb[mask > 0] = [195, 158, 111]
cfg = dict(boundary_radius_px=4, seed_radius_px=2, threads=2, iterations=3)
output, status = m.refine_boundary(rgb, mask, {}, cfg)
kernel = np.ones((9, 9), np.uint8)
assert output[cv2.erode(mask, kernel) > 0].all()
assert not output[cv2.dilate(mask, kernel) == 0].any()
assert status == 'boundary_refined'
output, _ = m.refine_boundary(rgb, mask, {'negative_points': [[60, 60]]}, cfg)
assert output[60, 60] == 0
thin = np.zeros_like(mask)
thin[20, 20] = 1
output, status = m.refine_boundary(rgb, thin, {}, cfg)
assert np.array_equal(output, thin)
assert status == 'sam_only_no_boundary_samples'
print('SAM boundary regression passed: fixed core/background, hole, exclusion seed, thin-object preservation')
