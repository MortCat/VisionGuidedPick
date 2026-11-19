"""One RF-DETR loader for published Teach checkpoints and Observe inference."""
import hashlib
from pathlib import Path


class ObserveModelAdapter:
    def __init__(self, descriptor, device):
        from rfdetr import RFDETR
        checkpoint = Path(descriptor['checkpoint_path'])
        with checkpoint.open('rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != descriptor['checkpoint_sha256']:
                raise ValueError('Checkpoint changed before the inference worker loaded it')
        # Older BestModelCallback exports omit model_config. Resolve resize from the
        # verified model bundle, never from the preview or a UI request.
        resolution = descriptor['preprocessing']['resolution']
        self.model = RFDETR.from_checkpoint(str(checkpoint), trust_checkpoint=False,
                                          device=device, resolution=resolution)
        self.model.model.model.eval()
        self.mapping = descriptor['class_mapping']
        self.contract = dict(model_version=descriptor['model_version'],
                             checkpoint_sha256=descriptor['checkpoint_sha256'],
                             resolution=resolution, color_order='RGB', dtype='float32',
                             preprocessing='RFDETR.predict PIL RGB; library resize/normalization',
                             class_mapping=self.mapping, mask_space='original_rgb_pixels')

    def predict(self, image, threshold):
        import torch
        with torch.inference_mode():
            result = self.model.predict(image, threshold=threshold, include_source_image=False)
        instances = []
        for index, (category, score, box) in enumerate(zip(result.class_id, result.confidence, result.xyxy)):
            if not float(score) > threshold:
                continue
            label = self.mapping.get(str(int(category)))
            # Unknown output classes are not silently relabelled as the taught target.
            if label is None:
                continue
            if result.mask is None or result.mask[index].shape != (image.height, image.width):
                raise ValueError('Model must return masks in original RGB coordinates')
            instances.append(dict(label=label, class_id=str(int(category)), confidence=float(score),
                                  bbox_xyxy=[float(x) for x in box], mask=result.mask[index].astype(bool)))
        return instances
