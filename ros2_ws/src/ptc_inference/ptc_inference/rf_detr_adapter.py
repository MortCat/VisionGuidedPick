"""Single loaded model; both stream detection and requested views reuse it."""
import io
import math


class RfDetrAdapter:
    def __init__(self, config):
        from rfdetr import RFDETR
        self.config = config
        # Never opt in to arbitrary pickle execution for a supplied checkpoint.
        self.model = RFDETR.from_checkpoint(
            config["model_path"], trust_checkpoint=False, device=config["device"]
        )
        if list(self.model.class_names) != config["model_class_names"]:
            raise ValueError("checkpoint class names differ from Settings class-index mapping")

    def predict(self, frame):
        import numpy as np
        from PIL import Image
        image = frame.color
        if image.encoding not in ("rgb8", "bgr8") or image.step < image.width * 3:
            raise ValueError("inference requires RGB8/BGR8 with valid stride")
        if len(image.data) != image.step * image.height:
            raise ValueError("invalid RGB payload length")
        rgb = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)
        rgb = rgb[:, :image.width * 3].reshape(image.height, image.width, 3)
        if image.encoding == "bgr8":
            rgb = rgb[:, :, ::-1]
        output = self.model.predict(
            Image.fromarray(rgb.copy()), threshold=self.config["candidate_confidence_threshold"]
        )
        if len(output) > self.config["max_detections_per_frame"]:
            raise ValueError("model output exceeds evidence capacity; not silently truncated")
        detections = []
        mask_bytes = 0
        masks = getattr(output, "mask", None)
        for index, (class_index, confidence, bbox) in enumerate(
            zip(output.class_id, output.confidence, output.xyxy)
        ):
            class_index = int(class_index)
            if not 0 <= class_index < len(self.config["model_class_names"]):
                raise ValueError("unknown model class index")
            confidence = float(confidence)
            box = [float(value) for value in bbox]
            if not math.isfinite(confidence) or not 0 <= confidence <= 1 or not all(map(math.isfinite, box)):
                raise ValueError("non-finite model result")
            if not (0 <= box[0] <= box[2] <= image.width and 0 <= box[1] <= box[3] <= image.height):
                raise ValueError("bbox is not in original image coordinates")
            label = self.config["model_class_names"][class_index]
            mask = b""
            if masks is not None:
                if masks[index].shape != (image.height, image.width):
                    raise ValueError("mask must match original RGB dimensions")
                encoded = io.BytesIO()
                Image.fromarray(masks[index].astype(np.uint8) * 255).save(encoded, format="PNG")
                mask = encoded.getvalue()
                mask_bytes += len(mask)
                if mask_bytes + len(frame.color.data) + len(frame.depth.data) > self.config["max_frame_megabytes"] * 1024 * 1024:
                    raise ValueError("mask evidence exceeds per-frame byte capacity")
            detections.append((label, label in self.config["defect_class_ids"], confidence, box, mask))
        return detections
