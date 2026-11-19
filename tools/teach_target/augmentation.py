"""Build copy-paste samples exclusively from the frozen training split."""
import math
import random

import numpy as np
from PIL import Image
from pycocotools import mask as coco_mask


def augment_training(rows, config, directory):
    settings = config.get("augmentation", {})
    fractions = settings.get("foreground_fractions", [])
    backgrounds = [row for row in rows if not row["annotation"]["instances"]]
    if not fractions or not backgrounds:
        return []

    rng = random.Random(config["seed"])
    directory.mkdir()
    samples = []
    for row in rows:
        with Image.open(row["full_path"]) as source:
            for index, instance in enumerate(row["annotation"]["instances"]):
                rle = instance["mask"]
                mask = Image.fromarray(
                    coco_mask.decode(coco_mask.frPyObjects(rle, row["height"], row["width"])) * 255
                )
                box = mask.getbbox()
                if box is None:
                    continue
                cutout = source.convert("RGB").crop(box)
                cutout_mask = mask.crop(box)
                area = np.count_nonzero(np.asarray(cutout_mask))
                for band, fraction in enumerate(fractions):
                    background = rng.choice(backgrounds)
                    with Image.open(background["full_path"]) as image:
                        canvas = image.convert("RGB")
                    width, height = canvas.size
                    scale = min(
                        math.sqrt(fraction * width * height / area),
                        0.30 * width / cutout.width,
                        0.40 * height / cutout.height,
                    )
                    size = tuple(max(1, round(edge * scale)) for edge in cutout.size)
                    center_y = (0.72, 0.50, 0.28)[band]
                    position = (round(width * 0.5 - size[0] / 2),
                                round(height * center_y - size[1] / 2))
                    resized_mask = cutout_mask.resize(size, Image.Resampling.NEAREST)
                    if resized_mask.getbbox() is None:
                        continue
                    canvas.paste(cutout.resize(size, Image.Resampling.LANCZOS), position, resized_mask)
                    label = Image.new("L", canvas.size)
                    label.paste(resized_mask, position)
                    flat = (np.asarray(label).ravel(order="F") != 0)
                    boundaries = np.flatnonzero(flat[1:] != flat[:-1]) + 1
                    counts = np.diff(np.r_[0, boundaries, flat.size]).tolist()
                    if flat[0]:
                        counts.insert(0, 0)
                    sample_id = f"aug_{row['image_id']}_{index}_{band}"
                    path = directory / f"{sample_id}.png"
                    canvas.save(path)
                    samples.append({
                        "image_id": sample_id,
                        "full_path": str(path),
                        "width": width,
                        "height": height,
                        "source_image_id": row["image_id"],
                        "background_image_id": background["image_id"],
                        "annotation": {"instances": [{"mask": {"size": [height, width], "counts": counts}}]},
                    })
    return samples
