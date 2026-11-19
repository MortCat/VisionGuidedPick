"""Read actual Lightning CSV rows; merge sparse train/validation epoch records."""
import csv
import math

FIELDS = ('train/loss', 'val/loss', 'train/loss_ce', 'val/loss_ce',
          'train/loss_bbox', 'val/loss_bbox', 'train/loss_mask_ce', 'val/loss_mask_ce',
          'train/loss_mask_dice', 'val/loss_mask_dice', 'val/mAP_50_95', 'val/segm_mAP_50_95')


def read_training_history(path):
    if not path.is_file():
        return {'source': 'training/metrics.csv', 'epochs': [], 'available': False}
    epochs = {}
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            try:
                epoch = int(float(row['epoch'])) + 1
            except (KeyError, TypeError, ValueError):
                continue
            point = epochs.setdefault(epoch, {'epoch': epoch})
            for key in FIELDS:
                try:
                    value = float(row.get(key, ''))
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    point[key] = value
    return {'source': 'training/metrics.csv', 'epochs': [epochs[k] for k in sorted(epochs)],
            'available': any(len(row) > 1 for row in epochs.values())}
