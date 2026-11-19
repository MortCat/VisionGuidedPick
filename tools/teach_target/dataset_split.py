"""Deterministic image membership shared by the UI summary and training worker."""
import random


def select_dataset_split(images, config):
    teaching = [x for x in images if x['purpose'] == 'teaching']
    validation = [x for x in images if x['purpose'] == 'validation']
    random.Random(config['seed']).shuffle(teaching)
    mode = 'specified_validation' if validation else 'automatic_holdout'
    if not validation:
        count = max(1, round(len(teaching) * config['validation_fraction']))
        validation, teaching = teaching[:count], teaching[count:]
    return teaching, validation, mode


def image_source(row):
    return dict(image_id=row['image_id'], original_filename=row.get('original_filename'),
                source_label=row.get('source_label') or None, purpose=row['purpose'])


def summarize_dataset(images, config):
    training, validation, mode = select_dataset_split(images, config)
    return dict(train_count=len(training), validation_count=len(validation), validation_mode=mode,
                pending_count=sum(x['annotation_state'] != 'reviewed' for x in images),
                training_images=[image_source(x) for x in training],
                validation_images=[image_source(x) for x in validation])
