"""Fresh RF-DETR fine-tuning subprocess; consumes frozen data/config, never ROS."""
import json,sys,random,shutil,os,threading,time
from pathlib import Path
import yaml
from PIL import Image,ImageDraw
from dataset_split import select_dataset_split,image_source

def build_validation_metrics(ious,positive_ious,negative_count,negative_false_positives):
    return {'mean_union_iou':sum(ious)/len(ious) if ious else None,
            'positive_mean_union_iou':sum(positive_ious)/len(positive_ious) if positive_ious else None,
            'positive_image_count':len(positive_ious),
            'positive_image_hit_rate_at_iou_0_5':sum(value>=.5 for value in positive_ious)/len(positive_ious) if positive_ious else None,
            'negative_image_count':negative_count,'negative_false_positive_images':negative_false_positives,
            'negative_false_positive_rate':negative_false_positives/negative_count if negative_count else None}

def main():
    parent=os.getppid()
    def watch_parent():
        while True:
            time.sleep(1)
            if os.getppid()!=parent:os._exit(2)
    threading.Thread(target=watch_parent,daemon=True).start()
    folder=Path(sys.argv[1]);cfg=yaml.safe_load((folder/'resolved_config.yaml').read_text());c=cfg['training'];data=json.loads((folder/'dataset.json').read_text())
    import numpy as np
    from pycocotools import mask as mu
    import torch
    from pytorch_lightning import Callback,seed_everything
    from rfdetr import RFDETRSegSmall,RFDETR
    from rfdetr.config import SegmentationTrainConfig
    from rfdetr.training import RFDETRDataModule,RFDETRModelModule,build_trainer
    if not torch.cuda.is_available():raise RuntimeError('CUDA training environment unavailable')
    dataset_counts = {}
    def progress(**kw):
        tmp=folder/'progress.tmp';tmp.write_text(json.dumps(dataset_counts | kw));os.replace(tmp,folder/'progress.json')
    images=data['images'];teaching,valid,validation_mode=select_dataset_split(images,c)
    if not teaching or not valid:raise ValueError('Need nonempty train and held-out validation sets')
    if not any(x['annotation']['instances'] for x in teaching):raise ValueError('Training set contains no target masks')
    from augmentation import augment_training
    progress(state='preparing', epoch=0, message='Preparing training images and augmentation')
    augmented = augment_training(teaching, c, folder / 'augmentation')
    (folder / 'augmentation.json').write_text(json.dumps([
        {key: row[key] for key in ('image_id', 'source_image_id', 'background_image_id')}
        for row in augmented
    ]))
    dataset_counts.update(train_count=len(teaching), augmented_train_count=len(augmented))
    progress(state='preparing', epoch=0, message='Training images ready')
    for split,rows in [('train',teaching + augmented),('valid',valid)]:
        directory=folder/'dataset'/split;directory.mkdir(parents=True);co={'images':[],'annotations':[],'categories':[{'id':1,'name':data['name']}],'info':{}}
        for image_id,x in enumerate(rows,1):
            name=x['image_id']+'.png';shutil.copy2(x['full_path'],directory/name);co['images'].append({'id':image_id,'file_name':name,'width':x['width'],'height':x['height']})
            for instance in x['annotation']['instances']:
                rle=mu.frPyObjects(instance['mask'],x['height'],x['width']);area=int(mu.area(rle));bbox=mu.toBbox(rle).tolist();rle['counts']=rle['counts'].decode()
                co['annotations'].append({'id':len(co['annotations'])+1,'image_id':image_id,'category_id':1,'segmentation':rle,'area':area,'bbox':bbox,'iscrowd':0})
        (directory/'_annotations.coco.json').write_text(json.dumps(co))
    seed_everything(c['seed'],workers=True)
    model=RFDETRSegSmall(num_classes=1,resolution=c['resolution']);model.model_config.model_name='RFDETRSegSmall'
    config=SegmentationTrainConfig(dataset_dir=str(folder/'dataset'),output_dir=str(folder/'training'),epochs=c['epochs'],batch_size=c['batch_size'],grad_accum_steps=c['grad_accum_steps'],lr=c['lr'],lr_encoder=c['lr_encoder'],num_workers=c['num_workers'],seed=c['seed'],class_names=[data['name']],multi_scale=False,expanded_scales=False,use_ema=False,run_test=False,compute_val_loss=True,compute_train_metrics=False,early_stopping=False,tensorboard=False,wandb=False,amp_dtype='bf16',checkpoint_interval=c['epochs'])
    class Metrics(Callback):
        def __init__(self):self.durations=[];self.epoch_started=None
        def on_train_epoch_start(self,trainer,module):
            self.epoch_started=time.monotonic()
            progress(state='training',epoch=trainer.current_epoch+1,completed_epochs=trainer.current_epoch,
                     eta_sec=sum(self.durations[-3:])/len(self.durations[-3:])*(c['epochs']-trainer.current_epoch) if self.durations else None,
                     eta_updated_at=time.time(),message=f'Training epoch {trainer.current_epoch+1} of {c["epochs"]}')
        def on_train_epoch_end(self,trainer,module):
            self.durations.append(time.monotonic()-self.epoch_started)
            completed=trainer.current_epoch+1
            progress(state='training',epoch=completed,completed_epochs=completed,
                     eta_sec=sum(self.durations[-3:])/len(self.durations[-3:])*(c['epochs']-completed),
                     eta_updated_at=time.time(),message=f'Completed {completed} of {c["epochs"]} epochs')
    trainer=build_trainer(config,model.model_config,accelerator='gpu',devices=1);trainer.callbacks.append(Metrics());trainer.fit(RFDETRModelModule(model.model_config,config),RFDETRDataModule(model.model_config,config))
    progress(state='evaluating',epoch=c['epochs'],completed_epochs=c['epochs'],eta_sec=None,message='Evaluating held-out images')
    # BestModelCallback exports the inference payload (model/args/model_config).
    # Native Lightning .ckpt files are resume snapshots, not RFDETR.from_checkpoint input.
    checkpoint=folder/'training'/'checkpoint_best_total.pth'
    if not checkpoint.is_file():raise RuntimeError('RF-DETR inference checkpoint was not exported; no model publication')
    import pickle
    try:
        checkpoint_metadata = torch.load(checkpoint, map_location='cpu', weights_only=True)
    except pickle.UnpicklingError:
        checkpoint_metadata = {}
    epoch_index = checkpoint_metadata.get('epoch')
    selected_epoch = epoch_index + 1 if type(epoch_index) is int and epoch_index >= 0 else None
    del checkpoint_metadata
    shutil.copy2(checkpoint,folder/'best.pth');predictor=RFDETR.from_checkpoint(str(folder/'best.pth'),resolution=c['resolution']);predictor.model.model.eval()
    ious=[];positive_ious=[];negative_false_positives=0;negative_count=0;per_image=[];overlays=[];evaluation=folder/"evaluation";evaluation.mkdir();job=json.loads((folder/"job.json").read_text())["job_id"]
    for x in valid:
        truth=np.zeros((x['height'],x['width']),bool)
        for instance in x['annotation']['instances']:truth|=mu.decode(mu.frPyObjects(instance['mask'],x['height'],x['width'])).astype(bool)
        detection=predictor.predict(Image.open(x['full_path']).convert('RGB'),threshold=.5);prediction=np.zeros_like(truth)
        if len(detection) and detection.mask is None:raise RuntimeError('Model returned no instance masks')
        target_indices=[index for index,category in enumerate(detection.class_id) if int(category)==0]
        for index in target_indices:prediction|=detection.mask[index].astype(bool)
        rgb=np.array(Image.open(x['full_path']).convert('RGB'));rgb[prediction]=(rgb[prediction]*.55+np.array([20,240,150])*.45).astype(np.uint8);output=Image.fromarray(rgb);ImageDraw.Draw(output).text((10,10),'Prediction overlay',fill='white',stroke_width=2,stroke_fill='black');output.save(evaluation/(x['image_id']+'.png'));overlays.append(f'/api/teach/training-jobs/{job}/artifacts/{x["image_id"]}')
        union=(truth|prediction).sum();iou=float((truth&prediction).sum()/union) if union else 1.;ious.append(iou)
        if truth.any():positive_ious.append(iou)
        else:
            negative_count+=1;negative_false_positives+=int(prediction.any())
        per_image.append({**image_source(x),'ground_truth_instances':len(x['annotation']['instances']),'predicted_instances':len(target_indices),'union_iou':iou,'max_target_confidence':max((float(detection.confidence[index]) for index in target_indices),default=None),'overlay_url':overlays[-1]})
    from importlib.metadata import version as installed_version
    library_versions={name:installed_version(name) for name in ('rfdetr','torch','torchvision')}
    (folder/'result.json').write_text(json.dumps({'library_versions':library_versions,'metrics_source':validation_mode,'selected_epoch':selected_epoch,'evaluation_threshold':0.5,'train_count':len(teaching),'augmented_train_count':len(augmented),'validation_count':len(valid),'metrics':build_validation_metrics(ious,positive_ious,negative_count,negative_false_positives),'validation_images':per_image,'training_images':[image_source(x) for x in teaching],'overlay_urls':overlays,'deployment_validated':False}))
if __name__=='__main__':main()
