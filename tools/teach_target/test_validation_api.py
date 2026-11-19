"""Explicit validation membership, negative labels, provenance and snapshot regression."""
from pathlib import Path
import importlib.util,io,json,sys,tempfile,time
from fastapi.testclient import TestClient
from PIL import Image
import yaml

r=Path(__file__).resolve().parents[2]
def module(name):
 spec=importlib.util.spec_from_file_location(name,r/'tools/teach_target'/f'{name}.py')
 value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value
service=module('service');split=module('dataset_split')
sys.path.insert(0,str(r/'tools/teach_target'))
worker=module('worker')
metrics=worker.build_validation_metrics([0,1,1],[0],2,0)
assert metrics['positive_mean_union_iou']==0 and metrics['mean_union_iou']==2/3 and metrics['negative_false_positive_rate']==0
assert worker.build_validation_metrics([1],[],1,0)['positive_mean_union_iou'] is None
assert worker.build_validation_metrics([.8],[.8],0,0)['negative_false_positive_rate'] is None
with tempfile.TemporaryDirectory() as name:
 folder=Path(name);cfg=yaml.safe_load((r/'config/teaching.yaml').read_text());cfg['server']['allowed_ips']=['testclient'];cfg['training']['python']=sys.executable
 config=folder/'config.yaml';config.write_text(yaml.safe_dump(cfg));store=folder/'store'
 with TestClient(service.create_app(config,store)) as client:
  def post(path,body,key):return client.post(path,json=body,headers={'Idempotency-Key':key})
  target=post('/api/teach/targets',{'name':'Target'},'create').json()['target_id'];base='/api/teach/targets/'+target;raws=[]
  for index in range(18):
   buf=io.BytesIO();Image.new('RGB',(32,24),(index*11,index*7,index*3)).save(buf,format='PNG');raws.append(buf.getvalue())
   purpose='teaching' if index<12 else 'validation';source='Orbit / test session' if index<12 else 'Gantry / test session'
   response=client.post(base+'/images',files={'file':(f'frame_{index:02}.png',raws[-1],'image/png')},data={'purpose':purpose,'source_label':source},headers={'Idempotency-Key':f'upload-{index}'})
   assert response.status_code==201,response.text
   image=response.json();assert image['source_label']==source and image['original_filename']==f'frame_{index:02}.png'
   instances=[] if index>=16 else [{'instance_id':'one','mask':{'size':[24,32],'counts':[0,384,384]}}]
   if index==12:instances.append({'instance_id':'two','mask':{'size':[24,32],'counts':[384,384]}})
   saved=client.put(base+f"/images/{image['image_id']}/annotation",json={'expected_revision':0,'width':32,'height':24,'instances':instances})
   assert saved.status_code==200,saved.text
   assert saved.json()['annotation']['outcome']==('instances' if instances else 'no_target')
  summary=client.get(base+'/dataset-summary').json()
  assert (summary['train_count'],summary['validation_count'],summary['pending_count'])==(12,6,0),summary
  assert summary['validation_mode']=='specified_validation'
  assert all(x['source_label']=='Gantry / test session' for x in summary['validation_images'])
  assert not {x['image_id'] for x in summary['training_images']} & {x['image_id'] for x in summary['validation_images']}
  before_duplicate=client.get(base).json()
  duplicate=client.post(base+'/images',files={'file':('renamed-same-content.png',raws[0],'image/png')},data={'purpose':'teaching'},headers={'Idempotency-Key':'same-content-new-key'})
  assert duplicate.status_code==201
  assert duplicate.json()['image_id']==before_duplicate['images'][0]['image_id']
  after_duplicate=client.get(base).json()
  assert len(after_duplicate['images'])==len(before_duplicate['images']) and after_duplicate['draft_revision']==before_duplicate['draft_revision']
  response=client.post(base+'/images',files={'file':('duplicate.png',raws[0],'image/png')},data={'purpose':'validation'},headers={'Idempotency-Key':'leak'})
  assert response.status_code==409 and response.json()['error']['code']=='DATA_SPLIT_CONFLICT',response.text
  assert len(client.get(base).json()['images'])==18
  response=client.post(base+'/images',files={'file':('invalid.png',raws[0],'image/png')},data={'source_label':'x'*257},headers={'Idempotency-Key':'long-source'})
  assert response.status_code==422
  rows=client.get(base).json()['images'];first=next(x for x in rows if x['purpose']=='validation')
  assert first['instance_count']==2
  assert sum(x.get('annotation_outcome')=='no_target' for x in rows)==2
  # The summary and the actual worker use the same deterministic split function.
  train,valid,mode=split.select_dataset_split(rows,cfg['training'])
  assert [x['image_id'] for x in train]==[x['image_id'] for x in summary['training_images']]
  assert [x['image_id'] for x in valid]==[x['image_id'] for x in summary['validation_images']]
  original=[x for x in rows if x['purpose']=='teaching'];a,b,mode=split.select_dataset_split(original,cfg['training']);assert (len(a),len(b),mode)==(9,3,'automatic_holdout')
  job=post(base+'/training-jobs',{'draft_revision':summary['draft_revision']},'job');assert job.status_code==202,job.text
  jid=job.json()['job_id'];frozen=json.loads((store/'jobs'/jid/'dataset.json').read_text())
  assert sum(x['purpose']=='validation' for x in frozen['images'])==6
  assert all(x['source_label'] and x['original_filename'] for x in frozen['images'])
  assert sum(not x['annotation']['instances'] for x in frozen['images'])==2
  post('/api/teach/training-jobs/'+jid+'/cancel',{},'cancel')
  for _ in range(100):
   state=client.get('/api/teach/training-jobs/'+jid).json()
   if state['state'] in ('cancelled','failed'):break
   time.sleep(.05)
  assert state['model_version'] is None
  frozen_bytes=(store/'jobs'/jid/'dataset.json').read_bytes()
  for selected in [rows[0],first]:
   revision=client.get(base).json()['draft_revision'];url=base+f"/images/{selected['image_id']}"
   stale=client.delete(url,params={'expected_revision':revision-1})
   assert stale.status_code==409 and stale.json()['error']['code']=='REVISION_CONFLICT'
   removed=client.delete(url,params={'expected_revision':revision});assert removed.status_code==200,removed.text
   assert removed.json()['draft_revision']==revision+1
   assert all(x['image_id']!=selected['image_id'] for x in client.get(base).json()['images'])
   assert client.get(url+'/content').status_code==404
   assert (store/'targets'/target/'images'/selected['image_id']/'full.png').exists()
   assert (store/'jobs'/jid/'dataset.json').read_bytes()==frozen_bytes
  summary_after=client.get(base+'/dataset-summary').json()
  assert (summary_after['train_count'],summary_after['validation_count'])==(11,5)
  contract=client.get('/openapi.json').json();assert any('dataset-summary' in path for path in contract['paths'])
print('Validation:12/6 explicit vs9/3 fallback, source filenames, multi-instance/No Target, cross-split duplicate rejection, frozen dataset, revision-checked draft removal for both splits and API contract: passed; no GPU training')
