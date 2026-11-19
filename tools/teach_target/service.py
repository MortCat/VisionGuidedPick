"""File-backed Teach Target application. Never imports ROS or sends motion commands."""
import copy,hashlib,io,json,os,shutil,subprocess,sys,threading,time,uuid
from datetime import datetime,timezone
from pathlib import Path
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from teaching_config import config_path as default_config_path, load_config
from PIL import Image,ImageOps,UnidentifiedImageError
from fastapi import FastAPI,Request,UploadFile,File,Form
from fastapi.responses import JSONResponse,FileResponse,Response
from fastapi.exceptions import RequestValidationError
from starlette.concurrency import run_in_threadpool

def stamp():return datetime.now(timezone.utc).isoformat()
def atomic(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False),encoding='utf-8');os.replace(tmp,path)
def load(path):return json.loads(path.read_text(encoding='utf-8'))
def identifier():return uuid.uuid4().hex
class ApiFailure(Exception):
    def __init__(self,status,code,message,**details):self.status=status;self.code=code;self.message=message;self.details=details
def fail(status,code,message,**details):raise ApiFailure(status,code,message,**details)
def create_app(config_path=None,root_override=None,observe_factory=None):
    project=Path(__file__).resolve().parents[2]
    cfg=load_config(project, config_path)
    root=Path(root_override or project/cfg['storage_root']).resolve();root.mkdir(parents=True,exist_ok=True)
    app=FastAPI(title='VisionGuidedPick Product API',version='0.4.0')
    # Swagger and direct clients read the same version-controlled technical contract.
    app.openapi=lambda:load(project/'docs/api/teach-target.openapi.json')
    owner=(root/'.owner.lock').open('a+b')
    try:
        if os.name=='nt':
            import msvcrt
            if owner.tell()==0:owner.write(b'0');owner.flush()
            owner.seek(0);msvcrt.locking(owner.fileno(),msvcrt.LK_NBLCK,1)
        else:
            import fcntl
            fcntl.flock(owner.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
    except OSError:
        owner.close();raise RuntimeError('Another Teach Target API owns this storage root')
    lock=threading.RLock();processes={}
    import importlib.util
    spec=importlib.util.spec_from_file_location('teach_sam_contour',Path(__file__).with_name('sam_contour.py'))
    sam_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(sam_module)
    contour_client=sam_module.ContourClient(cfg['annotation'])
    spec=importlib.util.spec_from_file_location('teach_training_metrics',Path(__file__).with_name('training_metrics.py'))
    metrics_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(metrics_module)
    spec=importlib.util.spec_from_file_location('product_observe_runtime',Path(__file__).with_name('observe_runtime.py'))
    observe_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(observe_module)
    observe_cfg=yaml.safe_load((project/'config/observe.yaml').read_text(encoding='utf-8'))
    observe=observe_module.ObserveRuntime(project,root,observe_cfg,cfg['training']['python'],observe_factory)
    @app.exception_handler(observe_module.ObserveFailure)
    async def observe_error(request,e):return JSONResponse({'error':{'code':e.code,'message':str(e),'details':{}}},status_code=e.status)
    spec=importlib.util.spec_from_file_location('teach_dataset_split',Path(__file__).with_name('dataset_split.py'))
    split_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(split_module)
    bundle_module=observe_module.model_bundle
    # An interrupted service never resumes old training implicitly.
    for f in root.glob('jobs/*/job.json'):
        row=load(f)
        if row['state'] not in ('succeeded','cancelled','failed','interrupted'):
            row.update(state='interrupted',message='Service restarted; start a fresh training job',updated_at=stamp());atomic(f,row)
    def safe(base,key):
        if not key or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in key):fail(404,'NOT_FOUND','Unknown resource')
        return root/base/key
    def profile(t):
        p=safe('targets',t)/'profile.json'
        if not p.exists():fail(404,'TARGET_NOT_FOUND','Target not found')
        return load(p)
    def image(t,i):
        p=profile(t)
        if not any(x['image_id']==i for x in p['images']):fail(404,'IMAGE_NOT_FOUND','Image not in target draft')
        return safe('targets',t)/'images'/i
    def job(j):
        p=safe('jobs',j)/'job.json'
        if not p.exists():fail(404,'JOB_NOT_FOUND','Job not found')
        return load(p)
    @app.exception_handler(json.JSONDecodeError)
    async def invalid_json(request,e):return JSONResponse({'error':{'code':'INVALID_REQUEST','message':'Request body must be valid JSON','details':{}}},400)
    @app.exception_handler(RequestValidationError)
    async def validation_error(request,e):return JSONResponse({'error':{'code':'INVALID_REQUEST','message':'Invalid request fields','details':{'errors':str(e)}}},422)
    @app.exception_handler(ApiFailure)
    async def error(request,e):return JSONResponse({'error':{'code':e.code,'message':e.message,'details':e.details}},status_code=e.status)
    @app.middleware('http')
    async def access(request,call_next):
        ip=request.client.host if request.client else ''
        if ip not in cfg['server']['allowed_ips']:return JSONResponse({'error':{'code':'ACCESS_DENIED','message':'Source IP is not authorized'}},403)
        method=request.method
        if method not in ('GET','HEAD'):
            origin=request.headers.get('origin')
            if origin and origin.rstrip('/')!=str(request.base_url).rstrip('/'):return JSONResponse({'error':{'code':'ACCESS_DENIED','message':'Cross-origin mutation denied'}},403)
        if method not in ('GET','HEAD') and request.headers.get('sec-fetch-site')=='cross-site':
            return JSONResponse({'error':{'code':'ACCESS_DENIED','message':'Cross-site mutation denied'}},403)
        return await call_next(request)
    def idem(request,payload,action):
        key=request.headers.get('Idempotency-Key','')
        if not key or len(key)>128:fail(422,'INVALID_REQUEST','Idempotency-Key required')
        # One installation, one operator: retries are scoped to method/path/key, not login cookies.
        p=root/'requests'/(hashlib.sha256(('single-user-v1:'+request.method+request.url.path+key).encode()).hexdigest()+'.json')
        digest=hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()
        with lock:
            if p.exists():
                cached=load(p)
                if cached['digest']!=digest:fail(409,'IDEMPOTENCY_CONFLICT','Key reused with different request')
                return JSONResponse(cached['body'],cached['status'])
            status,body=action();atomic(p,{'digest':digest,'status':status,'body':body});return JSONResponse(body,status)
    @app.get('/api/health')
    def health():return {'service':'teach_target','status':'ready','api_version':'0.4.0'}
    @app.get('/api/teach/targets')
    def targets():return [{k:v for k,v in load(p).items() if k!='images'}|{'image_count':len(load(p)['images'])} for p in sorted(root.glob('targets/*/profile.json'))]
    @app.post('/api/teach/targets')
    async def new_target(request:Request):
        data=await request.json();name=str(data.get('name','')).strip()
        if not name or len(name)>128:fail(422,'INVALID_NAME','Name must contain 1–128 characters')
        def action():
            row={'target_id':identifier(),'name':name,'draft_revision':0,'images':[],'latest_job_id':None};atomic(safe('targets',row['target_id'])/'profile.json',row);return 201,row
        return idem(request,data,action)
    @app.get('/api/teach/targets/{t}')
    def get_target(t:str):return profile(t)
    @app.post('/api/teach/targets/{t}/images')
    async def upload(t:str,request:Request,file:UploadFile=File(...),purpose:str=Form('teaching'),source_label:str=Form('')):
        if purpose not in ('teaching','validation'):fail(422,'INVALID_PURPOSE','Unknown image purpose')
        source_label=source_label.strip()
        if len(source_label)>256:fail(422,'INVALID_SOURCE','Source label is limited to256 characters')
        original_filename=(file.filename or '').replace('\\','/').rsplit('/',1)[-1][:255]
        data=await file.read(cfg['images']['max_upload_bytes']+1)
        if len(data)>cfg['images']['max_upload_bytes']:fail(413,'UPLOAD_TOO_LARGE','Image exceeds configured size')
        try:
            src=Image.open(io.BytesIO(data))
            if src.format not in ('JPEG','PNG'):fail(415,'UNSUPPORTED_IMAGE','JPEG/PNG required')
            full=ImageOps.exif_transpose(src).convert('RGB');full.load()
        except (UnidentifiedImageError,OSError,Image.DecompressionBombError):fail(400,'INVALID_IMAGE','Cannot decode image')
        digest=hashlib.sha256(data).hexdigest()
        def action():
            p=profile(t)
            for existing in p['images']:
                if existing['sha256']==digest:
                    if existing['purpose']!=purpose:fail(409,'DATA_SPLIT_CONFLICT','This image already belongs to the other dataset split')
                    return 201,existing
            i=identifier();folder=safe('targets',t)/'images'/i;folder.mkdir(parents=True)
            (folder/'raw').write_bytes(data);full.save(folder/'full.png');preview=full.copy();preview.thumbnail((cfg['images']['preview_max_edge_px'],)*2);preview.save(folder/'preview.png')
            base=f'/api/teach/targets/{t}/images/{i}/content?variant='
            row={'image_id':i,'target_id':t,'width':full.width,'height':full.height,'sha256':digest,'purpose':purpose,'source_label':source_label,'original_filename':original_filename,'raw_url':base+'raw','full_url':base+'full','preview_url':base+'preview','annotation_revision':0,'annotation_state':'pending','draft_revision':p['draft_revision']+1}
            atomic(folder/'metadata.json',row);atomic(folder/'annotation.json',{'image_id':i,'revision':0,'state':'pending','width':full.width,'height':full.height,'instances':[]})
            p['images'].append(row);p['draft_revision']+=1;atomic(safe('targets',t)/'profile.json',p);return 201,row
        return idem(request,{'sha256':digest,'purpose':purpose,'source_label':source_label,'original_filename':original_filename},action)
    @app.get('/api/teach/targets/{t}/dataset-summary')
    def dataset_summary(t:str):
        row=profile(t);return {'target_id':t,'draft_revision':row['draft_revision'],**split_module.summarize_dataset(row['images'],cfg['training'])}
    @app.get('/api/teach/targets/{t}/images/{i}/content')
    def content(t:str,i:str,variant:str='full'):
        folder=image(t,i)
        if variant not in ('raw','full','preview'):fail(422,'INVALID_VARIANT','Unknown image variant')
        return FileResponse(folder/('raw' if variant=='raw' else variant+'.png'),media_type=None if variant=='raw' else 'image/png')
    @app.delete('/api/teach/targets/{t}/images/{i}')
    def remove(t:str,i:str,expected_revision:int):
        with lock:
            p=profile(t)
            if p['draft_revision']!=expected_revision:fail(409,'REVISION_CONFLICT','Reload target')
            image(t,i);p['images']=[x for x in p['images'] if x['image_id']!=i];p['draft_revision']+=1;atomic(safe('targets',t)/'profile.json',p);return {'target_id':t,'draft_revision':p['draft_revision']}
    @app.get('/api/teach/targets/{t}/images/{i}/annotation')
    def annotation(t:str,i:str):return load(image(t,i)/'annotation.json')
    @app.put('/api/teach/targets/{t}/images/{i}/annotation')
    async def save_annotation(t:str,i:str,request:Request):
        data=await request.json()
        with lock:
            folder=image(t,i);old=load(folder/'annotation.json');p=profile(t)
            if data.get('expected_revision')!=old['revision']:fail(409,'REVISION_CONFLICT','Reload annotation')
            if (data.get('width'),data.get('height'))!=(old['width'],old['height']):fail(400,'INVALID_MASK','Mask dimensions do not match full image')
            instances=data.get('instances')
            if not isinstance(instances,list):fail(422,'INVALID_MASK','Instances array required')
            for instance in instances:
                mask=instance.get('mask',{});counts=mask.get('counts',[])
                if mask.get('size')!=[old['height'],old['width']] or not counts or not all(type(x) is int and x>=0 for x in counts) or sum(counts)!=old['width']*old['height']:fail(400,'INVALID_MASK','Invalid column-major RLE')
            row=old|{'revision':old['revision']+1,'state':'reviewed','instances':instances,'outcome':'instances' if instances else 'no_target'};atomic(folder/'annotation.json',row)
            p['draft_revision']+=1
            for x in p['images']:
                if x['image_id']==i:x.update(annotation_revision=row['revision'],annotation_state='reviewed',instance_count=len(instances),annotation_outcome=row['outcome'])
            atomic(safe('targets',t)/'profile.json',p);return {'annotation':row,'draft_revision':p['draft_revision']}
    @app.post('/api/teach/targets/{t}/images/{i}/mask-proposals')
    async def propose(t:str,i:str,request:Request):
        folder=image(t,i);prompt=await request.json()
        if not isinstance(prompt,dict):fail(422,'INVALID_REQUEST','Contour prompt must be an object')
        if any(proc.poll() is None for proc in list(processes.values())):
            fail(409,'TRAINING_BUSY','Finish or cancel training before annotation inference')
        if observe.busy():fail(409,'OBSERVE_BUSY','Stop Observe before annotation inference')
        try:result=await run_in_threadpool(contour_client.propose,folder/'full.png',prompt)
        except ValueError as e:fail(400,'INVALID_MASK_PROMPT',str(e))
        except sam_module.ContourUnavailable as e:fail(503,'ANNOTATION_BACKEND_UNAVAILABLE',str(e))
        result.pop('sam_mask',None)
        return {'image_id':i,**result}
    @app.get('/api/teach/training-jobs')
    def jobs(target_id:str|None=None):return [load(p) for p in sorted(root.glob('jobs/*/job.json')) if target_id is None or load(p)['target_id']==target_id]
    @app.get('/api/teach/training-jobs/{j}')
    def get_job(j:str):
        return {**job(j), 'training_history': metrics_module.read_training_history(
            safe('jobs',j)/'training'/'metrics.csv')}
    @app.post('/api/teach/targets/{t}/training-jobs')
    async def train(t:str,request:Request):
        data=await request.json()
        if set(data)!={'draft_revision'}:fail(422,'INVALID_REQUEST','Only draft_revision is accepted')
        def action():
            p=profile(t)
            if data['draft_revision']!=p['draft_revision']:fail(409,'REVISION_CONFLICT','Reload target')
            if any(x.poll() is None for x in processes.values()):fail(409,'TRAINING_BUSY','One training job is already running')
            if len(p['images'])<2 or any(x['annotation_state']!='reviewed' for x in p['images']):fail(409,'DATA_NOT_READY','Review masks for at least two images')
            planned_training,planned_validation,validation_mode=split_module.select_dataset_split(p['images'],cfg['training'])
            if not planned_training or not planned_validation:fail(409,'DATA_NOT_READY','Need both training and validation images')
            python=cfg['training']['python']
            if not Path(python).is_file():fail(503,'TRAINING_ENVIRONMENT_UNAVAILABLE','Configure training.python in config/teaching.yaml')
            j=identifier();folder=safe('jobs',j);folder.mkdir(parents=True);snapshot=copy.deepcopy(p)
            for x in snapshot['images']:x['annotation']=load(image(t,x['image_id'])/'annotation.json');x['full_path']=str(image(t,x['image_id'])/'full.png')
            atomic(folder/'dataset.json',snapshot);settings=copy.deepcopy(cfg);(folder/'resolved_config.yaml').write_text(yaml.safe_dump(settings),encoding='utf-8')
            row={'job_id':j,'target_id':t,'dataset_version':identifier(),'config_sha256':hashlib.sha256((folder/'resolved_config.yaml').read_bytes()).hexdigest(),'state':'preparing','epoch':0,'completed_epochs':0,'eta_sec':None,'total_epochs':cfg['training']['epochs'],'elapsed_sec':0,'updated_at':stamp(),'model_version':None,'message':'Preparing dataset','error':None};atomic(folder/'job.json',row)
            contour_client.close()  # Release annotation GPU memory before the training worker starts.
            f=(folder/'worker.log').open('w');proc=subprocess.Popen([python,str(Path(__file__).with_name('worker.py')),str(folder)],stdout=f,stderr=subprocess.STDOUT);f.close();processes[j]=proc
            p['latest_job_id']=j;atomic(safe('targets',t)/'profile.json',p)
            def monitor():
                started=time.monotonic()
                while proc.poll() is None:
                    time.sleep(.5)
                    with lock:
                        state=job(j);progress=folder/'progress.json'
                        if state['state']!='cancelling' and progress.exists():state.update(load(progress))
                        state.update(elapsed_sec=time.monotonic()-started,updated_at=stamp());atomic(folder/'job.json',state)
                with lock:
                    state=job(j)
                    if state['state']=='cancelling':state.update(state='cancelled',message='Cancelled; next Train starts fresh')
                    elif proc.returncode==0 and (folder/'result.json').exists():
                        # The model directory carries its own class map, preprocessing and evaluation.
                        try:
                            result=load(folder/'result.json');bundle=bundle_module.bundle_fields(folder,yaml.safe_load((folder/'resolved_config.yaml').read_text(encoding='utf-8')),[p['name']])
                        except (bundle_module.BundleError,OSError,KeyError,TypeError,ValueError) as e:
                            state.update(state='failed',eta_sec=None,message='Model bundle failed: '+str(e)[:400],error={'code':getattr(e,'code','MODEL_BUNDLE_FAILED'),'message':str(e)[:800]})
                        else:
                            version=identifier();dest=safe('models',version);dest.mkdir(parents=True);shutil.move(str(folder/'best.pth'),dest/'checkpoint.pth')
                            manifest=bundle_module.seal({'model_version':version,'target_id':t,'job_id':j,'dataset_version':state['dataset_version'],'config_sha256':state['config_sha256'],'checkpoint_sha256':bundle_module.file_sha256(dest/'checkpoint.pth'),'class_names':[p['name']],'state':'candidate','created_at':stamp(),'evaluation_url':f'/api/teach/training-jobs/{j}/results',**bundle});atomic(dest/'manifest.json',manifest);result.update(job_id=j,model_version=version);atomic(folder/'result.json',result);state.update(state='succeeded',model_version=version,message='Training completed; review validation results')
                    else:
                        lines=(folder/'worker.log').read_text(errors='replace').splitlines()
                        reason=next((line.strip() for line in reversed(lines) if line.strip()),'Worker exited without a result')
                        state.update(state='failed',eta_sec=None,message='Training failed: '+reason[:400],error={'code':'TRAINING_FAILED','message':reason[:800]})
                    for weight in folder.rglob('*'):
                        if weight.is_file() and weight.suffix in ('.pth','.pt','.ckpt') and weight.resolve().is_relative_to(folder.resolve()):weight.unlink()
                    state['updated_at']=stamp();atomic(folder/'job.json',state)
            threading.Thread(target=monitor,daemon=True).start();return 202,row
        return idem(request,data,action)
    @app.post('/api/teach/training-jobs/{j}/cancel')
    def cancel(j:str,request:Request):
        def action():
            row=job(j)
            if row['state'] in ('succeeded','failed','cancelled','interrupted'):return 200,row
            row.update(state='cancelling',message='Waiting for worker exit');atomic(safe('jobs',j)/'job.json',row)
            proc=processes.get(j)
            if proc and proc.poll() is None:proc.terminate()
            return 202,row
        return idem(request,{},action)
    @app.get('/api/teach/training-jobs/{j}/results')
    def results(j:str):
        row=job(j)
        if row['state']!='succeeded':fail(409,'JOB_NOT_READY','No successful result')
        return {**load(safe('jobs',j)/'result.json'),'training_history':metrics_module.read_training_history(safe('jobs',j)/'training'/'metrics.csv')}
    @app.get('/api/teach/training-jobs/{j}/artifacts/{artifact_id}')
    def artifact(j:str,artifact_id:str):
        row=job(j)
        if row['state']!='succeeded':fail(409,'JOB_NOT_READY','No completed evaluation')
        safe('jobs',artifact_id)
        result=load(safe('jobs',j)/'result.json');url=f'/api/teach/training-jobs/{j}/artifacts/{artifact_id}'
        if url not in result['overlay_urls']:fail(404,'NOT_FOUND','Artifact not in result manifest')
        return FileResponse(safe('jobs',j)/'evaluation'/(artifact_id+'.png'),media_type='image/png')
    @app.get('/api/teach/models/{v}')
    def model(v:str):
        p=safe('models',v)/'manifest.json'
        if not p.exists():fail(404,'MODEL_NOT_FOUND','Model not found')
        return load(p)
    @app.post('/api/teach/models/{v}/publish')
    def publish(v:str,request:Request):
        def action():
            row=model(v)
            try:bundle_module.verify(safe('models',v),row)
            except bundle_module.BundleError as e:fail(e.status,e.code,str(e))
            row['state']='published';atomic(safe('models',v)/'manifest.json',row);return 200,row
        return idem(request,{},action)
    @app.get('/api/teach/models')
    def models(target_id:str|None=None,state:str='published'):
        if state!='published':fail(422,'INVALID_REQUEST','Only published catalogue is supported')
        rows=observe_module.list_published_models(root,target_id)
        default=json.loads((project/'config/default_model.json').read_text())['model_version']
        return sorted(rows,key=lambda row:row['model_version']!=default)
    def observe_body(data,keys):
        if not isinstance(data,dict) or set(data)!=set(keys) or any(not isinstance(data[k],str) or not data[k] for k in keys):
            fail(422,'INVALID_REQUEST','Expected fields: '+', '.join(keys))
    @app.get('/api/observe/model-activations/{activation_id}')
    def activation_status(activation_id:str):
        with observe.lock:
            row=observe.activations.get(activation_id)
            if row is None:fail(404,'ACTIVATION_NOT_FOUND','Unknown model activation')
            return dict(row)
    @app.post('/api/observe/model-activations')
    async def activate(request:Request):
        data=await request.json();observe_body(data,['model_version'])
        def action():
            contour_client.close()
            return 202,observe.request_activation(data['model_version'])
        return await run_in_threadpool(idem,request,data,action)
    @app.post('/api/observe/runs')
    async def start_observe(request:Request):
        data=await request.json();observe_body(data,['activation_id'])
        return await run_in_threadpool(idem,request,data,lambda:(202,observe.start_run(data['activation_id'])))
    @app.post('/api/observe/runs/{run_id}/stop')
    async def stop_observe(run_id:str,request:Request):
        data=await request.json();observe_body(data,[])
        return await run_in_threadpool(idem,request,data,lambda:observe.stop_run(run_id))
    @app.get('/api/auto-pick/status')
    def auto_pick_status(arm: str = 'arm_01', point_cloud: bool = False, cloud_revision: str | None = None):
        if arm not in ('arm_01', 'arm_02'):fail(422, 'PICK_ARM_UNKNOWN', 'Unknown arm')
        return observe.auto_pick(arm, point_cloud=point_cloud, cloud_revision=cloud_revision)
    @app.get('/api/auto-pick/wrist-preview')
    def auto_pick_wrist_preview(arm: str = 'arm_01', frame: int | None = None):
        if arm not in ('arm_01', 'arm_02'):fail(422, 'PICK_ARM_UNKNOWN', 'Unknown arm')
        return Response(observe.wrist_preview(arm, frame), media_type='image/jpeg',
                        headers={'Cache-Control': 'no-store'})
    @app.post('/api/auto-pick/{action}')
    async def auto_pick_command(action:str,request:Request,arm:str='arm_01'):
        if action not in ('start','continuous','pause','cancel','home','clear-bin'):fail(404,'PICK_ACTION_UNKNOWN','Unknown Auto Pick action')
        if arm not in ('arm_01', 'arm_02'):fail(422, 'PICK_ARM_UNKNOWN', 'Unknown arm')
        data=await request.json();observe_body(data,['run_id'])
        return await run_in_threadpool(idem,request,dict(data,arm=arm),lambda:(202,observe.command_pick(data['run_id'],'pick_'+action.replace('-','_'),arm)))
    @app.get('/api/observe/status')
    def observe_status():return observe.status()
    @app.get('/api/observe/runs/{run_id}/objects')
    def observe_objects(run_id: str):return observe.history(run_id)
    @app.get('/api/observe/runs/{run_id}/objects/{track_id}/point-cloud')
    def observe_object_point_cloud(run_id: str, track_id: str, cloud_revision: str | None = None):
        return observe.object_point_cloud(run_id, track_id, cloud_revision)

    @app.get('/api/observe/runs/{run_id}/pick-preview')
    def observe_pick_preview(run_id: str):
        snapshot = observe_snapshot(run_id)
        status = observe.status()
        run = status.get('run') or {}
        age = status.get('display_age_sec')
        same_frame = (run.get('run_id') == run_id
                      and status.get('last_frame_key') == snapshot.get('frame_key'))
        candidates = []
        if same_frame and status.get('source_state') == 'live' and age is not None:
            candidates = [candidate for candidate in snapshot.get('pick_candidates', [])
                          if 0 <= age < candidate['valid_for_sec']]
        return {'run_id': run_id, 'frame_key': snapshot.get('frame_key'),
                'motion_control': False, 'candidates': candidates}

    @app.get('/api/observe/runs/{run_id}/snapshot')
    def observe_snapshot(run_id:str):
        with observe.lock:
            if run_id not in observe.runs:fail(404,'RUN_NOT_FOUND','Unknown Observe run')
            if not observe.run or run_id!=observe.run['run_id']:fail(410,'RUN_EXPIRED','Run snapshot is no longer cached')
            return copy.deepcopy(observe.snapshot) if observe.snapshot else {'run_id':run_id,'frame_key':None,'message':observe.run['message']}
    @app.get('/api/observe/frames/{frame_key}/preview')
    def observe_preview(frame_key:str,layer:str='rgb'):
        if layer not in ('rgb','mask'):fail(422,'INVALID_REQUEST','Layer must be rgb or mask')
        with observe.lock:
            blobs=observe.frames.get(frame_key)
            if blobs is None:fail(410,'FRAME_EXPIRED','Get the latest snapshot')
            return Response(blobs[layer],media_type='image/jpeg' if layer=='rgb' else 'image/png',headers={'Cache-Control':'no-store'})
    for url,file in [('/','index.html'),('/app.js','app.js'),('/style.css','style.css'),('/navigation.js','navigation.js'),('/observe','observe.html'),('/observe.js','observe.js'),('/gantry-cloud.js','gantry_cloud.js'),('/auto-pick','auto_pick.html'),('/auto-pick.js','auto_pick.js')]:
        def build_handler(asset):
            # The App keeps a persistent browser profile; revalidate so an updated checkout is used.
            def serve():return FileResponse(Path(__file__).with_name('web')/asset,headers={'Cache-Control':'no-cache'})
            return serve
        app.add_api_route(url,build_handler(file),methods=['GET'],include_in_schema=False)
    @app.on_event('shutdown')
    def shutdown():
        observe.close()
        contour_client.close()
        for proc in processes.values():
            if proc.poll() is None:proc.terminate()
        for proc in processes.values():
            if proc.poll() is None:
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
        owner.close()
    app.state.observe=observe
    app.state.root=root
    return app

if __name__=='__main__':
    import argparse,uvicorn
    parser=argparse.ArgumentParser();parser.add_argument('--config');args=parser.parse_args()
    project=Path(__file__).resolve().parents[2]
    p=Path(args.config or default_config_path(project));config=load_config(project, p)
    # Preserve the configured LAN interface and the already-authorized local loopback.
    # Do not bind other interfaces or add LAN clients to the source allowlist.
    import socket
    host=config['server']['host'];port=config['server']['port'];sockets=[]
    try:
        for address in dict.fromkeys([host,'127.0.0.1']):
            listener=socket.socket(socket.AF_INET6 if ':' in address else socket.AF_INET,socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            listener.bind((address,port));listener.listen(128);sockets.append(listener)
        uvicorn.Server(uvicorn.Config(create_app(p),host=host,port=port)).run(sockets=sockets)
    finally:
        for listener in sockets:listener.close()
