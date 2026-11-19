from pathlib import Path
import sys,importlib.util,os,tempfile,io,json,time
from unittest.mock import patch
from fastapi.testclient import TestClient
from PIL import Image
r=Path(__file__).resolve().parents[2];p=r/'tools/teach_target/service.py'
spec=importlib.util.spec_from_file_location('teach',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
import yaml
with tempfile.TemporaryDirectory() as folder:
    cfg=yaml.safe_load((r/'config/teaching.yaml').read_text());cfg['server']['allowed_ips']=['testclient'];cfg['training']['python']=sys.executable;cp=Path(folder)/'config.yaml';cp.write_text(yaml.safe_dump(cfg))
    with TestClient(m.create_app(cp,Path(folder)/'store')) as client:
        response=client.get('/api/teach/targets');assert response.status_code==200,response.text
        assert not client.cookies
        assert client.get('/api/session').status_code==404
        assert 'Access code' not in client.get('/').text
        # Persistent App profiles must revalidate UI files after a checkout update.
        for page in ('/','/app.js','/style.css','/navigation.js','/observe','/observe.js','/gantry-cloud.js','/auto-pick','/auto-pick.js'):
            served=client.get(page);assert served.status_code==200 and served.headers.get('cache-control')=='no-cache' and served.headers.get('etag'),(page,served.headers)
        static_revalidation=client.get('/observe.js',headers={'If-None-Match':served.headers['etag']}).status_code
        headers={'Idempotency-Key':'create'}
        first=client.post('/api/teach/targets',json={'name':'Rabbit'},headers=headers);assert first.status_code==201,first.text
        t=first.json()['target_id'];assert client.post('/api/teach/targets',json={'name':'Rabbit'},headers=headers).json()['target_id']==t
        assert client.post('/api/teach/targets',json={'name':'Bear'},headers=headers).status_code==409
        buf=io.BytesIO();Image.new('RGB',(1280,720),'tan').save(buf,format='PNG');raw=buf.getvalue();headers['Idempotency-Key']='image'
        response=client.post(f'/api/teach/targets/{t}/images',files={'file':('sample.png',raw,'image/png')},headers=headers);assert response.status_code==201,response.text
        asset=response.json();i=asset['image_id'];assert client.get(asset['raw_url']).content==raw
        im=Image.open(io.BytesIO(client.get(asset['full_url']).content));assert im.size==(1280,720)
        im=Image.open(io.BytesIO(client.get(asset['preview_url']).content));assert im.size==(1024,576)
        body={'expected_revision':0,'width':1280,'height':720,'instances':[{'instance_id':'rabbit1','mask':{'size':[720,1280],'counts':[0,1280*720]}}]}
        uri=f'/api/teach/targets/{t}/images/{i}/annotation'
        assert client.put(uri,json=body,headers=headers).status_code==200
        assert client.put(uri,json=body,headers=headers).status_code==409
        assert client.post('/api/observe/model-activations',json={'model_version':'x'},headers=headers).status_code==404
        assert client.post('/api/teach/targets',json={'name':'Missing key'}).status_code==422
        assert client.post('/api/teach/targets',json={'name':'Denied'},headers={'Origin':'https://unrelated.example','Idempotency-Key':'cross-origin'}).status_code==403
        assert client.post('/api/teach/targets',json={'name':'Denied'},headers={'Sec-Fetch-Site':'cross-site','Idempotency-Key':'cross-site'}).status_code==403
        assert client.post('/api/teach/targets',json={'name':'Rabbit'},headers={'Origin':'http://testserver','Idempotency-Key':'create'}).json()['target_id']==t
        assert client.post('/api/teach/targets',json={'name':'Rabbit'},headers={'Cookie':'ptc_session=obsolete','Idempotency-Key':'create'}).json()['target_id']==t
        assert client.get('/app.js?file=/etc/passwd').content==(r/'tools/teach_target/web/app.js').read_bytes()
        second=io.BytesIO();Image.new('RGB',(32,32),'blue').save(second,format='PNG');headers['Idempotency-Key']='second'
        stored=client.post(f'/api/teach/targets/{t}/images',files={'file':('second.png',second.getvalue(),'image/png')},data={'purpose':'validation'},headers=headers).json()
        client.put(f'/api/teach/targets/{t}/images/{stored["image_id"]}/annotation',json={'expected_revision':0,'width':32,'height':32,'instances':[]},headers=headers).raise_for_status()
        draft=client.get('/api/teach/targets/'+t).json()['draft_revision'];headers['Idempotency-Key']='training'
        # An occupied Observe runtime must not block a new training worker.
        with patch.object(client.app.state.observe, 'busy', return_value=True) as observe_busy:
            response=client.post(f'/api/teach/targets/{t}/training-jobs',json={'draft_revision':draft},headers=headers)
            assert response.status_code==202,response.text
            observe_busy.assert_not_called()
        jid=response.json()['job_id'];assert client.post(f'/api/teach/targets/{t}/training-jobs',json={'draft_revision':draft},headers=headers).json()['job_id']==jid
        headers['Idempotency-Key']='cancel';assert client.post(f'/api/teach/training-jobs/{jid}/cancel',headers=headers).status_code in (200,202)
        for _ in range(100):
            row=client.get('/api/teach/training-jobs/'+jid).json()
            if row['state'] in ('cancelled','failed'):break
            time.sleep(.1)
        assert row['state'] in ('cancelled','failed'),row
        assert row['model_version'] is None
        assert client.get('/').status_code==200
        assert client.get('/api/teach/targets/'+t).json()['images'][0]['annotation_state']=='reviewed'
    cfg['server']['allowed_ips']=[];cp.write_text(yaml.safe_dump(cfg))
    with TestClient(m.create_app(cp,Path(folder)/'denied-store')) as denied:
        assert denied.get('/api/teach/targets').status_code==403
        assert denied.post('/api/teach/targets',json={'name':'Denied'},headers={'Idempotency-Key':'ip-test'}).status_code==403
print(json.dumps({'no_login_upload':'passed','source_ip_and_cross_origin':'passed','idempotency_without_session':'passed','native_resolution':'1280x720 preserved; preview1024x576','annotation_revision':'passed','observe_no_false_ack':'passed','job_create_replay_cancel':'passed; no model published','static_ui_cache':f'no-cache with ETag; conditional GET {static_revalidation}'}))
