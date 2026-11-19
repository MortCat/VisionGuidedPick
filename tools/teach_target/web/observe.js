'use strict';
const el=id=>document.getElementById(id);
let models=[], current=null, bitmap=null, mask=null, renderedKey=null, pending=false, controlBusy=false, pollBusy=false;
let activeRun=null, previewBusy=false, previewDelayMs=100, previewStartedAt=-Infinity;
const displayedAt=[];
function displayFps(){const now=performance.now();while(displayedAt.length&&displayedAt[0]<now-2000)displayedAt.shift();return displayedAt.length>1?(displayedAt.length-1)/(displayedAt.at(-1)-displayedAt[0])*1000:0;}
let requestedVersion=new URLSearchParams(location.search).get('model_version');
function requestId(){return globalThis.crypto?.randomUUID?.()||Array.from(crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join('');}
async function request(url,method='GET',body){const options={method,cache:'no-store'};if(method==='POST'){options.headers={'Content-Type':'application/json','Idempotency-Key':requestId()};options.body=JSON.stringify(body||{});}const response=await fetch(url,options);if(!response.ok){let data;try{data=await response.json();}catch{}throw Error(data?.error?.message||`HTTP ${response.status}`);}return response.json();}
function message(text,error=false){el('observe-message').textContent=text;el('observe-message').style.color=error?'#a13b21':'#226b65';}
function choices(){const target=el('observe-target').value;const list=models.filter(m=>m.target_id===target);el('observe-model').replaceChildren(...list.map(m=>{const option=document.createElement('option');option.value=m.model_version;option.textContent=`${m.model_version.slice(0,8)} · ${m.created_at.slice(0,10)}`;return option;}));if(list.some(m=>m.model_version===requestedVersion))el('observe-model').value=requestedVersion;}
async function catalogue(){models=await request('/api/teach/models?state=published');const targets=await request('/api/teach/targets');const wanted=models.find(m=>m.model_version===requestedVersion)||models[0];el('observe-target').replaceChildren(...targets.filter(t=>models.some(m=>m.target_id===t.target_id)).map(t=>{const option=document.createElement('option');option.value=t.target_id;option.textContent=t.name;return option;}));if(wanted){el('observe-target').value=wanted.target_id;requestedVersion=wanted.model_version;}choices();message(models.length?'Model selected. Start Observe to load it.':'Publish a model in Teach Target first.');}
function isAnnotated(object){return object.display_confirmed ?? object.pick_confirmed;}
function draw(){const canvas=el('observe-canvas');if(!bitmap)return;canvas.width=bitmap.width;canvas.height=bitmap.height;const ctx=canvas.getContext('2d');ctx.drawImage(bitmap,0,0);if(mask&&el('show-mask').checked)ctx.drawImage(mask,0,0,canvas.width,canvas.height);if(!current)return;const sx=canvas.width/current.original_size.width,sy=canvas.height/current.original_size.height;ctx.font='12px system-ui';ctx.lineWidth=1.5;for(const object of current.objects){if(!isAnnotated(object))continue;const [x,y]=object.bbox_xyxy;const title=`${object.label} #${object.display_id ?? object.track_key.split(':').at(-1)} ${Math.round(object.confidence*100)}%`;ctx.fillStyle='#003d31';ctx.fillRect(x*sx,y*sy-17,ctx.measureText(title).width+8,18);ctx.fillStyle='#baffd5';ctx.fillText(title,x*sx+4,y*sy-4);if(el('show-box').checked){ctx.strokeStyle='#f5c64c';ctx.setLineDash(object.box_estimated ? [4, 3] : []);for(const edge of object.box_edges_rgb_pixels||[]){ctx.beginPath();ctx.moveTo(edge[0][0]*sx,edge[0][1]*sy);ctx.lineTo(edge[1][0]*sx,edge[1][1]*sy);ctx.stroke();}}}}
async function loadFrame(snapshot){if(pending||snapshot.frame_key===renderedKey)return;pending=true;let rgb,alpha;try{const responses=await Promise.all([fetch(snapshot.rgb_url,{cache:'no-store'}),fetch(snapshot.mask_url,{cache:'no-store'})]);if(responses.some(r=>r.status===410))return;if(responses.some(r=>!r.ok))throw Error('Preview unavailable');[rgb,alpha]=await Promise.all(responses.map(async r=>createImageBitmap(await r.blob())));if(activeRun?.run_id!==snapshot.run_id){rgb.close();alpha.close();return;}bitmap?.close();mask?.close();bitmap=rgb;mask=alpha;current=snapshot;renderedKey=snapshot.frame_key;el('observe-empty').hidden=true;draw();displayedAt.push(performance.now());while(displayedAt.length>60)displayedAt.shift();}catch(error){rgb?.close();alpha?.close();message(error.message,true);}finally{pending=false;}}
const objectStates = {
    tracked: 'Tracked', picking: 'Picking', holding: 'Holding',
    placing: 'Placing', placed: 'Placed', unverified: 'Unverified', failed: 'Failed', canceled: 'Canceled',
};
const followLatest = el('observe-cloud-latest');
const cloudCanvas = el('observe-cloud-canvas');
const cloudStatus = el('observe-cloud-status');
let recentObjects = [], rowsKey = null, historyRunId = null;
let pinnedTrackId = null, shownTrackId = null;
let cloudRevision = null, cloudRequest = null, cloudMessage = '';

function clearCloud() {
    cloudRequest?.abort();
    cloudRequest = null;
    cloudRevision = null;
    cloudMessage = '';
    cloudCanvas.hidden = true;
}

function highlightCloudSelection() {
    for (const button of el('observe-objects').querySelectorAll('button[data-track-id]')) {
        button.setAttribute('aria-pressed', String(button.dataset.trackId === shownTrackId));
    }
}

async function refreshCloud() {
    followLatest.hidden = pinnedTrackId === null;
    if (document.hidden) return;
    const object = pinnedTrackId ? recentObjects.find(row => row.track_id === pinnedTrackId) : recentObjects[0];
    const trackId = object?.track_id ?? null;
    const runId = activeRun?.run_id;
    if (trackId !== shownTrackId) {
        clearCloud();
        shownTrackId = trackId;
        highlightCloudSelection();
    }
    if (!runId || !object) {
        cloudStatus.textContent = pinnedTrackId ? 'Selected target is outside the recent object list.' : 'No confirmed targets.';
        return;
    }
    const label = '#' + object.display_id;
    cloudStatus.textContent = label + ' · ' + (cloudMessage || 'Loading…');
    if (cloudRequest) return;
    const controller = new AbortController();
    cloudRequest = controller;
    const query = cloudRevision ? '?' + new URLSearchParams({cloud_revision: cloudRevision}) : '';
    try {
        const url = '/api/observe/runs/' + runId + '/objects/' + encodeURIComponent(trackId) + '/point-cloud' + query;
        const response = await fetch(url, {cache: 'no-store', signal: controller.signal});
        if (!response.ok) throw Error('Point cloud unavailable');
        const {gantry_cloud: cloud, gantry_cloud_revision: revision} = await response.json();
        if (cloudRequest !== controller || activeRun?.run_id !== runId) return;
        if (cloud && revision !== cloudRevision) {
            renderCloud(cloudCanvas, cloud);
            cloudCanvas.hidden = false;
            const stamp = cloud.capture_stamp_ns;
            const capture = stamp ? ' · Sim ' + (Number(stamp) / 1e9).toFixed(2) + ' s' : '';
            cloudMessage = cloud.scans + (cloud.scans === 1 ? ' scan' : ' scans') + capture;
        } else if (!revision) {
            cloudCanvas.hidden = true;
            cloudMessage = 'No gantry point cloud available';
        }
        cloudRevision = revision;
        cloudStatus.textContent = label + ' · ' + cloudMessage;
    } catch (error) {
        if (cloudRequest !== controller) return;
        clearCloud();
        cloudStatus.textContent = label + ' · ' + error.message;
    } finally {
        if (cloudRequest === controller) cloudRequest = null;
    }
}

followLatest.addEventListener('click', () => {
    pinnedTrackId = null;
    refreshCloud();
});

function renderObjects(objects, detectedCount = 0, counts = {}) {
    recentObjects = objects;
    const table = el('observe-objects');
    el('object-count').textContent = detectedCount + ' detected';
    for (const state of ['placed', 'failed', 'unverified']) {
        el('object-' + state).textContent = (counts[state] ?? 0) + ' ' + objectStates[state];
    }
    el('object-empty').hidden = objects.length > 0;
    const key = JSON.stringify(objects.map(({track_id, display_id, label, state, arm, reason}) =>
        [track_id, display_id, label, state, arm, reason]));
    if (key === rowsKey) return;
    rowsKey = key;
    table.querySelector('tbody').replaceChildren(...objects.map(object => {
        const row = document.createElement('tr');
        row.title = object.reason || object.track_id;
        row.dataset.state = object.state;
        const idCell = document.createElement('td');
        const button = document.createElement('button');
        button.type = 'button';
        button.dataset.trackId = object.track_id;
        button.textContent = '#' + object.display_id;
        button.setAttribute('aria-label', 'Show gantry point cloud for #' + object.display_id);
        button.addEventListener('click', () => {
            pinnedTrackId = object.track_id;
            refreshCloud();
        });
        idCell.append(button);
        row.append(idCell);
        const arm = {arm_01: 'Arm 1', arm_02: 'Arm 2'}[object.arm] || '—';
        for (const value of [object.label, objectStates[object.state] || 'Unknown', arm]) {
            const cell = document.createElement('td');
            cell.textContent = value;
            row.append(cell);
        }
        return row;
    }));
    highlightCloudSelection();
}

async function refreshObjects(run) {
    const runId = run?.run_id ?? null;
    if (historyRunId !== runId) {
        historyRunId = runId;
        pinnedTrackId = null;
        shownTrackId = null;
        clearCloud();
        renderObjects([]);
    }
    if (runId) {
        const history = await request('/api/observe/runs/' + runId + '/objects');
        if (activeRun?.run_id !== history.run_id) return;
        renderObjects(history.objects, history.detected_count, history.outcome_counts);
    }
    refreshCloud();
}
async function refreshPreview(){
 if(previewBusy||controlBusy||!activeRun||performance.now()-previewStartedAt<previewDelayMs)return;
 if(['stopped','failed'].includes(activeRun.state)&&current?.run_id===activeRun.run_id)return;
 previewBusy=true;previewStartedAt=performance.now();const runId=activeRun.run_id;
 try{const snapshot=await request(`/api/observe/runs/${runId}/snapshot`);if(activeRun?.run_id===runId&&snapshot.frame_key)await loadFrame(snapshot);}
 catch(error){if(activeRun?.run_id===runId)message(error.message,true);}
 finally{previewBusy=false;}
}
function controls(status){const state=status.run?.state;const running=['starting','waiting_for_frame','observing','stopping'].includes(state);el('observe-start').disabled=controlBusy||running||!el('observe-model').value;el('observe-stop').disabled=controlBusy||!status.run||['stopped','failed'].includes(state);el('observe-target').disabled=controlBusy||running||!!status.active_model_version;el('observe-model').disabled=controlBusy||running||!!status.active_model_version;}
async function poll(){if(pollBusy)return;pollBusy=true;try{const status=await request('/api/observe/status');el('connection-state').textContent='Connected';if(status.active_model_version&&el('observe-model').value!==status.active_model_version){const resident=models.find(m=>m.model_version===status.active_model_version);if(resident){requestedVersion=resident.model_version;el('observe-target').value=resident.target_id;choices();}}controls(status);el('observe-diagnostics').textContent=JSON.stringify(status,null,2);const run=status.run;activeRun=run;previewDelayMs=1000/Math.max(1,Math.min(20,status.preview_hz||10));const health=status.health||{};el('observe-stats').textContent=[run?.state||'Stopped',status.active_model_version?`Model ${status.active_model_version.slice(0,8)}`:'No resident model',status.source_state,status.confidence_threshold==null?'':`Confidence ${Math.round(status.confidence_threshold*100)}%`,health.input_hz==null?'':`Input ${health.input_hz.toFixed(1)} Hz`,health.inference_hz==null?'':`Inference ${health.inference_hz.toFixed(1)} Hz`,`Display ${displayFps().toFixed(1)} FPS`,health.inference_ms==null?'':`${Math.round(health.inference_ms)} ms`,status.display_age_sec==null?'':`Display ${status.display_age_sec.toFixed(1)} s`].filter(Boolean).join(' · ');if(run&&!controlBusy){message(run.message,run.state==='failed');}await refreshObjects(run);if(status.source_state==='stale'&&run?.state==='observing')message('Camera stale / clock paused · last frame frozen');}catch(error){el('connection-state').textContent='Unavailable';message(error.message,true);}finally{pollBusy=false;}}
el('observe-target').onchange=choices;
el('show-mask').onchange=draw;el('show-box').onchange=draw;
el('observe-start').onclick=async()=>{controlBusy=true;try{message('Loading model…');const status=await request('/api/observe/status');let activation;if(status.activation_id){const existing=await request('/api/observe/model-activations/'+status.activation_id);if((status.active_model_version||existing.state==='loading')&&existing.model_version===el('observe-model').value)activation=existing;}if(!activation)activation=await request('/api/observe/model-activations','POST',{model_version:el('observe-model').value});let ack=activation;while(ack.state==='loading'){await new Promise(resolve=>setTimeout(resolve,500));ack=await request(`/api/observe/model-activations/${activation.activation_id}`);}if(ack.state!=='active')throw Error(ack.message);await request('/api/observe/runs','POST',{activation_id:ack.activation_id});message('Waiting for gantry camera…');}catch(error){message(error.message,true);}finally{controlBusy=false;await poll();}};
el('observe-stop').onclick=async()=>{controlBusy=true;try{const status=await request('/api/observe/status');if(status.run)await request(`/api/observe/runs/${status.run.run_id}/stop`,'POST');}catch(error){message(error.message,true);}finally{controlBusy=false;await poll();}};
catalogue().then(poll).catch(error=>message(error.message,true));setInterval(poll,1000);setInterval(()=>{refreshPreview();},20);

el('preview-pick').onclick = async () => {
    const output = el('pick-preview-output');
    const runId = activeRun?.run_id;
    if (!runId) {
        output.textContent = 'Start Observe to preview pick candidates.';
        return;
    }
    try {
        const preview = await request(`/api/observe/runs/${runId}/pick-preview`);
        if (activeRun?.run_id !== runId) {
            output.textContent = 'Observe run changed. Refresh the preview.';
            return;
        }
        output.textContent = preview.candidates.map(candidate => {
            const position = candidate.position_world_m.map(value => value.toFixed(3)).join(', ');
            return `${candidate.track_key} | ${(candidate.confidence * 100).toFixed(1)}% | measured XYZ (m): ${position}`;
        }).join('\n') || 'No fresh pick candidates.';
    } catch (error) {
        output.textContent = error.message;
    }
};
