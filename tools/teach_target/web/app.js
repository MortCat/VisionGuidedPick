'use strict';
const $=id=>document.getElementById(id);
const state={target:null,image:null,annotation:null,bitmap:null,mask:null,instances:[],instanceIndex:-1,bbox:null,include:[],exclude:[],undo:[],mode:'box',job:null,polling:false,busy:false};
function requestId(){return globalThis.crypto?.randomUUID?.()||Array.from(crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join('');}
async function api(path,method='GET',body,key){const headers={};if(method!=='GET'){headers['Idempotency-Key']=key||requestId();}if(body&&!(body instanceof FormData))headers['Content-Type']='application/json';const response=await fetch(path,{method,headers,body:body instanceof FormData?body:body?JSON.stringify(body):undefined});if(response.status===204)return;const data=await response.json();if(!response.ok)throw Error(data.error?.message||JSON.stringify(data));return data;}
function message(text,error=false){$('message').textContent=text;$('message').style.color=error?'#a13b21':'#28745a';}
function action(id,fn){$(id).onclick=async()=>{try{$(id).disabled=true;await fn();}catch(e){message(e.message,true);}finally{$(id).disabled=false;controls();}};}
function controls(){for(const id of ['auto','save','next','no-target','remove-image','confirm-remove-image','cancel-remove-image','add-instance','remove-instance','instance-select'])$(id).disabled=!state.image||state.busy;$('remove-instance').disabled=!state.image||state.busy||state.instanceIndex<0;$('train').disabled=!state.target||state.busy;for(const id of ['targets','new','files','validation-files'])$(id).disabled=state.busy;for(const container of ['images','validation-images'])for(const button of $(container).children)button.disabled=state.busy;}
async function refresh(){const rows=await api('/api/teach/targets');$('targets').replaceChildren(...rows.map(x=>new Option(x.name,x.target_id)));if(state.target)$('targets').value=state.target.target_id;else if(rows.length)await selectTarget(rows[0].target_id);}
function renderInstances(){const select=$('instance-select');select.replaceChildren(...state.instances.map((item,index)=>new Option(`Instance ${index+1}`,String(index))));if(state.instanceIndex<0)select.add(new Option('New instance','-1'));select.value=String(state.instanceIndex);$('remove-instance').disabled=state.instanceIndex<0||state.busy;}
function commitMask(){if(!state.mask)return;if(state.instanceIndex<0){state.instances.push({instance_id:requestId().replaceAll('-',''),mask:state.mask});state.instanceIndex=state.instances.length-1;}else state.instances[state.instanceIndex].mask=state.mask;}
function resetPrompt(){state.bbox=null;state.include=[];state.exclude=[];state.undo=[];}
async function renderSummary(){if(!state.target)return;const summary=await api(`/api/teach/targets/${state.target.target_id}/dataset-summary`);$('dataset-summary').textContent=`Next run · Train: ${summary.train_count} · Validation: ${summary.validation_count} · ${summary.pending_count} to review`+(summary.validation_mode==='automatic_holdout'?' · Automatic holdout until validation images are added.':' · Specified validation');}
async function selectTarget(id){$('remove-confirmation').hidden=true;const previous=state.target?.target_id;state.target=await api('/api/teach/targets/'+id);$('targets').value=id;$('title').textContent=state.target.name;if(previous!==id||(state.image&&!state.target.images.some(item=>item.image_id===state.image.image_id))){state.image=null;state.annotation=null;state.bitmap?.close();state.bitmap=null;state.mask=null;state.instances=[];state.instanceIndex=-1;resetPrompt();$('image-state').textContent='';$('image-origin').textContent='';draw();}for(const container of ['images','validation-images'])$(container).replaceChildren();for(const item of state.target.images){const button=document.createElement('button');button.dataset.imageId=item.image_id;button.title=[item.original_filename||item.image_id,`${item.width} × ${item.height}`].join(' · ');button.classList.toggle('selected',item.image_id===state.image?.image_id);const img=document.createElement('img');img.src=item.preview_url;button.append(img,document.createTextNode(item.annotation_state==='reviewed'?(item.annotation_outcome==='no_target'?'No Target':'Saved'):'Needs review'));button.onclick=()=>openImage(item).catch(e=>message(e.message,true));$(item.purpose==='validation'?'validation-images':'images').append(button);}state.job=state.target.latest_job_id;renderInstances();controls();await renderSummary();if(state.job)await poll();else $('job').textContent='No training job';}
action('new',async()=>{const created=await api('/api/teach/targets','POST',{name:$('name').value});await refresh();await selectTarget(created.target_id);$('name').value='';message('Target created. Upload your images.');});
$('targets').onchange=()=>selectTarget($('targets').value).catch(e=>message(e.message,true));
async function uploadImages(input,purpose){
 if(!state.target){message('Create a target first',true);return;}
 const target=state.target.target_id,failures=[];let added=0,existing=0;
 state.busy=true;controls();
 try{
  const before=await api('/api/teach/targets/'+target),known=new Set(before.images.map(row=>row.image_id));
  for(const file of input.files){
   try{
    message(`Uploading ${file.name}…`);
    const data=new FormData();data.append('file',file);data.append('purpose',purpose);
    const image=await api(`/api/teach/targets/${target}/images`,'POST',data);
    if(known.has(image.image_id))existing++;else{known.add(image.image_id);added++;}
   }catch(error){failures.push(file.name+': '+error.message);}
  }
  await selectTarget(target);input.value='';
 }finally{state.busy=false;controls();}
 if(!state.image&&state.target.images.length)await openImage(state.target.images[0]);
 const rows=state.target.images,training=rows.filter(row=>row.purpose==='teaching').length,validation=rows.filter(row=>row.purpose==='validation').length;
 const summary=`Added: ${added} · Already present: ${existing} · Failed: ${failures.length} · Training images: ${training} · Validation images: ${validation}`;
 $('upload-result').textContent=summary+(failures.length?' · '+failures.join('; '):'');
 message(summary+(existing?' · Identical image content is kept once.':'')+(failures.length?' · '+failures.join('; '):''),failures.length>0);
}
$('files').onchange=()=>uploadImages($('files'),'teaching').catch(e=>message(e.message,true));
$('validation-files').onchange=()=>uploadImages($('validation-files'),'validation').catch(e=>message(e.message,true));
async function openImage(item){if(state.busy)return;$('remove-confirmation').hidden=true;const target=state.target.target_id;const annotation=await api(`/api/teach/targets/${target}/images/${item.image_id}/annotation`);const bitmap=await createImageBitmap(await(await fetch(item.preview_url)).blob());if(target!==state.target.target_id||state.busy||!state.target.images.some(row=>row.image_id===item.image_id)){bitmap.close();return;}state.bitmap?.close();Object.assign(state,{image:item,annotation,bitmap,mask:annotation.instances[0]?.mask||null,instances:structuredClone(annotation.instances),instanceIndex:annotation.instances.length?0:-1,bbox:null,include:[],exclude:[],undo:[],mode:'box'});draw();controls();$('image-state').textContent=`${item.width} × ${item.height} · ${annotation.state}`;setMode('box');message('Drag around the target to generate a contour.');$('image-origin').textContent=`${item.purpose==='validation'?'Validation':'Training'} · ${item.original_filename||item.image_id}`;renderInstances();for(const container of ['images','validation-images'])for(const button of $(container).children)button.classList.toggle('selected',button.dataset.imageId===item.image_id);}
function decode(rle){const mask=new Uint8Array(rle.size[0]*rle.size[1]);let pos=0,on=0;for(const n of rle.counts){if(on)mask.fill(1,pos,pos+n);pos+=n;on=1-on;}return mask;}
function draw(){const canvas=$('editor');$('empty').hidden=!!state.bitmap;if(!state.bitmap){canvas.width=0;canvas.height=0;return;}canvas.width=state.bitmap.width;canvas.height=state.bitmap.height;const ctx=canvas.getContext('2d');ctx.drawImage(state.bitmap,0,0);const sx=canvas.width/state.image.width,sy=canvas.height/state.image.height;if(state.mask||state.instances.length){const combined=new Uint8Array(state.image.width*state.image.height);for(const instance of state.instances){const decoded=decode(instance.mask);for(let i=0;i<combined.length;i++)combined[i]|=decoded[i];}if(state.mask){const decoded=decode(state.mask);for(let i=0;i<combined.length;i++)combined[i]|=decoded[i];}const mask=combined,pixels=ctx.getImageData(0,0,canvas.width,canvas.height);for(let y=0;y<canvas.height;y++)for(let x=0;x<canvas.width;x++){const u=Math.min(state.image.width-1,Math.floor(x/sx)),v=Math.min(state.image.height-1,Math.floor(y/sy));if(mask[v+u*state.image.height]){const i=(y*canvas.width+x)*4;pixels.data[i]=pixels.data[i]*.6;pixels.data[i+1]=pixels.data[i+1]*.6+100;pixels.data[i+2]=pixels.data[i+2]*.6+60;}}ctx.putImageData(pixels,0,0);}if(state.bbox){const [x1,y1,x2,y2]=state.bbox;ctx.strokeStyle='#ffc857';ctx.lineWidth=2;ctx.setLineDash([6,4]);ctx.strokeRect(x1*sx,y1*sy,(x2-x1)*sx,(y2-y1)*sy);ctx.setLineDash([]);}for(const [points,color] of [[state.include,'#17b980'],[state.exclude,'#f05858']])for(const p of points){ctx.beginPath();ctx.fillStyle=color;ctx.arc(p[0]*sx,p[1]*sy,5,0,Math.PI*2);ctx.fill();ctx.strokeStyle='white';ctx.stroke();}}
function setMode(mode){state.mode=mode;$('adjust').open=false;$('instruction').textContent=mode==='box'?'Drag a loose box around the complete target. Segmentation starts when you release.':mode==='include'?'Click a target area that should be included. Auto Contour will refine the mask.':'Click a background area that should be excluded. Auto Contour will refine the mask.';for(const name of ['box','include','exclude'])$(name).dataset.active=String(name===mode);}
function coordinate(event){const rect=$('editor').getBoundingClientRect();return [Math.max(0,Math.min(state.image.width-.001,(event.clientX-rect.left)*state.image.width/rect.width)),Math.max(0,Math.min(state.image.height-.001,(event.clientY-rect.top)*state.image.height/rect.height))];}
let start=null;
$('editor').onpointerdown=e=>{if(!state.image||state.busy)return;start=coordinate(e);$('editor').setPointerCapture(e.pointerId);};
$('editor').onpointermove=e=>{if(!start||state.mode!=='box')return;const end=coordinate(e);state.bbox=[Math.min(start[0],end[0]),Math.min(start[1],end[1]),Math.max(start[0],end[0]),Math.max(start[1],end[1])];draw();};
$('editor').onpointerup=async e=>{if(!start)return;const point=coordinate(e);if(state.mode==='box'){state.bbox=[Math.min(start[0],point[0]),Math.min(start[1],point[1]),Math.max(start[0],point[0]),Math.max(start[1],point[1])];state.include=[];state.exclude=[];state.undo=[];}else {state.undo.push({mask:state.mask,include:[...state.include],exclude:[...state.exclude]});(state.mode==='include'?state.include:state.exclude).push(point);}start=null;draw();if(state.mode==='box'&&(state.bbox[2]-state.bbox[0]<3||state.bbox[3]-state.bbox[1]<3)){state.bbox=null;draw();message('Box not created. Press and drag across the target, then release.',true);return;}try{await autoContour();}catch(error){message(error.message,true);}};
$('editor').onpointercancel=()=>{start=null;};
async function autoContour(){if(!state.image)throw Error('Select an image');if(!state.bbox&&!state.mask)throw Error('Draw a rough box around the target first');const id=state.image.image_id;state.busy=true;controls();message('Finding contour with SAM… First use may take longer.');try{const prompt={positive_points:state.include,negative_points:state.exclude};if(state.bbox)prompt.bbox_xyxy=state.bbox;if(state.mask&&state.mode!=='box')prompt.initial_mask=state.mask;const result=await api(`/api/teach/targets/${state.target.target_id}/images/${id}/mask-proposals`,'POST',prompt);if(state.image?.image_id!==id)return;state.mask=result.mask;commitMask();renderInstances();draw();$('image-state').textContent=`${state.image.width} × ${state.image.height} · Unsaved contour`;message(`Contour ready · ${Math.round(result.elapsed_ms)} ms. Save when it looks correct.`);}finally{state.busy=false;controls();}}
action('auto',autoContour);for(const mode of ['box','include','exclude'])action(mode,()=>setMode(mode));
action('undo',()=>{const previous=state.undo.pop();if(previous){Object.assign(state,previous);commitMask();draw();message('Correction undone.');}});
action('reset',()=>{state.instances=structuredClone(state.annotation?.instances||[]);state.instanceIndex=state.instances.length?0:-1;state.mask=state.instances[0]?.mask||null;resetPrompt();renderInstances();draw();message('Returned to saved contour.');});
async function saveInstances(instances){const result=await api(`/api/teach/targets/${state.target.target_id}/images/${state.image.image_id}/annotation`,'PUT',{expected_revision:state.annotation.revision,width:state.image.width,height:state.image.height,instances});state.annotation=result.annotation;state.instances=structuredClone(result.annotation.instances);state.instanceIndex=state.instances.length?0:-1;state.mask=state.instances[0]?.mask||null;resetPrompt();draw();await selectTarget(state.target.target_id);$('image-state').textContent=`${state.image.width} × ${state.image.height} · ${instances.length?'Saved':'No Target'}`;message(instances.length?'All instance contours saved.':'Confirmed: no target in this image.');}
action('save',async()=>{if(!state.image)throw Error('Select an image');commitMask();if(!state.instances.length)throw Error('Use No Target to confirm an empty image');await saveInstances(state.instances);});
action('no-target',async()=>{if(!state.image)throw Error('Select an image');await saveInstances([]);});
action('add-instance',()=>{commitMask();state.mask=null;state.instanceIndex=-1;resetPrompt();renderInstances();draw();setMode('box');message('Draw a box around the next target. Save Contour saves every instance.');});
action('remove-instance',()=>{if(state.instanceIndex<0)return;state.instances.splice(state.instanceIndex,1);state.instanceIndex=state.instances.length?0:-1;state.mask=state.instances[0]?.mask||null;resetPrompt();renderInstances();draw();message('Instance removed. Save Contour to keep the change, or No Target if none remain.');});
$('instance-select').onchange=()=>{commitMask();state.instanceIndex=Number($('instance-select').value);state.mask=state.instances[state.instanceIndex]?.mask||null;resetPrompt();renderInstances();draw();};
action('remove-image',()=>{
 if(!state.image||state.busy)return;
 const item=state.image,name=item.original_filename||item.image_id;
 $('remove-prompt').textContent=`Remove "${name}" from ${item.purpose==='validation'?'Validation':'Training'}? Unsaved edits will be discarded. Existing runs and models are kept.`;
 $('remove-confirmation').hidden=false;
 message('Confirm image removal below.');
});
action('cancel-remove-image',()=>{$('remove-confirmation').hidden=true;message('Image kept.');});
action('confirm-remove-image',async()=>{
 if(!state.image||state.busy||$('remove-confirmation').hidden)return;
 $('remove-confirmation').hidden=true;
 const target=state.target.target_id,item=state.image,images=state.target.images,index=images.findIndex(row=>row.image_id===item.image_id);
 state.busy=true;controls();message('Removing image…');
 try{
  await api(`/api/teach/targets/${target}/images/${item.image_id}?expected_revision=${state.target.draft_revision}`,'DELETE');
  await selectTarget(target);
 }catch(error){
  try{await selectTarget(target);}catch{}
  throw error;
 }finally{state.busy=false;controls();}
 const remaining=state.target.images,sameSplit=remaining.filter(row=>row.purpose===item.purpose),next=sameSplit.find(row=>images.findIndex(old=>old.image_id===row.image_id)>index)||sameSplit.at(-1)||remaining[0];
 if(next)await openImage(next);
 message(`Image removed. Next training run uses the updated dataset. Existing runs and models are unchanged.`);
});
action('next',async()=>{const images=state.target.images,index=images.findIndex(x=>x.image_id===state.image?.image_id);if(images.length)await openImage(images[(index+1)%images.length]);});
action('train',async()=>{if(!state.target)throw Error('Select a target');state.target=await api('/api/teach/targets/'+state.target.target_id);const row=await api(`/api/teach/targets/${state.target.target_id}/training-jobs`,'POST',{draft_revision:state.target.draft_revision});state.job=row.job_id;await poll();});
let renderedHistory, renderedJob;
async function poll(){if(!state.job||state.polling)return;state.polling=true;try{const jobId=state.job;const row=await api('/api/teach/training-jobs/'+jobId);
if(state.job!==jobId)return;
if(renderedJob!==jobId){
  $('result').textContent='';
  delete $('training-charts').dataset.job;
  $('open-observe').hidden=true;
  renderedJob=jobId;
}
const historyKey=JSON.stringify([jobId,row.training_history]);
if(renderedHistory!==historyKey){renderTrainingCharts(row.training_history);renderedHistory=historyKey;}
const active=['preparing','training','evaluating','cancelling'].includes(row.state);const eta=row.eta_sec==null?'Estimating…':`${Math.max(0,Math.ceil(row.eta_sec-(Date.now()/1000-(row.eta_updated_at||Date.now()/1000))))} s`;const completed=row.completed_epochs??Math.max(0,row.epoch-1);$('job').textContent=`${row.state} · epoch ${row.epoch}/${row.total_epochs} · ${completed}/${row.total_epochs} completed\n${row.state==='succeeded'?'Training completed; review validation results':row.message}\nElapsed: ${Math.round(row.elapsed_sec)} s`+(active?` · Training ETA: ${row.state==='training'?eta:'Estimating…'}`:'')+(row.error?.message?`\n${row.error.message}`:'');if(row.train_count!=null)$('job').textContent+='\nOriginal training images: '+row.train_count+' · Augmented: '+row.augmented_train_count;$('cancel').disabled=!['preparing','training','evaluating'].includes(row.state);$('publish').disabled=row.state!=='succeeded';if(row.state==='succeeded'){const published=await api('/api/teach/models/'+row.model_version);$('open-observe').hidden=published.state!=='published';$('open-observe').href='/observe?model_version='+row.model_version;const result=await api('/api/teach/training-jobs/'+state.job+'/results');const iou=Object.hasOwn(result.metrics,'positive_mean_union_iou')?result.metrics.positive_mean_union_iou:result.metrics.mean_union_iou;const iouText=iou==null?'N/A':Number(iou).toFixed(4);$('result').textContent=`Train: ${result.train_count} original · Augmented: ${result.augmented_train_count??'Unknown'} · Selected epoch: ${result.selected_epoch??'Unknown'} · Validation: ${result.validation_count} · Positive-image mask IoU: ${iouText} · ${result.metrics_source}`+(iou===0?' — Needs review: validation predictions failed at the current confidence threshold.':'');if(result.metrics.negative_image_count){$('result').textContent+=` · Negative false alarms: ${result.metrics.negative_false_positive_images}/${result.metrics.negative_image_count}`;}if($('training-charts').dataset.job!==state.job){$('training-charts').dataset.job=state.job;renderTrainingCharts(result.training_history);}}return row;}finally{state.polling=false;}}
action('cancel',async()=>{if(state.job)await api('/api/teach/training-jobs/'+state.job+'/cancel','POST');await poll();});action('publish',async()=>{const row=await api('/api/teach/training-jobs/'+state.job);if(!row.model_version)throw Error('No completed model');await api('/api/teach/models/'+row.model_version+'/publish','POST');$('result').textContent='Published · Ready for Observe (not active)';$('open-observe').hidden=false;$('open-observe').href='/observe?model_version='+row.model_version;});
controls();$('cancel').disabled=true;$('publish').disabled=true;setInterval(()=>{poll().catch(e=>message(e.message,true));},1000);refresh().then(()=>{$('connection-state').textContent='Ready';}).catch(e=>{$('connection-state').textContent='Unavailable';message(e.message,true);});

function renderTrainingCharts(history){
 const host=$('training-charts');host.replaceChildren();
 if(!history?.available){host.textContent='Loss history was not recorded for this job.';return;}
 const groups=[['Total loss',['train/loss','val/loss']],['Classification loss',['train/loss_ce','val/loss_ce']],['Box loss',['train/loss_bbox','val/loss_bbox']],['Mask cross-entropy',['train/loss_mask_ce','val/loss_mask_ce']],['Mask Dice loss',['train/loss_mask_dice','val/loss_mask_dice']],['Trainer validation mAP @ .50:.95',['val/mAP_50_95','val/segm_mAP_50_95']]];
 const ns='http://www.w3.org/2000/svg';
 function svgNode(tag,attrs,text){const node=document.createElementNS(ns,tag);for(const [key,value] of Object.entries(attrs))node.setAttribute(key,value);if(text!==undefined)node.textContent=text;return node;}
 for(const [title,keys] of groups){
  const series=keys.map(key=>({key,points:history.epochs.filter(row=>Number.isFinite(row[key])).map(row=>[row.epoch,row[key]])})).filter(row=>row.points.length);
  if(!series.length)continue;
  const card=document.createElement('div');card.className='metric-chart';const heading=document.createElement('strong');heading.textContent=title;card.append(heading);
  const plot=svgNode('svg',{viewBox:'0 0 440 220',role:'img','aria-label':title});
  const values=series.flatMap(row=>row.points.map(point=>point[1]));let low=Math.min(...values),high=Math.max(...values);if(high===low){low=Math.max(0,low-.1);high+=.1;}const padding=(high-low)*.08;low=Math.max(0,low-padding);high+=padding;
  const maxEpoch=Math.max(2,...history.epochs.map(row=>row.epoch));const x=epoch=>48+(epoch-1)/(maxEpoch-1)*370,y=value=>180-(value-low)/(high-low)*155;
  for(let tick=0;tick<4;tick++){const value=low+(high-low)*tick/3,position=y(value);plot.append(svgNode('line',{x1:48,y1:position,x2:418,y2:position,stroke:'#e2e8ef'}),svgNode('text',{x:43,y:position+4,'text-anchor':'end','font-size':10,fill:'#60758a'},value.toFixed(3)));}
  plot.append(svgNode('text',{x:48,y:198,'font-size':10},'1'),svgNode('text',{x:418,y:198,'text-anchor':'end','font-size':10},String(maxEpoch)),svgNode('text',{x:230,y:214,'text-anchor':'middle','font-size':11},'Epoch'));
  const legend=document.createElement('p');legend.className='hint';
  series.forEach((row,index)=>{const color=index?'#e58b25':'#1967a6';plot.append(svgNode('polyline',{points:row.points.map(point=>`${x(point[0])},${y(point[1])}`).join(' '),fill:'none',stroke:color,'stroke-width':2}));for(const [epoch,value] of row.points){const dot=svgNode('circle',{cx:x(epoch),cy:y(value),r:2,fill:color});dot.append(svgNode('title',{},`${row.key} 繚 epoch ${epoch}: ${value}`));plot.append(dot);}const label=document.createElement('span');label.style.color=color;label.textContent=`${row.key} (last ${row.points.at(-1)[1].toFixed(4)})  `;legend.append(label);});
  card.append(plot,legend);host.append(card);
 }
 const note=document.createElement('p');note.className='hint';note.textContent='Source: '+history.source+'. Trainer validation mAP and reloaded-model IoU measure different stages. Curves do not establish mixed-clutter deployment accuracy.';host.append(note);
}
