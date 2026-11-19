const fs=require('fs'),vm=require('vm'),assert=require('assert');
const root=require('path').resolve(__dirname,'../..');
const source=fs.readFileSync(root+'/tools/teach_target/web/observe.js','utf8');
const elements=new Map(),timers=[],painted=[];
let now=0,requests=0,sequence=0,hold=null,pendingResolve=null;
function element(){return {style:{},checked:true,disabled:false,hidden:false,value:'',dataset:{},children:[],listeners:{},replaceChildren(...children){this.children=children;},append(){},setAttribute(){},addEventListener(type,fn){this.listeners[type]=fn;},querySelector(){return element();},querySelectorAll(){return [];},getContext(){return {drawImage(image){painted.push([image.key,image.layer]);}};}};}
function snapshot(){const key='frame'+(++sequence);return {run_id:'r',frame_key:key,original_size:{width:1280,height:720},objects:[],rgb_url:`/frames/${key}?layer=rgb`,mask_url:`/frames/${key}?layer=mask`};}
const context={console,URLSearchParams,location:{search:''},performance:{now:()=>now},crypto:{randomUUID:()=>String(sequence)},setInterval(fn,ms){timers.push(ms);},document:{getElementById(id){if(!elements.has(id))elements.set(id,element());return elements.get(id);},createElement:element},createImageBitmap:async blob=>({width:640,height:360,key:blob.key,layer:blob.layer,close(){}}),fetch:async url=>{
 if(url.startsWith('/api/teach/models'))return new Promise(()=>{}); // Isolate automatic catalogue bootstrap.
 if(url.includes('/snapshot')){requests++;const data=snapshot();if(hold){hold=null;return new Promise(resolve=>{pendingResolve=()=>resolve({ok:true,json:async()=>data});});}return {ok:true,json:async()=>data};}
 const match=url.match(/\/frames\/(frame\d+)\?layer=(rgb|mask)/);
 assert(match,'Unexpected request '+url);return {ok:true,status:200,blob:async()=>({key:match[1],layer:match[2]})};
}};
vm.createContext(context);vm.runInContext(source,context);
(async()=>{
 assert(timers.includes(1000)&&timers.includes(20),'Status remains1Hz while preview has an independent scheduling loop');
 vm.runInContext("activeRun={run_id:'r',state:'observing'};previewDelayMs=100;",context);
 hold=true;
 const first=vm.runInContext('refreshPreview()',context);
 await vm.runInContext('refreshPreview()',context);
 assert.strictEqual(requests,1,'Only one snapshot/preview request in flight');
 pendingResolve();await first;
 assert.deepStrictEqual(painted,[['frame1','rgb'],['frame1','mask']],'RGB and mask come from exactly the same capture frame');
 now=50;await vm.runInContext('refreshPreview()',context);assert.strictEqual(requests,1,'Respect configured10Hz cadence');
 now=100;await vm.runInContext('refreshPreview()',context);assert.strictEqual(requests,2);
 assert.strictEqual(vm.runInContext('renderedKey',context),'frame2');
 now=200;hold=true;const obsolete=vm.runInContext('refreshPreview()',context);
 vm.runInContext("activeRun={run_id:'replacement',state:'observing'}",context);pendingResolve();await obsolete;
 assert.strictEqual(vm.runInContext('renderedKey',context),'frame2','Late prior-run snapshots cannot paint the replacement run');
 assert.strictEqual(vm.runInContext('displayedAt.length',context),2,'FPS counts only uniquely rendered matching frames');
 await catalogueSelectsWithoutActivation();
 console.log('Observe preview contract passed: independent10Hz cadence, one request in flight, matched RGB/mask, old-run rejection, and catalogue preselection without activation; browser visual smoothness not measured');
})().catch(error=>{console.error(error);process.exitCode=1;});

// Setup's default model is preselected, the prompt reflects that, and nothing
// is activated or started until the operator presses Start Observe.
async function catalogueSelectsWithoutActivation(){
 const nodes=new Map(),sent=[];
 const version='2cdd643429fb420e97dcd63141a23e11',target='cc8931e971aa40aea77dd21604f3e0a8';
 const replies={'/api/teach/models?state=published':[{model_version:version,target_id:target,created_at:'2026-11-03T05:23:55Z'}],
  '/api/teach/targets':[{target_id:target,name:'Stanford Bunny'}],
  '/api/observe/status':{run:null,active_model_version:null,activation_id:null,source_state:'waiting',health:{},confidence_threshold:.5,preview_hz:10}};
 const page={console,URLSearchParams,location:{search:''},performance:{now:()=>0},crypto:{randomUUID:()=>'k'},setInterval(){},
  document:{getElementById(id){if(!nodes.has(id))nodes.set(id,element());return nodes.get(id);},createElement:element},
  fetch:async(url,options={})=>{sent.push([options.method||'GET',url]);if(url in replies)return {ok:true,json:async()=>replies[url]};
   if(url.startsWith('/api/observe/runs/'))return {ok:true,json:async()=>({objects:[],counts:{}})};return new Promise(()=>{});}};
 vm.createContext(page);vm.runInContext(source,page);
 for(let i=0;i<20;i++)await new Promise(resolve=>setImmediate(resolve));
 assert.strictEqual(nodes.get('observe-model').value,version,'Setup default model is preselected');
 assert.strictEqual(page.document.getElementById('observe-message').textContent,'Model selected. Start Observe to load it.');
 assert.notStrictEqual(page.document.getElementById('observe-message').style.color,'#a13b21','Preselection is not shown as an error');
 assert(sent.some(([,url])=>url==='/api/observe/status'),'Status is polled after the catalogue loads');
 assert.deepStrictEqual(sent.filter(([method])=>method!=='GET'),[],'No activation or run request before Start Observe');
}
