const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const project=process.argv[2]||path.resolve(__dirname,'../..');
const elements=new Map(),deletes=[];
const images=[{image_id:'a',purpose:'teaching'},{image_id:'b',purpose:'teaching'},{image_id:'c',purpose:'validation'}].map(x=>({...x,width:32,height:24,original_filename:x.image_id+'.png',preview_url:'/preview/'+x.image_id}));
let profile={target_id:'t',name:'Target',draft_revision:7,latest_job_id:null,images},conflict=false,closed=0;
function element(){return {disabled:false,hidden:false,value:'',dataset:{},style:{},children:[],classList:{toggle(){}},append(...x){this.children.push(...x);},replaceChildren(...x){this.children=x;},add(x){this.children.push(x);},getContext(){return {drawImage(){}};}};}
const context={console,Uint8Array,structuredClone,FormData,window:{confirm(){throw Error("Native confirmation must not be used");}},setInterval(){},Option:function(text,value){return {text,value};},crypto:{getRandomValues(a){return a.fill(17);}},createImageBitmap:async()=>({width:32,height:24,close(){closed++;}}),document:{getElementById(id){if(!elements.has(id))elements.set(id,element());return elements.get(id);},createElement:element,createTextNode:t=>t},fetch:async(url,options={})=>{
 let data,status=200;
 if(options.method==='DELETE'){
  const match=url.match(/images\/(\w+)\?expected_revision=(\d+)/);assert(match,url);deletes.push(match[1]);
  assert.strictEqual(+match[2],profile.draft_revision);
  if(conflict){profile.draft_revision++;status=409;data={error:{message:'Reload target'}};conflict=false;}
  else{profile={...profile,draft_revision:profile.draft_revision+1,images:profile.images.filter(x=>x.image_id!==match[1])};data={draft_revision:profile.draft_revision};}
 }else if(url==='/api/teach/targets')data=[];
 else if(url==='/api/teach/targets/t')data=structuredClone(profile);
 else if(url.endsWith('/dataset-summary'))data={train_count:profile.images.filter(x=>x.purpose==='teaching').length,validation_count:profile.images.filter(x=>x.purpose==='validation').length,pending_count:0,validation_mode:'specified_validation'};
 else if(url.endsWith('/annotation'))data={revision:0,instances:[],state:'pending'};
 else if(url.startsWith('/preview/'))return {blob:async()=>new Blob()};
 else throw Error(url);
 return {ok:status===200,status,json:async()=>data};
}};
vm.createContext(context);vm.runInContext(fs.readFileSync(project+'/tools/teach_target/web/app.js','utf8'),context);
async function removeSelected(){await vm.runInContext("$('remove-image').onclick()",context);assert.strictEqual(elements.get('remove-confirmation').hidden,false);await vm.runInContext("$('confirm-remove-image').onclick()",context);}
(async()=>{
 await new Promise(resolve=>setImmediate(resolve));
 await vm.runInContext("selectTarget('t').then(()=>openImage(state.target.images[0]))",context);
 await vm.runInContext("$('remove-image').onclick()",context);assert.strictEqual(deletes.length,0);assert.strictEqual(elements.get('remove-confirmation').hidden,false);assert(elements.get('remove-prompt').textContent.includes('a.png'));
 await vm.runInContext("$('cancel-remove-image').onclick()",context);assert.strictEqual(deletes.length,0);assert(elements.get('remove-confirmation').hidden);
 conflict=true;await removeSelected();
 assert.strictEqual(vm.runInContext('state.target.draft_revision',context),8);
 assert.strictEqual(vm.runInContext('state.image.image_id',context),'a');assert.strictEqual(profile.images.length,3);
 assert.strictEqual(elements.get('message').textContent,'Reload target');
 await removeSelected();
 assert.strictEqual(vm.runInContext('state.image.image_id',context),'b');
 assert(elements.get('dataset-summary').textContent.includes('Train: 1 · Validation: 1'));
 await removeSelected();
 assert.strictEqual(vm.runInContext('state.image.image_id',context),'c');
 await removeSelected();
 assert.strictEqual(vm.runInContext('state.image',context),null);assert.strictEqual(vm.runInContext('state.bitmap',context),null);
 assert(elements.get('remove-image').disabled&&elements.get('save').disabled);
 assert.strictEqual(elements.get('image-state').textContent,'');assert.strictEqual(elements.get('image-origin').textContent,'');
 assert.strictEqual(elements.get('images').children.length,0);assert.strictEqual(elements.get('validation-images').children.length,0);assert(closed>=3);
 assert(elements.get('dataset-summary').textContent.includes('Train: 0 · Validation: 0'));
 console.log('Image removal passed: inline confirmation without browser dialogs, cancel,409 refresh, both splits, next selection, empty editor and released bitmap; DOM/HTTP contract only');
})().catch(e=>{console.error(e);process.exitCode=1;});
