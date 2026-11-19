const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const root=process.argv[2]||path.resolve(__dirname,'../..'),elements=new Map();
function element(){return {disabled:false,hidden:false,value:'',dataset:{},style:{},children:[],classList:{toggle(){}},append(...x){this.children.push(...x);},replaceChildren(...x){this.children=x;},add(x){this.children.push(x);}};}
const row=(id,purpose='teaching')=>({image_id:id,purpose,width:32,height:24,original_filename:id+'.png',preview_url:'/preview/'+id});
let profile={target_id:'t',name:'Target',draft_revision:3,latest_job_id:null,images:[row('a'),row('b'),row('v','validation')]};
class FormData{constructor(){this.parts={};}append(name,value){this.parts[name]=value;}}
const context={console,Uint8Array,structuredClone,FormData,setInterval(){},Option:function(text,value){return {text,value};},crypto:{getRandomValues(a){return a.fill(17);}},document:{getElementById(id){if(!elements.has(id))elements.set(id,element());return elements.get(id);},createElement:element,createTextNode:t=>t},fetch:async(url,options={})=>{
 let data,status=200;
 if(options.method==='POST'){
  const file=options.body.parts.file;
  if(file.name==='validation-copy.png'){status=409;data={error:{message:'This image already belongs to the other dataset split'}};}
  else{let image=profile.images.find(x=>x.image_id===file.id);if(!image){image=row(file.id);profile.images.push(image);profile.draft_revision++;}data=image;status=201;}
 }else if(url==='/api/teach/targets')data=[];
 else if(url==='/api/teach/targets/t')data=structuredClone(profile);
 else if(url.endsWith('/dataset-summary'))data={train_count:profile.images.filter(x=>x.purpose==='teaching').length,validation_count:1,pending_count:0,validation_mode:'specified_validation'};
 else throw Error(url);
 return {ok:status<400,status,json:async()=>data};
}};
vm.createContext(context);vm.runInContext(fs.readFileSync(root+'/tools/teach_target/web/app.js','utf8'),context);
(async()=>{
 await new Promise(resolve=>setImmediate(resolve));
 await vm.runInContext("selectTarget('t')",context);vm.runInContext("state.image=state.target.images[0]",context);
 elements.get('files').files=[{name:'a.png',id:'a'},{name:'new.png',id:'c'},{name:'same-new-content.png',id:'c'},{name:'validation-copy.png',id:'v'}];
 await vm.runInContext("$('files').onchange()",context);
 const text=elements.get('upload-result').textContent;
 assert(text.startsWith('Added: 1 · Already present: 2 · Failed: 1 · Training images: 3 · Validation images: 1'),text);
 assert(text.includes('validation-copy.png'));assert.strictEqual(profile.images.length,4);
 assert(elements.get('dataset-summary').textContent.includes('Train: 3 · Validation: 1'));
 assert(!elements.get('files').disabled&&!elements.get('validation-files').disabled);
 while(profile.images.filter(x=>x.purpose==='teaching').length<40)profile.images.push(row('extra'+profile.images.length));
 elements.get('files').files=[{name:'forty-first.png',id:'forty-first'}];
 await vm.runInContext("$('files').onchange()",context);
 assert(elements.get('upload-result').textContent.includes('Added: 1 · Already present: 0 · Failed: 0 · Training images: 41'));
 console.log('Upload feedback passed: new/existing/within-batch duplicate/failure, refreshed counts and40→41; DOM/HTTP contract');
})().catch(e=>{console.error(e);process.exitCode=1;});
