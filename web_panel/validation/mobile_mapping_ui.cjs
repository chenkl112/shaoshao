const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[2]||require('path').join(__dirname,'../index.html'),'utf8');
let lineSegments=0;
const fills=[];
const ctx=new Proxy({},{get:(target,key)=>()=>{if(key==='lineTo')lineSegments++;if(key==='fill')fills.push(target.fillStyle)}});
function element(dataset={}){return {dataset,value:'1.5',style:{},classList:{toggle(){}},
  setAttribute(){},removeAttribute(){},addEventListener(){},setCustomValidity(){},
  checkValidity:()=>true,getContext:()=>ctx,getBoundingClientRect:()=>({left:0,top:0}),clientWidth:400,clientHeight:300};}
const elements={};const modeButtons=['mapping','navigation','idle'].map(mode=>element({mode}));
const viewButtons=['live','map'].map(view=>element({view}));
const posts=[],goalPosts=[],planPosts=[];
const state={mode:'idle',switching:false,error:'',preview:true,mapping_only:true,
  mapping_control:{can_start:true,can_stop:false},cloud_age:.1,odom_age:null,pose:null,
  mapping_filter:{enabled:true,height_m:2.5,dynamic_enabled:true,locked:false},
  spin:{enabled:false,rate_rad_s:.5,max_rate_rad_s:.5},yaw:{ready:false,reason:'no feedback'},navigation:{ready:false,status:'idle',reason:'当前仅启用雷达建图；地图定位、路径规划和底盘自动控制尚未接通，暂不能前往目标'}};
const context={URLSearchParams,AbortController,Date,Math,Number,console,
  location:{search:'?token=phone-token'},window:{devicePixelRatio:1},
  sessionStorage:{getItem(){throw Error('blocked storage')},setItem(){throw Error('blocked storage')}},
  document:{getElementById(id){return elements[id]??=(element())},querySelectorAll(selector){return selector==='[data-mode]'?modeButtons:viewButtons}},
  ResizeObserver:class{observe(){}},requestAnimationFrame(){},setInterval(){},setTimeout(){return 1},clearTimeout(){},
  async fetch(path,options){
    assert.equal(options.headers.Authorization,'Bearer phone-token');
    let result=path==='/api/state'?state:path==='/api/cloud'?{sequence:1,points:[[0,0,0]],frame:'livox_frame'}:{available:false};
    if(path==='/api/mode'){posts.push(JSON.parse(options.body));result=state}
    if(path==='/api/navigation/goal'){goalPosts.push(JSON.parse(options.body));result=state}
    if(path==='/api/navigation/plan'){
      const goal=JSON.parse(options.body);planPosts.push(goal);
      state.navigation.plan={status:'ready',message:'路径已计算',goal,path:[[0,0],[goal.x,goal.y]]};result=state;
    }
    return {ok:true,json:async()=>result};
  }};
vm.createContext(context);vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
(async()=>{
  await new Promise(r=>setImmediate(r));
  assert.equal(modeButtons[0].disabled,false,'start must work without AbortSignal.timeout or session storage');
  assert.equal(modeButtons[0].textContent,'开始建图');
  assert.equal(posts.length,0,'opening the page must not start mapping or navigation');
  await modeButtons[1].onclick();
  assert.equal(posts.length,0,'unavailable navigation must not issue a mode change');
  assert.equal(elements.error.textContent,state.navigation.reason,'show why navigation is unavailable');
  assert.equal(elements.navStatus.textContent,state.navigation.reason);
  assert.equal(elements.goGoal.disabled,true);
  await modeButtons[0].onclick();assert.deepEqual(posts,[{mode:'mapping'}]);
  state.navigation_available=true;
  await modeButtons[1].onclick();assert.deepEqual(posts,[{mode:'mapping'},{mode:'navigation'}]);
  state.preview=false;state.mode='navigation';state.navigation.ready=true;state.navigation.reason='';
  vm.runInContext("mapImage={width:100,height:100};mapInfo={origin:[0,0,0],resolution:0.1};mapRevision='saved-map';selectedGoal={x:0.5,y:0.25,map_revision:'saved-map'};navigationControls()",context);
  assert.equal(elements.goGoal.disabled,true,'wait for a real plan before allowing departure');
  vm.runInContext('previewGoal()',context);await new Promise(r=>setImmediate(r));
  assert.deepEqual(planPosts,[{x:.5,y:.25,map_revision:'saved-map'}]);
  assert.equal(goalPosts.length,0,'planning must never start driving');
  assert.equal(elements.goGoal.disabled,false,'successful preview enables departure');
  const previous=lineSegments;vm.runInContext('drawNavigationOverlay()',context);
  assert(lineSegments>previous,'draw the actual calculated path');
  state.navigation.pose={x:.1,y:.1,yaw:0};state.navigation.localized=false;
  vm.runInContext("activeView='map';drawMap()",context);
  assert(fills.includes('#f3aa52'),'uncertain localization uses an orange arrow');
  state.navigation.localized=true;vm.runInContext('drawMap()',context);
  assert(fills.includes('#64d4a0'),'confirmed localization uses a green arrow');
  elements.goGoal.onclick();await new Promise(r=>setImmediate(r));
  assert.deepEqual(goalPosts,[{x:.5,y:.25,map_revision:'saved-map'}]);
  state.navigation.plan={...state.navigation.plan,status:'failed',message:'无可通行路径',path:[]};
  vm.runInContext('navigationControls()',context);
  assert.equal(elements.goGoal.disabled,true);assert(elements.navStatus.textContent.includes('无可通行路径'));
  console.log('Mobile mapping UI: blocked storage, no AbortSignal.timeout, authenticated polling and start click passed');
})().catch(e=>{console.error(e);process.exitCode=1});
