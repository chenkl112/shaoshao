const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(require('path').join(__dirname,'../index.html'),'utf8');
const modeButtons=['mapping','navigation','idle'].map(mode=>({dataset:{mode},classList:{toggle(){}}}));
const elements=Object.fromEntries(['stopMapping','spin','spinRate','error'].map(id=>[id,{classList:{toggle(){}},setAttribute(){}}]));
const context={lastState:null,connected:true,requesting:false,mappingFilterControls(){},$:id=>elements[id],
 document:{querySelectorAll(){return modeButtons}}};
vm.createContext(context);
vm.runInContext(html.slice(html.indexOf('function buttons(){'),html.indexOf("document.querySelectorAll('[data-mode]').forEach(button=>")),context);
function state(mode,control){context.lastState={preview:true,mapping_only:true,mode,mapping_control:control,switching:!!(control.starting||control.stopping)};context.buttons()}
state('idle',{can_start:true});assert.equal(modeButtons[0].disabled,false);assert.equal(modeButtons[0].textContent,'开始建图');assert.equal(elements.stopMapping.disabled,true);
state('idle',{starting:true,can_start:false});assert.equal(modeButtons[0].disabled,true);assert.equal(elements.stopMapping.textContent,'正在启动建图…');
state('mapping',{can_start:false,can_stop:true});assert.equal(modeButtons[0].disabled,true);assert.equal(elements.stopMapping.disabled,false);assert.equal(modeButtons[2].disabled,false);
state('mapping',{stopping:true});assert.equal(elements.stopMapping.disabled,true);assert.equal(modeButtons[2].disabled,true);
state('idle',{can_start:true,saved:true});assert.equal(modeButtons[0].disabled,false);
console.log('Mapping UI: idle start, starting lock, stop, stopping lock, and restart passed');
