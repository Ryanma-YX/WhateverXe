#!/usr/bin/env python3
"""Execute live-EDID publication and the actual native signal return boundary."""
from pathlib import Path
import subprocess,tempfile

root=Path(__file__).resolve().parents[1]
firmware=(root/'WhateverXe/kern_firmware.cpp').read_text()
resolve=firmware[firmware.index('void XE::resolveTGLSignalPanel('):]
source=(root/'WhateverXe/kern_xe.cpp').read_text()
start=source.index('uint32_t XE::wrapTGLSignalLevels(')
end=source.index('\n\tauto bytes = static_cast<uint8_t *>(port);',start)
native=source[start:end]+'\n return fallback();\n}\n'
code=r'''
#include "kern_vbt.hpp"
#include <cassert>
#include <cstdio>
#include <map>
#include <string>
#include <vector>
using mach_vm_address_t=uintptr_t;
struct Root {
 std::map<std::string,std::vector<uint32_t>> properties;
 void setProperty(const char *name,void *data,size_t bytes) {
  const auto p=static_cast<const uint32_t *>(data);properties[name]={p,p+bytes/4};
 }
} registry;
struct IORegistryEntry {static Root *getRegistryRoot(){return &registry;}};
constexpr uint32_t TGLEDPSignalNative=1;
bool tglDisplayTraceEnabled=true,chooseNative=true;
unsigned nativeCalls=0,fallbackCalls=0,records=0;
uint32_t nativeResult=0;
void clock_get_uptime(uint64_t *value){*value=0x123456789ULL;}
void absolutetime_to_nanoseconds(uint64_t value,uint64_t *ns){*ns=value;}
bool tglProgramComboSignal(void *,uint32_t,uint32_t,void *,uint32_t &,bool &native){native=chooseNative;return false;}
uint32_t fallback(){++fallbackCalls;return 0x12345678;}
void tglTransitionRecord(unsigned stage,bool done,void *,uint64_t ddi,uint64_t adjust,uint64_t result,const void *payload,size_t bytes) {
 assert(stage==88&&done&&ddi==0&&adjust==(2|(1<<8))&&result==nativeResult&&bytes==48&&bytes<=56);
 const auto p=static_cast<const uint32_t *>(payload);assert(p[0]==1&&p[4]==20&&p[5]==4&&p[11]==(result==0));++records;
}
struct XE {
 bool tglActive=true;uint32_t tglEDPSignalPolicy=1;
 struct {XELP::Family family=XELP::Family::TigerLake;} xeProfile;
 XELP::VBT::Ports firmwarePorts;
 XELP::VBT::PanelCatalog tglSignalPanels;
 const XELP::VBT::Ports::Panel *tglSignalPanel=nullptr;
 void *tglObservationController=nullptr;mach_vm_address_t orgTGLSignalLevels=0;
 void resolveTGLSignalPanel(void *,const uint8_t *,uint32_t);
 static uint32_t wrapTGLSignalLevels(void *,uint32_t,uint32_t,void *);
} state,*callbackXE=&state;
#define FunctionCast(f,p) reinterpret_cast<decltype(&f)>(p)
RESOLVE
NATIVE
uint32_t donor(void *,uint32_t swing,uint32_t emphasis,void *){assert(swing==2&&emphasis==1);++nativeCalls;return nativeResult;}
template<class T>void put(uint8_t *p,unsigned off,T value){memcpy(p+off,&value,sizeof(value));}
int main(){
 for(unsigned bad=0;bad<18;++bad) {
  state={};registry.properties.clear();
  alignas(8) uint8_t port[0x600]{},controller[0xd00]{},edid[128]{};uint32_t cfg[4]{};
  const uint8_t header[]={0,255,255,255,255,255,255,0};memcpy(edid,header,8);edid[8]=0x12;edid[9]=0x34;
  uint8_t sum=0;for(auto b:edid)sum+=b;edid[127]-=sum;
  state.tglSignalPanels.ready=true;state.tglSignalPanels.requested=255;
  memcpy(state.tglSignalPanels.id[3],edid+8,10);state.tglSignalPanels.panel[3].valid=true;
  state.tglSignalPanels.panel[3].ddi=0;state.tglSignalPanels.panel[3].type=3;
  state.firmwarePorts.status=XELP::VBT::Status::Valid;state.firmwarePorts.present=1;
  state.firmwarePorts.port[0].type=0x78c6;state.tglObservationController=controller;
  put(port,0x548,cfg+0);put(port,0x550,1U);put(port,0x578,controller+0);put(controller,0xc40,controller+0);
  if(bad==1)state.tglActive=false;
  if(bad==2)state.tglEDPSignalPolicy=0;
  if(bad==3)state.xeProfile.family=XELP::Family::AlderLakeN;
  if(bad==4)cfg[0]=3;
  if(bad==5)cfg[2]=1;
  if(bad==6)cfg[3]=1;
  if(bad==7)port[0x117]=1;
  if(bad==8)put(port,0x578,port+0);
  if(bad==9)state.firmwarePorts.status=XELP::VBT::Status::Missing;
  if(bad==10)state.firmwarePorts.port[0].type=0x60d6;
  if(bad==11)state.firmwarePorts.port[0].flags=4;
  if(bad==12)state.tglSignalPanels.panel[3].ddi=1;
  if(bad==13)edid[127]^=1;
  if(bad==14)state.tglSignalPanels.ready=false;
  if(bad==15)put(port,0x550,3U);
  if(bad==16)state.firmwarePorts.present=0;
  if(bad==17)state.firmwarePorts.panel.valid=true;
  state.resolveTGLSignalPanel(port,edid,128);
  assert(bool(state.tglSignalPanel)==(bad==0));
  assert(registry.properties.count("WhateverXeEDPResolvedPanel")==unsigned(bad==0));
  if(!bad) {
   assert(!state.firmwarePorts.panel.valid&&state.tglSignalPanel->type==3);
   const auto saved=state.tglSignalPanel;edid[127]^=1;state.resolveTGLSignalPanel(port,edid,128);
   assert(state.tglSignalPanel==saved); // immutable once published
  }
 }
 // Enter the actual native callback, preserve its exact success/failure
 // result, and publish evidence only after the donor has been called.
 state={};state.orgTGLSignalLevels=reinterpret_cast<uintptr_t>(donor);
 alignas(8) uint8_t port[0x600]{};uint32_t cfg[4]{};put(port,0x548,cfg+0);put(port,0x124,4U);put(port,0x128,20U);
 for(auto result:{0U,0xe00002d6U}) {
  nativeResult=result;registry.properties.clear();const auto calls=nativeCalls;
  assert(XE::wrapTGLSignalLevels(port,2,1,nullptr)==result&&nativeCalls==calls+1);
  auto applied=registry.properties.at("WhateverXeEDPSignalApplied");
  assert(applied==std::vector<uint32_t>({1,1,0,20,4,result,nativeCalls,0x23456789U,1}));
 }
 assert(records==2&&!fallbackCalls);
 chooseNative=false;registry.properties.clear();assert(XE::wrapTGLSignalLevels(port,2,1,nullptr)==0x12345678);
 assert(fallbackCalls==1&&!registry.properties.count("WhateverXeEDPSignalApplied")&&records==2);
 puts("PASS live internal-EDID guards, immutable selection, bounded signal trace and native path evidence");
}
'''.replace('RESOLVE',resolve).replace('NATIVE',native)
with tempfile.TemporaryDirectory(prefix='tgl-edp-panel-') as tmp:
    p=Path(tmp);(p/'test.cpp').write_text(code)
    subprocess.run(['clang++','-std=c++17','-O1','-fsanitize=address,undefined','-I'+str(root/'WhateverXe'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
