#include "kern_xe.hpp"
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOMemoryDescriptor.h>

static void publishBytes(IOPCIDevice *pci,const char *key,const void *bytes,unsigned length) {
    auto data=OSData::withBytes(bytes,length);
    if(data){pci->setProperty(key,data);data->release();}
}

// Firmware is read once, before topology/hook selection. No PCI BAR/ROM scan,
// power acquisition, or MMIO probing occurs here. Explicit properties win later.
void XE::xelpReadFirmware(IOPCIDevice *pci) {
    // Wiring is needed by framebuffer-only profiles before Render is admitted.
    if(!pci || !xeProfile.displayAdmitted())return;
    auto catalog=XELP::displayAdapter(xeProfile.family)==XELP::DisplayAdapter::TGL && tglEDPSignalPolicy ? &tglSignalPanels : nullptr;
    const uint32_t asls=pci->configRead32(0xFC);
    uint32_t source=0;
    if(asls && asls<=0xFFFFE000U) {
        auto memory=IOMemoryDescriptor::withPhysicalAddress(asls,8192,kIODirectionIn);
        auto mapping=memory ? memory->map(kIOMapReadOnly) : nullptr;
        if(mapping && mapping->getVirtualAddress() && mapping->getLength()>=8192) {
            const auto bytes=reinterpret_cast<const uint8_t *>(mapping->getVirtualAddress());
            const auto region=XELP::VBT::region(bytes,8192,asls);
            if(region.valid) {
                // Preserve existing diagnostics for collectors.
                pci->setProperty("TGLASLS",asls,32);
                publishBytes(pci,"TGLOpRegion",bytes,8192);
                if(region.address) {
                    auto vbtMemory=IOMemoryDescriptor::withPhysicalAddress(region.address,region.bytes,kIODirectionIn);
                    auto vbtMap=vbtMemory ? vbtMemory->map(kIOMapReadOnly) : nullptr;
                    if(vbtMap && vbtMap->getVirtualAddress() && vbtMap->getLength()>=region.bytes) {
                        const auto data=reinterpret_cast<const void *>(vbtMap->getVirtualAddress());
                        firmwarePorts=XELP::VBT::parse(data,region.bytes,xeProfile.family,catalog);
                        if(firmwarePorts.valid()){source=2;publishBytes(pci,"TGLVBT",data,firmwarePorts.bytes);}
                    }
                    if(vbtMap)vbtMap->release();
                    if(vbtMemory)vbtMemory->release();
                }
                if(!firmwarePorts.valid()) {
                    const auto embedded=XELP::VBT::parse(bytes+0x400,region.inlineBytes,xeProfile.family,catalog);
                    if(embedded.valid()) {
                        firmwarePorts=embedded;source=1;
                        publishBytes(pci,"TGLVBT",bytes+0x400,firmwarePorts.bytes);
                    } else if(firmwarePorts.status==XELP::VBT::Status::Missing)firmwarePorts=embedded;
                }
            }
        }
        if(mapping)mapping->release();
        if(memory)memory->release();
    }
    const auto hdmi=firmwarePorts.hdmi(3); // physical Combo A/B on both admitted families
    uint32_t summary[]={1,static_cast<uint32_t>(firmwarePorts.status),source,firmwarePorts.version,
        firmwarePorts.present,firmwarePorts.unmapped,firmwarePorts.ambiguous,hdmi.valid,hdmi.hdmi,
        hdmi.ddc[0],hdmi.ddc[1],hdmi.ddc[2]};
    pci->setProperty("WhateverXeFirmwarePorts",summary,sizeof(summary));
    pci->setProperty("WhateverXeFirmwareChecksum",firmwarePorts.checksum,32);
    pci->setProperty("WhateverXeFirmwarePortRecords",firmwarePorts.port,sizeof(firmwarePorts.port));
    const auto &panel=firmwarePorts.panel;
    uint32_t panelFacts[]={panel.valid,panel.type,panel.ddi,panel.lowSwing,panel.hobl};
    pci->setProperty("WhateverXeFirmwarePanel",panelFacts,sizeof(panelFacts));
    pci->setProperty("WhateverXeEDPSignalPolicy",tglEDPSignalPolicy,32);
    if(catalog) {
        uint32_t selection[]={1,catalog->requested,catalog->ready,
            static_cast<uint32_t>(catalog->ready?XELP::VBT::PanelMatch::WaitingEDID:XELP::VBT::PanelMatch::Unavailable)};
        pci->setProperty("WhateverXeEDPPanelCatalog",selection,sizeof(selection));
    }
}

// No new AUX/MMIO or OpRegion commands. Use bytes already returned by the
// exact donor EDID callbacks. In particular, mailbox-5's cached EDID may name
// a different firmware panel and cannot replace a live internal-port read.
void XE::resolveTGLSignalPanel(void *port,const uint8_t *edid,uint32_t length) {
    if(!tglActive || !tglEDPSignalPolicy ||
       XELP::displayAdapter(xeProfile.family)!=XELP::DisplayAdapter::TGL ||
       !tglSignalPanels.ready || firmwarePorts.panel.valid ||
       __atomic_load_n(&tglSignalPanel,__ATOMIC_ACQUIRE))return;
    auto p=static_cast<uint8_t *>(port);
    auto c=static_cast<uint8_t *>(tglObservationController);
    if(!p || !c)return;
    auto cfg=*reinterpret_cast<uint32_t **>(p+0x548);
    if(!cfg || cfg[0]>1 || cfg[2]!=0 || cfg[3]!=0 || p[0x117] ||
       *reinterpret_cast<uint32_t *>(p+0x550)!=1 ||
       *reinterpret_cast<void **>(p+0x578)!=*reinterpret_cast<void **>(c+0xC40) ||
       !firmwarePorts.valid() || !(firmwarePorts.present&(1U<<cfg[0])) ||
       !firmwarePorts.port[cfg[0]].internal() || !firmwarePorts.port[cfg[0]].dp() ||
       (firmwarePorts.port[cfg[0]].flags&0x17))return;
    XELP::VBT::PanelMatch status;
    const auto selected=tglSignalPanels.match(edid,length,status);
    uint32_t selection[]={1,tglSignalPanels.requested,static_cast<uint32_t>(status),
        selected?selected->type:UINT32_MAX,cfg[0]};
    auto root=IORegistryEntry::getRegistryRoot();
    root->setProperty("WhateverXeEDPPanelMatch",selection,sizeof(selection));
    if(!selected || selected->ddi!=cfg[0])return;
    const XELP::VBT::Ports::Panel *expected=nullptr;
    if(!__atomic_compare_exchange_n(&tglSignalPanel,&expected,selected,false,__ATOMIC_RELEASE,__ATOMIC_RELAXED))return;
    uint32_t facts[]={selected->valid,selected->type,selected->ddi,selected->lowSwing,selected->hobl};
    root->setProperty("WhateverXeEDPResolvedPanel",facts,sizeof(facts));
}
