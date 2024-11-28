/*
	 *
 * AMDPCNet32II.m -- driver for AMDPCNet32-II Network Device
 *
 * Created by Austin Wells 11/01/21
 */

#import <driverkit/i386/directDevice.h>
#import <driverkit/i386/IOPCIDeviceDescription.h>
#import <driverkit/i386/IOPCIDirectDevice.h>
#import <driverkit/i386/PCI.h>
#import <driverkit/i386/ioPorts.h>
#import <driverkit/generalFuncs.h>
#import <driverkit/kernelDriver.h>
#import <driverkit/align.h>
#import <kernserv/prototypes.h>
#import <kernserv/i386/spl.h>
#import <driverkit/interruptMsg.h>
#import <net/netbuf.h>

#import "AMDPCNet32II.h"
#import "PCNetIO.h"

/** 
    returns YES if this is owned by the driver
    returns NO if this is a free DE we can use
**/
BOOL driverOwn(char *RingBuffer, int idx) {
 
        IOLog("DE_SIZE:%d, idx: %d, total: %x\n", DE_SIZE, idx, RingBuffer + DE_SIZE*idx + 7);
	IOLog("owned? %d\n", RingBuffer[DE_SIZE * idx] & 0x80);

	if (RingBuffer[(DE_SIZE * idx) + 7] & 0x80) {
		return NO;
	} 
	else {
		return YES;
	}
	return YES;
}


@implementation AMDPCNet32II

+ (BOOL)probe:deviceDescription
{
    AMDPCNet32II *driver = [self alloc];
    
    IOLog("AMDPCNet32II Driver: %p\n", driver);
    IOLog("By Austin Wells\n");

    return [driver initFromDeviceDescription:deviceDescription] != nil;
}

- (void)transmit:(netbuf_t)pkt {

   int i = 0;
   unsigned int *s;
   char t;
   unsigned char *map;
   unsigned int pkt_size = nb_size(pkt);

   IOLog("transmit\n");

   map = kalloc(pkt_size);
   bcopy(nb_map(pkt), map, pkt_size);
   
   if (driverOwn(txDescriptorVirtualAddr, tx_idx)) {
	IOLog("Yes\n");
        bcopy(map, &(tx_buffers[1520*tx_idx]), pkt_size);
	*(short*)(&txDescriptorVirtualAddr[(DE_SIZE * tx_idx) + 4]) = (((-pkt_size) & 0x0FFF) | 0xF000);
   	// This line seems problematic 
        txDescriptorVirtualAddr[(DE_SIZE * tx_idx) + 7] |= 0x02;
	txDescriptorVirtualAddr[(DE_SIZE * tx_idx) + 7] |= 0x01;
	txDescriptorVirtualAddr[(DE_SIZE * tx_idx) + 7] |= 0x80;
   	tx_idx += 1;
	tx_idx = tx_idx % NUM_TX_DES;
   }
   else {
	IOLog("No\n");
	tx_idx += 1;
	tx_idx = tx_idx % NUM_TX_DES;
   }

   nb_free(pkt);
}

- (void)writeRAP16:(int)val {
  outw(ioBase+RAP16, val);
  inw(ioBase+RAP16);  
}

- (void)writeRAP32:(long)val {
  outl(ioBase+RAP32, val);  
  inl(ioBase+RAP32);
}

- (int)readCSR16:(int)csr_no { 

  [self writeRAP16: csr_no];
  return inw(ioBase + RDP16);

}

- (long)readCSR32:(long)csr_no { 

  [self writeRAP32: csr_no];
  return inl(ioBase + RDP32);

}

- (void)writeCSR16:(int)csr_no :(int)val { 

  [self writeRAP16: csr_no];
  return outw(ioBase + RDP16, val);

}

- (void)writeCSR32:(long)csr_no :(long)val { 

  [self writeRAP32: csr_no];
  return outl(ioBase + RDP32, val);

}

- (int)readBCR16:(int)bsr_no { 

  [self writeRAP16: bsr_no];
  return inw(ioBase + BDP16);

}

- (long)readBCR32:(long)bsr_no { 

  [self writeRAP32: bsr_no];
  return inl(ioBase + BDP32);

}

- (void)writeBCR16:(int)bsr_no :(int)val { 

  [self writeRAP16: bsr_no];
  return outw(ioBase + BDP16, val);

}

- (void)writeBCR32:(long)bsr_no :(long)val { 

  [self writeRAP32: bsr_no];
  return outl(ioBase + BDP32, val);

}

- (void)interruptOccurred {
  
  int csr0;  
  
  csr0 = [self readCSR32: 0];
  //csr0 &= ~0x04;
  [self writeCSR32: 0 : csr0];
  
  csr0 = [self readCSR32: 5];
  //csr0 &= ~0x04;
  [self writeCSR32: 5 : csr0];

  /**[self writeCSR32: 0 : 0x7940];
  [self writeCSR32: 0 : 0x0048];
  [self writeCSR32: 0 : 0x4];
  [self writeCSR32: 0 : 0x2];
  [self writeCSR32: 0 : 0x4];
  [self writeCSR32: 0 : 0x2];**/
  [self enableAllInterrupts];
  IOLog("got an interrupt?\n");
  [self writeCSR32: 0 : 0x02];
}


- (void)timeoutOccurred {

}

- initFromDeviceDescription:deviceDescription
{
  
    IOPCIConfigSpace config;
    IOPCIDeviceDescription *devDescription = (IOPCIDeviceDescription *)deviceDescription;	 

    ns_time_t start, end; 
    unsigned long conf;

    unsigned long csr58;
    unsigned long bcr2;

    //vm_address_t registerSetup;
    volatile unsigned int *registerSetup;
    volatile unsigned char *registerSetupBytes;
    volatile unsigned long *registerSetupPhysical;
    volatile unsigned int *s;

    unsigned int i = 0;
    unsigned long csr0;

    volatile AMDPCNetRegisters initRegisters;
   
    unsigned int a;
    unsigned int b;

    IOReturn rc;
    isRunningNow = NO;

    tx_idx = 0;
	
    if([IODirectDevice getPCIConfigSpace: &config withDeviceDescription: devDescription]) {
      IOLog("Can't access config space, exiting\n");
      //[self free];
      return nil;
    } 

    IOLog("got PCI Config Space\n");
 
    // Need to enable bus mastering on the card
    if([IODirectDevice getPCIConfigData: &conf atRegister: 0x04 withDeviceDescription: devDescription] != IO_R_SUCCESS) {
      IOLog("Can't get config space at register 1, exiting\n");
      //[self free];
      return nil;
    }

    IOLog("PCI Config: %lx\n", conf);

    //conf &= 0xFFFF0000; // 		
    //conf |= 0x5; // set bus mastering bit
    conf |= 0x05; 

    IOLog("PCI Config: %lx\n", conf);

    if([IODirectDevice setPCIConfigData: conf atRegister: 0x04 withDeviceDescription: devDescription] != IO_R_SUCCESS) {
      IOLog("Can't set config space, exiting\n");
      //[self free];
      return nil;
    } 

    IOLog("Bus Mastering Enabled\n");

    // At this point bus mastering should have been enabled


    // figure out the base address for I/O ports and the IRQ number
    ioBase = config.BaseAddress[0] & 0xfffffffe;
    irq = (int)config.InterruptLine; 

    IOLog("AMDPCNet32II\n");
    IOLog("Vendor:%x\n", config.VendorID);
    IOLog("Device:%x\n", config.DeviceID);
    IOLog("Base:%x\n", ioBase);
    IOLog("Irq num: %x\n", irq);
    IOLog("Mac Addr: ");

    for(i = 0; i < 6; i++) {
    	mac[i] = inb(ioBase + i);
	    IOLog("%02x ", mac[i]);
    }

    IOLog("\n");   

    port.start = (IOEISAPortAddress) ioBase;
    port.size  = 0x20;	

    [devDescription setInterruptList: &irq  num: 1]; 
    [devDescription setPortRangeList: &port num: 1];

    if ([super initFromDeviceDescription:deviceDescription] == nil) {
        IOLog("super init failed\n");
    	return nil;
    }

    // Do a reset so we know the card is in 16bit mode
    IOLog("Resetting card ...\n");

    inl(ioBase + 0x18); //if 32bit
    inw(ioBase + 0x14); //if 16bit

    // Wait at least 1 usec for the card to come back
    IOLog("waiting for card to come back ...\n");    

    IOGetTimestamp(&start);
    while(1) {
       IOGetTimestamp(&end);
       IOLog(".");
       if((end - start) > 10000)
           break;
    }

    IOLog("we're back baby\n");

    // We now must be in 16-bit mode. 
    // Put the card in 32-bit mode
    outl(ioBase + 0x10, 0);
    
    // Set SWStyle to 2
    csr58 = [self readCSR32: 58];
    csr58 &= 0xFF00;
    csr58 |= 0x2;
    [self writeCSR32: 58 : csr58];
    
    // Set autoselect to use either 10/100 baseT or coaxial outputs 
    bcr2 = [self readBCR32: 2];
    bcr2 |= 0x2;
    [self writeBCR32: 2 : bcr2]; 

    // allocate 64 bytes so we can get it 32 byte aligned
    s = kalloc(64);
    registerSetup = IOAlign(void*, s, 32); //kmem_alloc_wired(IOVmtaskSelf(), s, 32);
    bzero(s, 64); // zero memory and ensure it has been accessed
    IOLog("vaddr: %x:%x\n", s, registerSetup);	 
  
    rc = IOPhysicalFromVirtual(IOVmTaskSelf(), registerSetup, registerSetupPhysical);
    if(rc == IO_R_INVALID_ARG) {
      IOLog("FAILED TO GET PHYSICAL ADDRESS\n");
      return nil;
    }
    else 
      IOLog("phys: %x\n", *registerSetupPhysical);

    // This section is a mess
    // TODO: get packed working so we can use the struct directly

    // Use hardcoded values for now
    initRegisters.TLEN = NUM_TX_DES_LOG2;
    initRegisters.RLEN = NUM_RX_DES_LOG2;
    initRegisters.MODE = 0;
    for(i = 0; i < 6; i++) {
      // TODO: support setting the mac address
    	initRegisters.MAC[i] = mac[i];
    }
    for(i = 0; i < 8; i++) {
      // TODO: support logical address filtering
    	initRegisters.LADR[i] = 0x00;
    }
    
    // allocate the ring buffers
    // zero out the ring buffers
    rxDescriptorVirtualAddr = kalloc(32 * DE_SIZE * NUM_RX_DES);
    bzero(rxDescriptorVirtualAddr, 32 * DE_SIZE * NUM_RX_DES);

    txDescriptorVirtualAddr = kalloc(16 * DE_SIZE * NUM_TX_DES);
    IOLog("init txDescriptorVirtualAddr: %x\n", txDescriptorVirtualAddr);
    IOLog("init txDescriptorVirtualAddr: %x\n", txDescriptorVirtualAddr + 16 * DE_SIZE * (NUM_TX_DES - 1));
    bzero(txDescriptorVirtualAddr, 16 * DE_SIZE * NUM_TX_DES);

    for(i = 0; i < NUM_TX_DES; i++) {
       driverOwn(txDescriptorVirtualAddr, i);
    }

    IOLog("rx virtual %x\n", rxDescriptorVirtualAddr);
    IOLog("tx virtual %x\n", txDescriptorVirtualAddr);	

    // the card needs the physical address of the ring buffers
    rc = IOPhysicalFromVirtual(IOVmTaskSelf(), rxDescriptorVirtualAddr, rxDescriptorPhysicalAddr);
    a = *rxDescriptorPhysicalAddr;
    IOLog("rx physical %x\n", *rxDescriptorPhysicalAddr);

    IOLog("----------------\n"); 

    rc = IOPhysicalFromVirtual(IOVmTaskSelf(), txDescriptorVirtualAddr, txDescriptorPhysicalAddr);
    b = *txDescriptorPhysicalAddr;
    IOLog("tx physical %x\n", *txDescriptorPhysicalAddr);

    IOLog("a,b = %x,%x\n", a, b);

    initRegisters.rxDescriptorAddr = *rxDescriptorPhysicalAddr;
    initRegisters.txDescriptorAddr = *txDescriptorPhysicalAddr;

    // Init the DEs
    rx_buffers = kalloc(NUM_RX_DES * 1520);
    tx_buffers = kalloc(NUM_TX_DES * 1520);

    IOLog("rx_buffers: %x\n", rx_buffers);    
    IOLog("tx_buffers: %x\n", tx_buffers);    

    for(i = 0; i < NUM_RX_DES; i++) {
	IOPhysicalFromVirtual(IOVmTaskSelf(), &(rx_buffers[1520*i]), s);
	IOLog("s: %x\n", *s);
	IOLog("rx_buffer: %x\n", &(rx_buffers[1520*i]));
	
	*(int*)(&rxDescriptorVirtualAddr[DE_SIZE * i]) = *s;
	*(short*)(&rxDescriptorVirtualAddr[DE_SIZE * i + 4]) = (((-1520) & 0x0fff) | 0xf000);
        rxDescriptorVirtualAddr[DE_SIZE * i + 7] = 0x80;

	if(driverOwn(rxDescriptorVirtualAddr, i)) {
		IOLog("YES\n");
	}
	else {
		IOLog("NO\n");
	}

    }
    
    for(i = 0; i < NUM_TX_DES; i++) {
	IOPhysicalFromVirtual(IOVmTaskSelf(), &(tx_buffers[1520*i]), s);
	IOLog("s: %x\n", *s);
	IOLog("tx_buffer: %x\n", &(tx_buffers[1520*i]));
	
	*(int*)(&txDescriptorVirtualAddr[DE_SIZE * i]) = *s;
	*(short*)(&txDescriptorVirtualAddr[DE_SIZE * i + 4]) = (((-1520) & 0x0fff) | 0xf000);

	if(driverOwn(txDescriptorVirtualAddr, i)) {
		IOLog("YES\n");
	}
	else {
		IOLog("NO\n");
	}
    }

    registerSetupBytes = (unsigned char *)registerSetup;

    // Set up registerSetup byte by byte
    registerSetupBytes[0] = (char)(initRegisters.MODE & 0xFF);
    registerSetupBytes[1] = (char)((initRegisters.MODE >> 8) & 0xFF);

    registerSetupBytes[2] = 0x30;
    registerSetupBytes[3] = 0x30;
    
    for (i = 0; i < sizeof(initRegisters.MAC); i++) {
        registerSetupBytes[4 + i] = initRegisters.MAC[i];
    }
    
    registerSetupBytes[10] = initRegisters.reserved;
    registerSetupBytes[11] = initRegisters.reserved;
    
    for (i = 0; i < sizeof(initRegisters.LADR); i++) {
        registerSetupBytes[12 + i] = initRegisters.LADR[i];
    }

    *(unsigned int *)(registerSetupBytes + 20) = a;//*rxDescriptorPhysicalAddr; //initRegisters.rxDescriptorAddr;
    *(unsigned int *)(registerSetupBytes + 24) = b;//*txDescriptorPhysicalAddr; //initRegisters.txDescriptorAddr;
    for (i = 0; i < sizeof(initRegisters.PAD); i++) {
        registerSetupBytes[28 + i] = 0;
    }

    IOLog("a=%x", a);
    IOLog("b=%x", b);
    for(i = 0; i < 28; i++) {
	IOLog("register[%d] = %x\n", i , registerSetupBytes[i]);
    }

    // write s to the card
    // To actually set up the card registers, we provide it with the address of our initialization structure by writing the low 16-bits of its address to CSR1 and the high 16-bits to CSR2
    [self writeCSR32: 1 : (unsigned long)*registerSetupPhysical & 0xFFFF];
    [self writeCSR32: 2 : (unsigned long)*registerSetupPhysical >> 16];

    // set bits 10,9,8,2 of CSR3 to 1
    // bit 10 - recieve interrupts mask
    // bit 9  - transmit interrupts mask
    // bit 8  - interrupt done mask (0 = unmask, 1 = mask)
    // bit 2  - endianess (0 = little, 1 = big)
    //[self writeCSR32: 3 : (unsigned long) ((1 << 10) | (1 << 9) | (1 << 8))];
    [self writeCSR32: 3 : (unsigned long) (~(1 << 10)) & (~(1 << 2))];

    // set all bits of CSR4 to 1, autopad
    [self writeCSR32: 4 : (unsigned long) (0x915)];
    //[self writeCSR32: 4 : (unsigned long) (1 << 11)];
    // set the finish bit of CSR0 to 1
    //[self readCSR32: 0];

    //[self writeCSR32: 0 : 1 | ((1 << 2))];
    //[self writeCSR32: 4 : 0x1];

    [self writeCSR32: 0 : 0x4];
    [self writeCSR32: 0 : 0x1];

    // poll CSR0 bit 8 to be set to 1
    IOGetTimestamp(&start);
    while(1) {
      IOGetTimestamp(&end);
      csr0 = [self readCSR32: 0];
      if(csr0 & (1 << 8)) {
        IOLog("csr0:%ld\n", csr0); 
	      break;
      }
       if((end - start) > 100000) {
          IOLog("didn't come back\n");
          break;
       }
    }

    IOLog("card is start, pray for me\n");
     
    // TODO Cleanup so we don't have 2 copies
    for(i = 0; i < 6; i++) {
	real_mac.ether_addr_octet[i] = mac[i];
    	IOLog("mac[%d] = %x\n", i, real_mac.ether_addr_octet[i]);
     }			
    bcopy(mac, &real_mac, 6);
    network = [super attachToNetworkWithAddress:real_mac];

    // this might be redundant
    isRunningNow = YES;

    // clear the init bit, stop bit and start bit of CSR0
    //[self writeCSR32: 0 : ((1 << 6) | (1 << 1))];
    //[self readCSR32: 0];
    [self writeCSR32: 0 : 0x142];

    [self resetAndEnable: NO];
    IOLog("pray even harder\n");

    return self;
}

- (BOOL)resetAndEnable:(BOOL)enable {

  if(enable) {
	IOLog("reset and enable: Enable\n");	
  }
  else {
	IOLog("reset and enable: Disable\n");	
  }

  [self disableAllInterrupts];
  
  if (enable && [self enableAllInterrupts] != IO_R_SUCCESS) {
	IOLog("failed to renable interrupts\n");
	[self setRunning:NO];
	return NO;
  }

  [self setRunning:enable];
  return YES;

}

- (IOReturn)enableAllInterrupts {
  	return [super enableAllInterrupts];
}

- (void)disableAllInterrupts {
	[super disableAllInterrupts];
}

- free
{
    return [super free];
}

@end
