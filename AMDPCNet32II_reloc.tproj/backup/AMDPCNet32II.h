/*
 *
 * AMDPCNet32II.h -- interface for AMDPCNet32II Ethernet driver.
 *
 * Created by Austin Wells 11/01/21
 */

#ifndef AMDPCNET32II
#define AMDPCNET32II

#import <driverkit/i386/directDevice.h>
#import <driverkit/i386/ioPorts.h>

#import <driverkit/IOEthernet.h>

/** each entry is 16 bytes in SWSTYLE=2 **/
#define DE_SIZE 16

// Must be a power of 2
#define NUM_RX_DES 8
#define NUM_TX_DES 8

// we could do these programmatically, but it's not worth it for now
#define NUM_RX_DES_LOG2 3
#define NUM_TX_DES_LOG2 3

typedef struct _descriptorEntry {
 
  /** physical address of the packet **/
  unsigned int addr;  
  

} descriptorEntry;

typedef struct _AMDPCNetRegisters {
  /**
  unsigned long mode:8;
  unsigned long rlen:4;
  unsigned long reserved1:4;
  unsigned long tlen:4;
  unsigned long reserved2:4;
  unsigned char mac[6];
  unsigned int reserved3;
  unsigned char ladr[8];
  unsigned long rdx;
  unsigned long tdx;
  **/
  
  unsigned char TLEN;
  unsigned char RLEN;
  unsigned int MODE;
  unsigned char MAC[6];
  unsigned char reserved;
  unsigned char LADR[8];
  unsigned long rxDescriptorAddr;
  unsigned long txDescriptorAddr;
  unsigned char PAD[4];  

} AMDPCNetRegisters;




@interface AMDPCNet32II:IOEthernet
{
 IOEISAPortAddress ioBase;
 IORange port;	

 int irq;

 unsigned char mac[6];
 enet_addr_t real_mac;
 
 BOOL PromisciousMode;	
 BOOL isRunningNow;

 char *rxDescriptorVirtualAddr;
 char *txDescriptorVirtualAddr;

 int *rxDescriptorPhysicalAddr;
 int *txDescriptorPhysicalAddr;

 unsigned int tx_idx;
 unsigned int rx_idx;
 unsigned char *rx_buffers;
 unsigned char *tx_buffers;

 IONetwork *network;

}

// IODirectDevice methods
+ (BOOL)probe: deviceDescription;
- initFromDeviceDescription: deviceDescription;
- free;

- (IOReturn)enableAllInterrupts;
- (void)disableAllInterrupts;

- (void)transmit:(netbuf_t)pkt;

- (BOOL)resetAndEnable:(BOOL)enable;
- (void)interruptOccurred;
- (void)timeoutOccurred;

// - (BOOL)enableMulticastMode;
// - (void)disableMulticastMode;

// - (void)addMulticastAddress:(enet_addr_t *) address;
// - (void)removeMulticastAddress:(enet_addr_t *) address;

// - (BOOL)enablePromiscuousMode;
// - (void)disablePromiscuousMode;

//- (BOOL)isRunning;

//- (BOOL)isUnwantedMulticastPacket:(ether_header_t *)header;


// IONetworkDeviceMethods protocol methods
// - (netbuf_t)allocateNetbuf;
// - (int)finishInitialization;
// - (int)outputPacket:(netbuf_t)packet address:(void *)address;
// - (int)performCommand:(const char *)command data:(void *)data;

// AMDPCNet32II methods
- (void)writeRAP16:  (int)val;
- (void)writeRAP32: (long)val;

- (int)readCSR16: (int)csr_no;
- (long)readCSR32: (long)csr_no;

- (void)writeCSR16:  (int)csr_no  :(int)val;
- (void)writeCSR32: (long)csr_no :(long)val;

- (int)readBCR16: (int)bsr_no;
- (long)readBCR32: (long)bsr_no;

- (void)writeBCR16:  (int)bsr_no  :(int)val;
- (void)writeBCR32: (long)bsr_no :(long)val;

@end

#endif /* AMDPCNET32II */
