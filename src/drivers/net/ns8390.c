/**************************************************************************
ETHERBOOT -  BOOTP/TFTP Bootstrap Program

Author: Martin Renters
  Date: May/94

 This code is based heavily on David Greenman's if_ed.c driver

 Copyright (C) 1993-1994, David Greenman, Martin Renters.
  This software may be used, modified, copied, distributed, and sold, in
  both source and binary form provided that the above copyright and these
  terms are retained. Under no circumstances are the authors responsible for
  the proper functioning of this software, nor do the authors assume any
  responsibility for damages incurred with its use.

Multicast support added by Timothy Legge (timlegge@users.sourceforge.net) 09/28/2003
Relocation support added by Ken Yap (ken_yap@users.sourceforge.net) 28/12/02
3c503 support added by Bill Paul (wpaul@ctr.columbia.edu) on 11/15/94
SMC8416 support added by Bill Paul (wpaul@ctr.columbia.edu) on 12/25/94
3c503 PIO support added by Jim Hague (jim.hague@acm.org) on 2/17/98
RX overrun by Klaus Espenlaub (espenlaub@informatik.uni-ulm.de) on 3/10/99
  parts taken from the Linux 8390 driver (by Donald Becker and Paul Gortmaker)
SMC8416 PIO support added by Andrew Bettison (andrewb@zip.com.au) on 4/3/02
  based on the Linux 8390 driver (by Donald Becker and Paul Gortmaker)

**************************************************************************/

#include "etherboot.h"
#include "nic.h"
#include "ns8390.h"
#include "isa.h"
#include "rtl8019conf.h"
#include "rtl8019.h"

static unsigned char	eth_vendor, eth_flags;
static unsigned short	eth_nic_base, eth_asic_base;
static unsigned char	eth_memsize, eth_rx_start, eth_tx_start;
static Address		eth_bmem, eth_rmem;
static unsigned char	eth_drain_receiver;

#define	eth_probe	ne_probe
#define	ASIC_PIO	NE_DATA

#define u08 byte

/**************************************************************************
ETH_PIO_READ - Read a frame via Programmed I/O
**************************************************************************/
static void eth_pio_read(unsigned int src, unsigned char *dst, unsigned int cnt)
{
	// Abort any reads in process
	outb(D8390_COMMAND_RD2 |
		D8390_COMMAND_STA, eth_nic_base + D8390_P0_COMMAND);

    // Set up DMA Byte Count
	outb(cnt, eth_nic_base + D8390_P0_RBCR0);
	outb(cnt>>8, eth_nic_base + D8390_P0_RBCR1);

    // Set up Destination address in memory
	outb(src, eth_nic_base + D8390_P0_RSAR0);
	outb(src>>8, eth_nic_base + D8390_P0_RSAR1);

    // Perform Remote DMA Read
	outb(D8390_COMMAND_RD0 |
		D8390_COMMAND_STA, eth_nic_base + D8390_P0_COMMAND);

    // Copy Back the Data!
	if (eth_flags & FLAG_16BIT)
		cnt = (cnt + 1) >> 1;

	while(cnt--) {
		if (eth_flags & FLAG_16BIT) {
			*((unsigned short *)dst) = inw(eth_asic_base + ASIC_PIO);
			dst += 2;
		}
		else
			*(dst++) = inb(eth_asic_base + ASIC_PIO);
	}
}

/**************************************************************************
ETH_PIO_WRITE - Write a frame via Programmed I/O
**************************************************************************/
static void eth_pio_write(const unsigned char *src, unsigned int dst, unsigned int cnt)
{
#ifdef	COMPEX_RL2000_FIX
	unsigned int x;
#endif	/* COMPEX_RL2000_FIX */
	outb(D8390_COMMAND_RD2 |
		D8390_COMMAND_STA, eth_nic_base + D8390_P0_COMMAND);
	outb(D8390_ISR_RDC, eth_nic_base + D8390_P0_ISR);
	outb(cnt, eth_nic_base + D8390_P0_RBCR0);
	outb(cnt>>8, eth_nic_base + D8390_P0_RBCR1);
	outb(dst, eth_nic_base + D8390_P0_RSAR0);
	outb(dst>>8, eth_nic_base + D8390_P0_RSAR1);
	outb(D8390_COMMAND_RD1 |
		D8390_COMMAND_STA, eth_nic_base + D8390_P0_COMMAND);

	if (eth_flags & FLAG_16BIT)
		cnt = (cnt + 1) >> 1;

	while(cnt--)
	{
		if (eth_flags & FLAG_16BIT) {
			outw(*((unsigned short *)src), eth_asic_base + ASIC_PIO);
			src += 2;
		}
		else
			outb(*(src++), eth_asic_base + ASIC_PIO);
	}

#ifdef	COMPEX_RL2000_FIX
	for (x = 0;
		x < COMPEX_RL2000_TRIES &&
		(inb(eth_nic_base + D8390_P0_ISR) & D8390_ISR_RDC)
		!= D8390_ISR_RDC;
		++x);
	if (x >= COMPEX_RL2000_TRIES)
		printf("Warning: Compex RL2000 aborted wait!\n");
#endif	/* COMPEX_RL2000_FIX */
	while((inb(eth_nic_base + D8390_P0_ISR) & D8390_ISR_RDC)
		!= D8390_ISR_RDC);
}


/**************************************************************************
enable_multycast - Enable Multicast
**************************************************************************/
static void enable_multicast(unsigned short eth_nic_base)
{
	unsigned char mcfilter[8];
	int i;
	memset(mcfilter, 0xFF, 8);
	outb(4, eth_nic_base+D8390_P0_RCR);
	outb(D8390_COMMAND_RD2 + D8390_COMMAND_PS1, eth_nic_base + D8390_P0_COMMAND);
	for(i=0;i<8;i++)
	{
		outb(mcfilter[i], eth_nic_base + 8 + i);
		if(inb(eth_nic_base + 8 + i)!=mcfilter[i])
			printf("Error SMC 83C690 Multicast filter read/write mishap %d\n",i);
	}
	outb(D8390_COMMAND_RD2 + D8390_COMMAND_PS0, eth_nic_base + D8390_P0_COMMAND);
	outb(4 | 0x08, eth_nic_base+D8390_P0_RCR);
}

/**************************************************************************
NS8390_RESET - Reset adapter
**************************************************************************/
static void ns8390_reset(struct nic *nic)
{
	int i;

	eth_drain_receiver = 0;
    outb(D8390_COMMAND_PS0 | D8390_COMMAND_RD2 |
         D8390_COMMAND_STP, eth_nic_base+D8390_P0_COMMAND);
	if (eth_flags & FLAG_16BIT)
		outb(0x49, eth_nic_base+D8390_P0_DCR);
	else
		outb(0x48, eth_nic_base+D8390_P0_DCR);
	outb(0, eth_nic_base+D8390_P0_RBCR0);
	outb(0, eth_nic_base+D8390_P0_RBCR1);
	outb(0x20, eth_nic_base+D8390_P0_RCR);	/* monitor mode */
	outb(2, eth_nic_base+D8390_P0_TCR);
	outb(eth_tx_start, eth_nic_base+D8390_P0_TPSR);
	outb(eth_rx_start, eth_nic_base+D8390_P0_PSTART);
	outb(eth_memsize, eth_nic_base+D8390_P0_PSTOP);
	outb(eth_memsize - 1, eth_nic_base+D8390_P0_BOUND);
	outb(0xFF, eth_nic_base+D8390_P0_ISR);
	outb(0, eth_nic_base+D8390_P0_IMR);
		outb(D8390_COMMAND_PS1 |
			D8390_COMMAND_RD2 | D8390_COMMAND_STP, eth_nic_base+D8390_P0_COMMAND);
	for (i=0; i<ETH_ALEN; i++)
		outb(nic->node_addr[i], eth_nic_base+D8390_P1_PAR0+i);
	for (i=0; i<ETH_ALEN; i++)
		outb(0xFF, eth_nic_base+D8390_P1_MAR0+i);
	outb(eth_rx_start, eth_nic_base+D8390_P1_CURR);
		outb(D8390_COMMAND_PS0 |
			D8390_COMMAND_RD2 | D8390_COMMAND_STA, eth_nic_base+D8390_P0_COMMAND);
	outb(0xFF, eth_nic_base+D8390_P0_ISR);
	outb(0, eth_nic_base+D8390_P0_TCR);	/* transmitter on */
	outb(4, eth_nic_base+D8390_P0_RCR);	/* allow rx broadcast frames */

	enable_multicast(eth_nic_base);
}

static int ns8390_poll(struct nic *nic, int retrieve);

/**************************************************************************
ETH_RX_OVERRUN - Bring adapter back to work after an RX overrun
**************************************************************************/
static void eth_rx_overrun(struct nic *nic)
{
	int start_time;

		outb(D8390_COMMAND_PS0 | D8390_COMMAND_RD2 |
			D8390_COMMAND_STP, eth_nic_base+D8390_P0_COMMAND);

	/* wait for at least 1.6ms - we wait one timer tick */
	start_time = currticks();
	while (currticks() - start_time <= 1)
		/* Nothing */;

	outb(0, eth_nic_base+D8390_P0_RBCR0);	/* reset byte counter */
	outb(0, eth_nic_base+D8390_P0_RBCR1);

	/*
	 * Linux driver checks for interrupted TX here. This is not necessary,
	 * because the transmit routine waits until the frame is sent.
	 */

	/* enter loopback mode and restart NIC */
	outb(2, eth_nic_base+D8390_P0_TCR);
	outb(D8390_COMMAND_PS0 | D8390_COMMAND_RD2 |
	     D8390_COMMAND_STA, eth_nic_base+D8390_P0_COMMAND);

	/* clear the RX ring, acknowledge overrun interrupt */
	eth_drain_receiver = 1;
	while (ns8390_poll(nic, 1))
		/* Nothing */;
	eth_drain_receiver = 0;
	outb(D8390_ISR_OVW, eth_nic_base+D8390_P0_ISR);

	/* leave loopback mode - no packets to be resent (see Linux driver) */
	outb(0, eth_nic_base+D8390_P0_TCR);
}

/**************************************************************************
NS8390_TRANSMIT - Transmit a frame
**************************************************************************/
static void ns8390_transmit(
	struct nic *nic,
	const char *d,			/* Destination */
	unsigned int t,			/* Type */
	unsigned int s,			/* size */
	const char *p)			/* Packet */
{
	{
		/* Programmed I/O */
		unsigned short type;
		type = (t >> 8) | (t << 8);
		eth_pio_write(d, eth_tx_start<<8, ETH_ALEN);
		eth_pio_write(nic->node_addr, (eth_tx_start<<8)+ETH_ALEN, ETH_ALEN);
		/* bcc generates worse code without (const+const) below */
		eth_pio_write((unsigned char *)&type, (eth_tx_start<<8)+(ETH_ALEN+ETH_ALEN), 2);
		eth_pio_write(p, (eth_tx_start<<8)+ETH_HLEN, s);
		s += ETH_HLEN;
		if (s < ETH_ZLEN) s = ETH_ZLEN;
	}
	outb(D8390_COMMAND_PS0 |
		D8390_COMMAND_RD2 | D8390_COMMAND_STA, eth_nic_base+D8390_P0_COMMAND);
	outb(eth_tx_start, eth_nic_base+D8390_P0_TPSR);
	outb(s, eth_nic_base+D8390_P0_TBCR0);
	outb(s>>8, eth_nic_base+D8390_P0_TBCR1);
		outb(D8390_COMMAND_PS0 |
			D8390_COMMAND_TXP | D8390_COMMAND_RD2 |
			D8390_COMMAND_STA, eth_nic_base+D8390_P0_COMMAND);
}

/**************************************************************************
NS8390_POLL - Wait for a frame
**************************************************************************/
static int ns8390_poll(struct nic *nic, int retrieve)
{
	int ret = 0;
	unsigned char rstat, curr, next;
	unsigned short len, frag;
	unsigned short pktoff;
	unsigned char *p;
	struct ringbuffer pkthdr;

	/* avoid infinite recursion: see eth_rx_overrun() */
	if (!eth_drain_receiver && (inb(eth_nic_base+D8390_P0_ISR) & D8390_ISR_OVW)) {
		eth_rx_overrun(nic);
		return(0);
	}
	rstat = inb(eth_nic_base+D8390_P0_RSR);
	if (!(rstat & D8390_RSTAT_PRX)) return(0);
	next = inb(eth_nic_base+D8390_P0_BOUND)+1;
	if (next >= eth_memsize) next = eth_rx_start;
	outb(D8390_COMMAND_PS1, eth_nic_base+D8390_P0_COMMAND);
	curr = inb(eth_nic_base+D8390_P1_CURR);
	outb(D8390_COMMAND_PS0, eth_nic_base+D8390_P0_COMMAND);
	if (curr >= eth_memsize) curr=eth_rx_start;
	if (curr == next) return(0);

	if ( ! retrieve ) return 1;

	pktoff = next << 8;
	if (eth_flags & FLAG_PIO)
		eth_pio_read(pktoff, (char *)&pkthdr, 4);
	else
		memcpy(&pkthdr, bus_to_virt(eth_rmem + pktoff), 4);
	pktoff += sizeof(pkthdr);
	/* incoming length includes FCS so must sub 4 */
	len = pkthdr.len - 4;
	if ((pkthdr.status & D8390_RSTAT_PRX) == 0 || len < ETH_ZLEN
		|| len > ETH_FRAME_LEN) {
		printf("Bogus packet, ignoring\n");
		return (0);
	}
	else {
		p = nic->packet;
		nic->packetlen = len;		/* available to caller */
		frag = (eth_memsize << 8) - pktoff;
		if (len > frag) {		/* We have a wrap-around */
			/* read first part */
			if (eth_flags & FLAG_PIO)
				eth_pio_read(pktoff, p, frag);
			else
				memcpy(p, bus_to_virt(eth_rmem + pktoff), frag);
			pktoff = eth_rx_start << 8;
			p += frag;
			len -= frag;
		}
		/* read second part */
		if (eth_flags & FLAG_PIO)
			eth_pio_read(pktoff, p, len);
		else
			memcpy(p, bus_to_virt(eth_rmem + pktoff), len);
		ret = 1;
	}
	next = pkthdr.next;		/* frame number of next packet */
	if (next == eth_rx_start)
		next = eth_memsize;
	outb(next-1, eth_nic_base+D8390_P0_BOUND);
	return(ret);
}

/**************************************************************************
NS8390_DISABLE - Turn off adapter
**************************************************************************/
static void ns8390_disable(struct dev *dev)
{
	struct nic *nic = (struct nic *)dev;
	/* reset and disable merge */
	ns8390_reset(nic);
}

/**************************************************************************
NS8390_IRQ - Enable, Disable, or Force interrupts
**************************************************************************/
static void ns8390_irq(struct nic *nic __unused, irq_action_t action __unused)
{
  switch ( action ) {
  case DISABLE :
    break;
  case ENABLE :
    break;
  case FORCE :
    break;
  }
}

/**************************************************************************
ETH_PROBE - Look for an adapter
**************************************************************************/
static int eth_probe (struct dev *dev, unsigned short *probe_addrs __unused)
{
	struct nic *nic = (struct nic *)dev;
	int i;
	eth_vendor = VENDOR_NONE;
	eth_drain_receiver = 0;

	nic->irqno  = 0;

{
	/******************************************************************
	Search for NE1000/2000 if no WD/SMC or 3com cards
	******************************************************************/
	unsigned char c;
	if (eth_vendor == VENDOR_NONE) {
		char romdata[16], testbuf[32];
		int idx;
		static char test[] = "NE*000 memory";
		static unsigned short base[] = {
			NE_SCAN,
			0 };
		/* if no addresses supplied, fall back on defaults */
		if (probe_addrs == 0 || probe_addrs[0] == 0)
			probe_addrs = base;
		eth_bmem = 0;		/* No shared memory */
		for (idx = 0; (eth_nic_base = probe_addrs[idx]) != 0; ++idx) {
			eth_flags = FLAG_PIO;
			eth_asic_base = eth_nic_base + NE_ASIC_OFFSET;
			eth_memsize = MEM_12288;
			eth_tx_start = 32;
			eth_rx_start = 32 + D8390_TXBUF_SIZE;

			/* Reset */
			c = inb(eth_asic_base + NE_RESET);
			outb(c, eth_asic_base + NE_RESET);

			/* Sleep to allow the reset to take place */
			inb(0x84);
			inb(0x84);
			inb(0x84);
			inb(0x84);

			/* Stop the card */
			outb(D8390_COMMAND_STP |
				D8390_COMMAND_RD2, eth_nic_base + D8390_P0_COMMAND);

			/* Set FIFO threshold to 8, no auto-init remote DNA, byte order=80x86, word-wide DMA transfers */
			outb(D8390_DCR_FT1 | D8390_DCR_LS | D8390_DCR_WTS, eth_nic_base + D8390_P0_DCR);
            eth_flags |= FLAG_16BIT;

			/* TODO Test card address */
		}
        printf("Hard code card address for now\n");
		eth_nic_base = 768;

		if (eth_nic_base == 0)
			return (0);
		eth_vendor = VENDOR_NOVELL;
		eth_pio_read(0, romdata, sizeof(romdata));
		for (i=0; i<ETH_ALEN; i++) {
			nic->node_addr[i] = romdata[i + ((eth_flags & FLAG_16BIT) ? i : 0)];
		}
		nic->ioaddr = eth_nic_base;
		printf("\nNE%c000 base %#hx, addr %!\n",
			(eth_flags & FLAG_16BIT) ? '2' : '1', eth_nic_base,
			nic->node_addr);
	}
}
	if (eth_vendor == VENDOR_NONE)
		return(0);
	eth_rmem = eth_bmem;
	ns8390_reset(nic);

	dev->disable  = ns8390_disable;
	nic->poll     = ns8390_poll;
	nic->transmit = ns8390_transmit;
	nic->irq      = ns8390_irq;

    /* Based on PnP ISA map */
    dev->devid.vendor_id = htons(GENERIC_ISAPNP_VENDOR);
    dev->devid.device_id = htons(0x80d6);
	return 1;
}

static struct isa_driver ne_driver __isa_driver = {
	.type    = NIC_DRIVER,
	.name    = "NE*000",
	.probe   = ne_probe,
	.ioaddrs = 0,
};










// pointers to locations in the RTL8019 receive buffer
static unsigned char NextPage;              // page pointer to next Rx packet
static unsigned int CurrentRetreiveAddress; // DMA address for read Rx packet location


void nicInit(void)
{
    rtl8019Init();
}

void nicSend(unsigned int len, unsigned char* packet)
{
    rtl8019BeginPacketSend(len);
    rtl8019SendPacketData(packet, len);
    rtl8019EndPacketSend();
}

unsigned int nicPoll(unsigned int maxlen, unsigned char* packet)
{
    unsigned int packetLength;

    packetLength = rtl8019BeginPacketRetreive();

    // if there's no packet or an error - exit without ending the operation
    if( !packetLength )
        return 0;

    // drop anything too big for the buffer
    if( packetLength > maxlen )
    {
        rtl8019EndPacketRetreive();
        return 0;
    }

    // copy the packet data into the packet buffer
    rtl8019RetreivePacketData( packet, packetLength );
    rtl8019EndPacketRetreive();

    return packetLength;
}

void nicGetMacAddress(u08* macaddr)
{
    u08 tempCR;
    // switch register pages
    tempCR = rtl8019Read(CR);
    rtl8019Write(CR,tempCR|PS0);
    // read MAC address registers
    *macaddr++ = rtl8019Read(PAR0);
    *macaddr++ = rtl8019Read(PAR1);
    *macaddr++ = rtl8019Read(PAR2);
    *macaddr++ = rtl8019Read(PAR3);
    *macaddr++ = rtl8019Read(PAR4);
    *macaddr++ = rtl8019Read(PAR5);
    // switch register pages back
    rtl8019Write(CR,tempCR);
}

void nicSetMacAddress(u08* macaddr)
{
    u08 tempCR;
    // switch register pages
    tempCR = rtl8019Read(CR);
    rtl8019Write(CR,tempCR|PS0);
    // write MAC address registers
    rtl8019Write(PAR0, *macaddr++);
    rtl8019Write(PAR1, *macaddr++);
    rtl8019Write(PAR2, *macaddr++);
    rtl8019Write(PAR3, *macaddr++);
    rtl8019Write(PAR4, *macaddr++);
    rtl8019Write(PAR5, *macaddr++);
    // switch register pages back
    rtl8019Write(CR,tempCR);
}

void nicRegDump(void)
{
    rtl8019RegDump();
}


void rtl8019SetupPorts(void)
{
#if NIC_CONNECTION == MEMORY_MAPPED
    // enable external SRAM interface - no wait states
    sbi(MCUCR, SRE);
//  sbi(MCUCR, SRW10);
//  sbi(XMCRA, SRW00);
//  sbi(XMCRA, SRW01);
//  sbi(XMCRA, SRW11);
#else
    // make the address port output
    RTL8019_ADDRESS_DDR |= RTL8019_ADDRESS_MASK;
    // make the data port input with pull-ups
    RTL8019_DATA_PORT = 0xFF;

    // initialize the control port read and write pins to de-asserted
    RTL8019_CONTROL_DDR |= (1<<RTL8019_CONTROL_READPIN);
    RTL8019_CONTROL_DDR |= (1<<RTL8019_CONTROL_WRITEPIN);
    // set the read and write pins to output
    RTL8019_CONTROL_PORT |= (1<<RTL8019_CONTROL_READPIN);
    RTL8019_CONTROL_PORT |= (1<<RTL8019_CONTROL_WRITEPIN);
#endif
    // set reset pin to output
    sbi(RTL8019_RESET_DDR, RTL8019_RESET_PIN);
}


#if NIC_CONNECTION == MEMORY_MAPPED
inline void rtl8019Write(u08 address, u08 data)
{
    *(volatile u08*)(RTL8019_MEMORY_MAPPED_OFFSET + address) = data;
}
#else
void rtl8019Write(unsigned char address, unsigned char data)
{
    // assert the address
    RTL8019_ADDRESS_PORT = address | (RTL8019_ADDRESS_PORT&~RTL8019_ADDRESS_MASK);
    // set data bus as output and place data on bus
    RTL8019_DATA_DDR = 0xFF;
    RTL8019_DATA_PORT = data;
    // clock write pin
    cbi(RTL8019_CONTROL_PORT, RTL8019_CONTROL_WRITEPIN);
    nop();
    nop();
    sbi(RTL8019_CONTROL_PORT, RTL8019_CONTROL_WRITEPIN);
    // set data port back to input with pullups enabled
    RTL8019_DATA_DDR = 0x00;
    RTL8019_DATA_PORT = 0xFF;
}
#endif


#if NIC_CONNECTION == MEMORY_MAPPED
inline u08 ax88796Read(u08 address)
{
    return *(volatile u08*)(RTL8019_MEMORY_MAPPED_OFFSET + address);
}
#else
unsigned char rtl8019Read(unsigned char address)
{
    unsigned char data;

    // assert the address
    RTL8019_ADDRESS_PORT = address | (RTL8019_ADDRESS_PORT&~RTL8019_ADDRESS_MASK);
    // assert read
    cbi(RTL8019_CONTROL_PORT, RTL8019_CONTROL_READPIN);
    nop();
    nop();
    // read in the data
    data = RTL8019_DATA_PIN;
    // negate read
    sbi(RTL8019_CONTROL_PORT, RTL8019_CONTROL_READPIN);
    // return data
    return data;
}
#endif


void rtl8019Init(void)
{
    // setup I/O ports
    rtl8019SetupPorts();

    // do a hard reset
    sbi(RTL8019_RESET_PORT, RTL8019_RESET_PIN);
    delay_ms(10);
    cbi(RTL8019_RESET_PORT, RTL8019_RESET_PIN);

    // clear interrupt state
    rtl8019Write( ISR, rtl8019Read(ISR) );
    delay_ms(50);

    // switch to page 3 to load config registers
    rtl8019Write(CR, (PS0|PS1|RD2|STOP));

    // disable EEPROM write protect of config registers
    rtl8019Write(RTL_EECR, (EEM1|EEM0));

    // set network type to 10 Base-T link test
    rtl8019Write(CONFIG2, 0x20);

    // disable powerdown and sleep
    rtl8019Write(CONFIG3, 0);
    delay_ms(255);

    // reenable EEPROM write protect
    rtl8019Write(RTL_EECR, 0);

    // go back to page 0, stop NIC, abort DMA
    rtl8019Write(CR, (RD2|STOP));
    delay_ms(2);                    // wait for traffic to complete
    rtl8019Write(DCR, DCR_INIT);
    rtl8019Write(RBCR0,0x00);
    rtl8019Write(RBCR1,0x00);
    rtl8019Write(RCR, AB);
    rtl8019Write(TPSR, TXSTART_INIT);
    rtl8019Write(TCR, LB0);
    rtl8019Write(PSTART, RXSTART_INIT);
    rtl8019Write(BNRY, RXSTART_INIT);
    rtl8019Write(PSTOP, RXSTOP_INIT);
    rtl8019Write(CR, (PS0|RD2|STOP));   // switch to page 1
    delay_ms(2);
    rtl8019Write(CPR, RXSTART_INIT);

    // initialize sequence per NE2000 spec
    rtl8019Write(CR, (RD2|STOP));
    rtl8019Write(DCR, DCR_INIT);
    rtl8019Write(CR, (RD2|START));
    rtl8019Write(ISR,0xFF);         // clear all interrupts
    rtl8019Write(IMR, IMR_INIT);
    rtl8019Write(TCR, TCR_INIT);

    rtl8019Write(CR, (RD2|START));  // start the NIC
}


void rtl8019BeginPacketSend(unsigned int packetLength)
{
    unsigned int sendPacketLength;
    sendPacketLength = (packetLength>=ETHERNET_MIN_PACKET_LENGTH)?
                        (packetLength):ETHERNET_MIN_PACKET_LENGTH;

    //start the NIC
    rtl8019Write(CR, (RD2|START));

    // still transmitting a packet - wait for it to finish
    while( rtl8019Read(CR) & TXP );

    // load beginning page for transmit buffer
    rtl8019Write(TPSR,TXSTART_INIT);

    // set start address for remote DMA operation
    rtl8019Write(RSAR0,0x00);
    rtl8019Write(RSAR1,0x40);

    // clear the packet stored interrupt
    rtl8019Write(ISR,PTX);

    // load data byte count for remote DMA
    rtl8019Write(RBCR0, (unsigned char)(packetLength));
    rtl8019Write(RBCR1, (unsigned char)(packetLength>>8));

    rtl8019Write(TBCR0, (unsigned char)(sendPacketLength));
    rtl8019Write(TBCR1, (unsigned char)((sendPacketLength)>>8));

    // do remote write operation
    rtl8019Write(CR,(RD1|START));
}


void rtl8019SendPacketData(unsigned char *localBuffer, unsigned int length)
{
    unsigned int i;

    // write data to DMA port
    for(i=0;i<length;i++)
        rtl8019Write(RDMAPORT, localBuffer[i]);
}


void rtl8019EndPacketSend(void)
{
    //send the contents of the transmit buffer onto the network
    rtl8019Write(CR,(RD2|TXP));
    // clear the remote DMA interrupt
    rtl8019Write(ISR, RDC);
}


unsigned int rtl8019BeginPacketRetreive(void)
{
    unsigned char i;
    unsigned char bnry;

    unsigned char pageheader[4];
    unsigned int rxlen;

    // check for and handle an overflow
    rtl8019ProcessInterrupt();

    // read CPR from page 1
    rtl8019Write(CR,(PS0|RD2|START));
    i = rtl8019Read(CPR);

    // return to page 0
    rtl8019Write(CR,(RD2|START));

    // read the boundary register - pointing to the beginning of the packet
    bnry = rtl8019Read(BNRY) ;

    // return if there is no packet in the buffer
    if( bnry == i )
        return 0;

    // clear the packet received interrupt flag
    rtl8019Write(ISR, PRX);

    // if boundary pointer is invalid
    if( (bnry >= RXSTOP_INIT) || (bnry < RXSTART_INIT) )
    {
        // reset the contents of the buffer and exit
        rtl8019Write(BNRY, RXSTART_INIT);
        rtl8019Write(CR, (PS0|RD2|START));
        rtl8019Write(CPR, RXSTART_INIT);
        rtl8019Write(CR, (RD2|START));
        return 0;
    }

    // initiate DMA to transfer the RTL8019 packet header
    rtl8019Write(RBCR0, 4);
    rtl8019Write(RBCR1, 0);
    rtl8019Write(RSAR0, 0);
    rtl8019Write(RSAR1, bnry);
    rtl8019Write(CR, (RD0|START));
    // transfer packet header
    for(i=0;i<4;i++)
        pageheader[i] = rtl8019Read(RDMAPORT);
    // end the DMA operation
    rtl8019Write(CR, (RD2|START));
    // wait for remote DMA complete
    for(i = 0; i < 20; i++)
        if(rtl8019Read(ISR) & RDC)
            break;
    rtl8019Write(ISR, RDC);

    rxlen = (pageheader[PKTHEADER_PKTLENH]<<8) + pageheader[PKTHEADER_PKTLENL];
    NextPage = pageheader[PKTHEADER_NEXTPAGE];

    CurrentRetreiveAddress = (bnry<<8) + 4;

    // if the NextPage pointer is invalid, the packet is not ready yet - exit
    if( (NextPage >= RXSTOP_INIT) || (NextPage < RXSTART_INIT) )
        return 0;

    return rxlen-4;
}


void rtl8019RetreivePacketData(unsigned char * localBuffer, unsigned int length)
{
    unsigned int i;

    // initiate DMA to transfer the data
    rtl8019Write(RBCR0, (unsigned char)length);
    rtl8019Write(RBCR1, (unsigned char)(length>>8));
    rtl8019Write(RSAR0, (unsigned char)CurrentRetreiveAddress);
    rtl8019Write(RSAR1, (unsigned char)(CurrentRetreiveAddress>>8));
    rtl8019Write(CR, (RD0|START));
    // transfer packet data
    for(i=0;i<length;i++)
        localBuffer[i] = rtl8019Read(RDMAPORT);
    // end the DMA operation
    rtl8019Write(CR, (RD2|START));
    // wait for remote DMA complete
    for(i=0; i<20; i++)
        if(rtl8019Read(ISR) & RDC)
            break;
    rtl8019Write(ISR, RDC);
    // keep track of current address
    CurrentRetreiveAddress += length;
    if( CurrentRetreiveAddress >= 0x6000 )
        CurrentRetreiveAddress = CurrentRetreiveAddress - (0x6000-0x4600) ;
}


void rtl8019EndPacketRetreive(void)
{
    unsigned char i;

    // end the DMA operation
    rtl8019Write(CR, (RD2|START));
    // wait for remote DMA complete
    for(i=0; i<20; i++)
        if(rtl8019Read(ISR) & RDC)
            break;
    rtl8019Write(ISR, RDC);

    // set the boundary register to point to the start of the next packet
    rtl8019Write(BNRY, NextPage);
}


void rtl8019ProcessInterrupt(void)
{
    unsigned char byte = rtl8019Read(ISR);

    if( byte & OVW )
        rtl8019ReceiveOverflowRecover();
}

void rtl8019ReceiveOverflowRecover(void)
{
    unsigned char data_L, resend;

    data_L = rtl8019Read(CR);
    rtl8019Write(CR, 0x21);
    delay_ms(2);
    rtl8019Write(RBCR0, 0x00);
    rtl8019Write(RBCR1, 0x00);
    if(!(data_L & 0x04))
        resend = 0;
    else if(data_L & 0x04)
    {
        data_L = rtl8019Read(ISR);
        if((data_L & 0x02) || (data_L & 0x08))
            resend = 0;
        else
            resend = 1;
    }

    rtl8019Write(TCR, 0x02);
    rtl8019Write(CR, 0x22);
    rtl8019Write(BNRY, RXSTART_INIT);
    rtl8019Write(CR, 0x62);
    rtl8019Write(CPR, RXSTART_INIT);
    rtl8019Write(CR, 0x22);
    rtl8019Write(ISR, 0x10);
    rtl8019Write(TCR, TCR_INIT);

    if(resend)
        rtl8019Write(CR, 0x26);

    rtl8019Write(ISR, 0xFF);
}








/*
 * Local variables:
 *  c-basic-offset: 8
 * End:
 */





