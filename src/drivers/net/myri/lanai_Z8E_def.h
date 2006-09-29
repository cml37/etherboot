/************************************************* -*- linux-c -*-
 * Myricom 10Gb Network Interface Card Software
 * Copyright 2006, Myricom, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 ****************************************************************/

#ifndef LANAI_Z8E_DEF_H
#define LANAI_Z8E_DEF_H


/*****************************
 ** Memory-mapped registers **
 *****************************/

/* Port 0 receive special registers, 0-5 */
#define	P0_RECV_REG_OFFSET	0
#define	P0_RECV_REG(A)		(((P0_RECV_REG_OFFSET+A)<<3))
#define	P0_RECV_REG_MSH(A)	(((P0_RECV_REG_OFFSET+A)<<3)+0)
#define	P0_RECV_REG_LSH(A)	(((P0_RECV_REG_OFFSET+A)<<3)+2)
#define	P0_RECV_REG_MSB(A)	(((P0_RECV_REG_OFFSET+A)<<3)+0)
#define	P0_RECV_REG_msb(A)	(((P0_RECV_REG_OFFSET+A)<<3)+1)
#define	P0_RECV_REG_lsb(A)	(((P0_RECV_REG_OFFSET+A)<<3)+2)
#define	P0_RECV_REG_LSB(A)	(((P0_RECV_REG_OFFSET+A)<<3)+3)

/* Port 0 send special registers, 0-1 */
#define	P0_SEND_REG_OFFSET	6
#define	P0_SEND_REG(A)		(((P0_SEND_REG_OFFSET+A)<<3))
#define	P0_SEND_REG_MSH(A)	(((P0_SEND_REG_OFFSET+A)<<3)+0)
#define	P0_SEND_REG_LSH(A)	(((P0_SEND_REG_OFFSET+A)<<3)+2)
#define	P0_SEND_REG_MSB(A)	(((P0_SEND_REG_OFFSET+A)<<3)+0)
#define	P0_SEND_REG_msb(A)	(((P0_SEND_REG_OFFSET+A)<<3)+1)
#define	P0_SEND_REG_lsb(A)	(((P0_SEND_REG_OFFSET+A)<<3)+2)
#define	P0_SEND_REG_LSB(A)	(((P0_SEND_REG_OFFSET+A)<<3)+3)

/* Port 1 receive special registers, 0-2 */
#define	PCIE_RECV_REG_OFFSET	8
#define	PCIE_RECV_REG(A)	(((PCIE_RECV_REG_OFFSET+A)<<3))
#define	PCIE_RECV_REG_MSH(A)	(((PCIE_RECV_REG_OFFSET+A)<<3)+0)
#define	PCIE_RECV_REG_LSH(A)	(((PCIE_RECV_REG_OFFSET+A)<<3)+2)
#define	PCIE_RECV_REG_MSB(A)	(((PCIE_RECV_REG_OFFSET+A)<<3)+0)
#define	PCIE_RECV_REG_msb(A)	(((PCIE_RECV_REG_OFFSET+A)<<3)+1)
#define	PCIE_RECV_REG_lsb(A)	(((PCIE_RECV_REG_OFFSET+A)<<3)+2)
#define	PCIE_RECV_REG_LSB(A)	(((PCIE_RECV_REG_OFFSET+A)<<3)+3)

/* Port 1 send special registers, 0-1 */
#define	PCIE_SEND_REG_OFFSET	11
#define	PCIE_SEND_REG(A)	(((PCIE_SEND_REG_OFFSET+A)<<3))
#define	PCIE_SEND_REG_MSH(A)	(((PCIE_SEND_REG_OFFSET+A)<<3)+0)
#define	PCIE_SEND_REG_LSH(A)	(((PCIE_SEND_REG_OFFSET+A)<<3)+2)
#define	PCIE_SEND_REG_MSB(A)	(((PCIE_SEND_REG_OFFSET+A)<<3)+0)
#define	PCIE_SEND_REG_msb(A)	(((PCIE_SEND_REG_OFFSET+A)<<3)+1)
#define	PCIE_SEND_REG_lsb(A)	(((PCIE_SEND_REG_OFFSET+A)<<3)+2)
#define	PCIE_SEND_REG_LSB(A)	(((PCIE_SEND_REG_OFFSET+A)<<3)+3)

/* Copy-Engine special registers, 0-3 */
#define	COPY_REG_OFFSET		13
#define	COPY_REG(A)		(((COPY_REG_OFFSET+A)<<3))
#define	COPY_REG_MSH(A)		(((COPY_REG_OFFSET+A)<<3)+0)
#define	COPY_REG_LSH(A)		(((COPY_REG_OFFSET+A)<<3)+2)
#define	COPY_REG_MSB(A)		(((COPY_REG_OFFSET+A)<<3)+0)
#define	COPY_REG_msb(A)		(((COPY_REG_OFFSET+A)<<3)+1)
#define	COPY_REG_lsb(A)		(((COPY_REG_OFFSET+A)<<3)+2)
#define	COPY_REG_LSB(A)		(((COPY_REG_OFFSET+A)<<3)+3)

/* CRC32-Engine special registers, 0-3 */
#define	CRC32_REG_OFFSET	17
#define	CRC32_REG(A)		(((CRC32_REG_OFFSET+A)<<3))
#define	CRC32_REG_MSH(A)	(((CRC32_REG_OFFSET+A)<<3)+0)
#define	CRC32_REG_LSH(A)	(((CRC32_REG_OFFSET+A)<<3)+2)
#define	CRC32_REG_MSB(A)	(((CRC32_REG_OFFSET+A)<<3)+0)
#define	CRC32_REG_msb(A)	(((CRC32_REG_OFFSET+A)<<3)+1)
#define	CRC32_REG_lsb(A)	(((CRC32_REG_OFFSET+A)<<3)+2)
#define	CRC32_REG_LSB(A)	(((CRC32_REG_OFFSET+A)<<3)+3)

/* Dispatch special registers, 0-5 */
#define	DISPATCH_REG_OFFSET	21
#define	DISPATCH_REG(A)		(((DISPATCH_REG_OFFSET+A)<<3))
#define	DISPATCH_REG_MSH(A)	(((DISPATCH_REG_OFFSET+A)<<3)+0)
#define	DISPATCH_REG_LSH(A)	(((DISPATCH_REG_OFFSET+A)<<3)+2)
#define	DISPATCH_REG_MSB(A)	(((DISPATCH_REG_OFFSET+A)<<3)+0)
#define	DISPATCH_REG_msb(A)	(((DISPATCH_REG_OFFSET+A)<<3)+1)
#define	DISPATCH_REG_lsb(A)	(((DISPATCH_REG_OFFSET+A)<<3)+2)
#define	DISPATCH_REG_LSB(A)	(((DISPATCH_REG_OFFSET+A)<<3)+3)

/* JTAG special registers, 0 */
#define	JTAG_REG_OFFSET		27
#define	JTAG_REG(A)		(((JTAG_REG_OFFSET+A)<<3))
#define	JTAG_REG_MSH(A)		(((JTAG_REG_OFFSET+A)<<3)+0)
#define	JTAG_REG_LSH(A)		(((JTAG_REG_OFFSET+A)<<3)+2)
#define	JTAG_REG_MSB(A)		(((JTAG_REG_OFFSET+A)<<3)+0)
#define	JTAG_REG_msb(A)		(((JTAG_REG_OFFSET+A)<<3)+1)
#define	JTAG_REG_lsb(A)		(((JTAG_REG_OFFSET+A)<<3)+2)
#define	JTAG_REG_LSB(A)		(((JTAG_REG_OFFSET+A)<<3)+3)

/* PCIE special registers, 0-24 */
#define	PCIE_REG_OFFSET		28
#define	PCIE_REG(A)		(((PCIE_REG_OFFSET+A)<<3))
#define	PCIE_REG_MSH(A)		(((PCIE_REG_OFFSET+A)<<3)+0)
#define	PCIE_REG_LSH(A)		(((PCIE_REG_OFFSET+A)<<3)+2)
#define	PCIE_REG_MSB(A)		(((PCIE_REG_OFFSET+A)<<3)+0)
#define	PCIE_REG_msb(A)		(((PCIE_REG_OFFSET+A)<<3)+1)
#define	PCIE_REG_lsb(A)		(((PCIE_REG_OFFSET+A)<<3)+2)
#define	PCIE_REG_LSB(A)		(((PCIE_REG_OFFSET+A)<<3)+3)

/* Regular special registers, 0-9 */
#define	SPEC_REG_OFFSET		53
#define	SPEC_REG(A)		(((SPEC_REG_OFFSET+A)<<3))
#define	SPEC_REG_MSH(A)		(((SPEC_REG_OFFSET+A)<<3)+0)
#define	SPEC_REG_LSH(A)		(((SPEC_REG_OFFSET+A)<<3)+2)
#define	SPEC_REG_MSB(A)		(((SPEC_REG_OFFSET+A)<<3)+0)
#define	SPEC_REG_msb(A)		(((SPEC_REG_OFFSET+A)<<3)+1)
#define	SPEC_REG_lsb(A)		(((SPEC_REG_OFFSET+A)<<3)+2)
#define	SPEC_REG_LSB(A)		(((SPEC_REG_OFFSET+A)<<3)+3)

/* ISR-related special registers, 0 */
#define	ISR_REG_OFFSET		63
#define	ISR_REG(A)		(((ISR_REG_OFFSET+A)<<3))
#define	ISR_REG_MSH(A)		(((ISR_REG_OFFSET+A)<<3)+0)
#define	ISR_REG_LSH(A)		(((ISR_REG_OFFSET+A)<<3)+2)
#define	ISR_REG_MSB(A)		(((ISR_REG_OFFSET+A)<<3)+0)
#define	ISR_REG_msb(A)		(((ISR_REG_OFFSET+A)<<3)+1)
#define	ISR_REG_lsb(A)		(((ISR_REG_OFFSET+A)<<3)+2)
#define	ISR_REG_LSB(A)		(((ISR_REG_OFFSET+A)<<3)+3)



#define MMREG_VOID_PTR(A)      (*((void* volatile * const ) (0xFFFFFE00 + (A))))
#define MMREG_WORD(A) (*((unsigned int   volatile * const ) (0xFFFFFE00 + (A))))
#define MMREG_HALF(A) (*((unsigned short volatile * const ) (0xFFFFFE00 + (A))))
#define MMREG_BYTE(A) (*((unsigned char  volatile * const ) (0xFFFFFE00 + (A))))




#define	CRC32			MMREG_WORD(CRC32_REG(0))
#define	CRC32_BYTE		MMREG_BYTE(CRC32_REG_lsb(1))
#define	CRC32_HALF		MMREG_HALF(CRC32_REG_MSH(2))
#define	CRC32_WORD		MMREG_WORD(CRC32_REG(3))

#define	COPY			MMREG_WORD(COPY_REG(0))
#define	COPY_FROM		MMREG_VOID_PTR(COPY_REG(0))
#define	COPY_SIZE		MMREG_WORD(COPY_REG(0))
#define	COPY_TO			MMREG_VOID_PTR(COPY_REG(0))
#define		COPY_TO_CRC32		(0x80000000)
#define		COPY_LENGTH		(0x7FFFFFFF)

#define COPY_STATE		MMREG_BYTE(COPY_REG_LSB(1))
#define CRC32_CONFIG		MMREG_BYTE(COPY_REG_lsb(2))
#define	COPY_BLOCK		MMREG_WORD(COPY_REG(3))

#define	IT0			MMREG_WORD(SPEC_REG(0))
#define	IT1			MMREG_WORD(SPEC_REG(1))
#define	IT2			MMREG_WORD(SPEC_REG(2))
#define	IT3			MMREG_WORD(SPEC_REG(3))
#define IT(n)			MMREG_WORD(SPEC_REG(n))
#define	CPUC			MMREG_WORD(SPEC_REG(4))
#define	RTC			MMREG_WORD(SPEC_REG(5))

#define	LED			MMREG_BYTE(SPEC_REG_MSB(6))

#define	MDI			MMREG_BYTE(SPEC_REG_msb(7))
#define		MDC			(0x04)
#define		MDEN			(0x02)
#define		MDIO			(0x01)

#define PORT_ENABLE		MMREG_BYTE(SPEC_REG_MSB(8))
#define PORT_ENABLE_P0		(0x01)
#define PORT_ENABLE_PCIE	(0x06)
#define PORT_ENABLE_PCIE_PHY	(0x04)

#define	REBOOT_STATUS		MMREG_WORD(SPEC_REG(9))
#define		REBOOT_CAUSE			(0xFC000000)
#define		REBOOT_TIME3_INT		(0x80000000)
#define		REBOOT_MEMORY_INT		(0x40000000)
#define		REBOOT_SEND_BUFFER_PARITY_INT	(0x20000000)
#define		REBOOT_PCIE_BUFFER_PARITY_INT	(0x10000000)
#define		REBOOT_P0_BUFFER_PARITY_INT	(0x08000000)
#define		REBOOT_SRAM_PARITY_INT		(0x04000000)
#define		REBOOT_ADDR			(0x03FFFFFF)

#define	P0_RECV_BUFFER_SIZE	(48*1024)
#define	PCIE_RECV_BUFFER_SIZE	(32*1024)
#define	RECV_BUFFER_WIDTH	16
#define	P0_CHIP_BUFFER		((void*)(0x80000000))
#define	PCIE_CHIP_BUFFER	((void*)(0x80010000))
#define	P0_SRAM_BUFFER		((void*)(SRAM_SIZE-P0_RECV_BUFFER_SIZE))
#define	P0_A_SRAM_BUFFER	((void*)(SRAM_SIZE-(2*P0_RECV_BUFFER_SIZE)))
#define	PCIE_SRAM_BUFFER	((void*)(SRAM_SIZE-(2*P0_RECV_BUFFER_SIZE+PCIE_RECV_BUFFER_SIZE)))

#define	WSCAN			((unsigned int*)((char*)PCIE_SRAM_BUFFER-0x04))
#define	RSCAN			((unsigned int*)((char*)PCIE_SRAM_BUFFER-0x44))
#define	ASCAN			((unsigned int*)((char*)PCIE_SRAM_BUFFER-0xa4))
#define	SCAN_BOTTOM		((unsigned int*)((char*)PCIE_SRAM_BUFFER-0x100))
#define	STRING_SPECS		((char        *)((char*)PCIE_SRAM_BUFFER-0x200))
#define	HIGHMEM_START		((void *)STRING_SPECS)

#define	SEND_FIFO_SIZE		(5*1024)
#define	SEND_FIFO_WIDTH		16

#define	WRITE_FIFO_SIZE		(6*1024)

#define	HARDWARE_COMPLETIONS	16

#define	P0_SEND			MMREG_VOID_PTR(P0_SEND_REG(0))
#define	P0_SEND_POINTER		MMREG_VOID_PTR(P0_SEND_REG(0))
#define	P0_SEND_LENGTH		MMREG_WORD(P0_SEND_REG(0))
#define	P0_SEND_COUNT		MMREG_HALF(P0_SEND_REG_LSH(1))

#define	P0_RECV_POINTER		MMREG_HALF(P0_RECV_REG_MSH(0))
#define	P0_RECV_COUNT		MMREG_HALF(P0_RECV_REG_LSH(0))
#define	P0_RECV_SNAPSHOT        MMREG_WORD(P0_RECV_REG(0))
#define		RECV_POINTER		(0xFFFF0000)
#define		RECV_COUNT		(0x0000FFFF)
#define	P0_DROP_COUNT		MMREG_WORD(P0_RECV_REG(1))
#define		PACKET_DROP		(0xFFFF0000)
#define		OVERFLOW_DROP		(0x0000FFFF)
#define	P0_RECV_LIMIT		MMREG_HALF(P0_RECV_REG_LSH(2))

#define	P0_A_RECV_POINTER	MMREG_HALF(P0_RECV_REG_MSH(3))
#define	P0_A_RECV_COUNT		MMREG_HALF(P0_RECV_REG_LSH(3))
#define	P0_A_RECV_SNAPSHOT	MMREG_WORD(P0_RECV_REG(3))
#define	P0_A_DROP_COUNT		MMREG_WORD(P0_RECV_REG(4))
#define	P0_A_RECV_LIMIT		MMREG_HALF(P0_RECV_REG_MSH(5))

#define	PCIE_SEND		MMREG_VOID_PTR(PCIE_SEND_REG(0))
#define	PCIE_SEND_POINTER	MMREG_VOID_PTR(PCIE_SEND_REG(0))
#define	PCIE_SEND_LENGTH	MMREG_WORD(PCIE_SEND_REG(0))
#define	PCIE_SEND_COUNT		MMREG_HALF(PCIE_SEND_REG_MSH(1))

#define	PCIE_RECV_POINTER	MMREG_HALF(PCIE_RECV_REG_MSH(0))
#define	PCIE_RECV_COUNT		MMREG_HALF(PCIE_RECV_REG_LSH(0))
#define	PCIE_RECV_SNAPSHOT	MMREG_WORD(PCIE_RECV_REG(0))
#define	PCIE_DROP_COUNT		MMREG_WORD(PCIE_RECV_REG(1))
	/* how do we define the drop count ????? */
#define	PCIE_RECV_LIMIT		MMREG_HALF(PCIE_RECV_REG_LSH(2))

#define	POINTER_LIMIT_MASK	(0x7FF0)

#define	ISR			MMREG_WORD(ISR_REG(0))

#define		MEMORY_INT		(0x20000000)
#define		SEND_BUFFER_PARITY_INT	(0x10000000)
#define		PCIE_BUFFER_PARITY_INT	(0x08000000)
#define		P0_BUFFER_PARITY_INT	(0x04000000)
#define		SRAM_PARITY_INT		(0x02000000)

#define		WAKE_INT		(0x01000000)
#define		TIME3_INT		(0x00800000)
#define		TIME2_INT		(0x00400000)
#define		TIME1_INT		(0x00200000)
#define		TIME0_INT		(0x00100000)
#define		TIME_INT(n)		(0x00100000<<(n))
#define         COPY_BUSY               (0x00080000)

#define		PCIE_EVENT		(0x00040000)
#define		PCIE_DLL_BUSY		(0x00020000)
#define		PCIE_FC_VC1_CPL_READY	(0x00010000)
#define		PCIE_FC_VC1_NP_READY	(0x00008000)
#define		PCIE_FC_VC1_P_READY	(0x00004000)
#define		PCIE_FC_VC0_CPL_READY	(0x00002000)
#define		PCIE_FC_VC0_NP_READY	(0x00001000)
#define		PCIE_FC_VC0_P_READY	(0x00000800)
#define		PCIE_FC_READY(vc,t)	(0x00000800<<((vc)*3+(t)))
#define		PCIE_PACKET_ACKD	(0x00000400)
#define		PCIE_PACKET_RCVD	(0x00000200)
#define		PCIE_PACKET_SENT	(0x00000100)
#define		PCIE_LINK_DOWN_INT	(0x00000080)

#define		P0_TX_STOPPED		(0x00000040)
#define		P0_RX_STOPPED		(0x00000020)
#define		P0_A_PACKET_RCVD	(0x00000010)
#define		P0_PACKET_HEAD		(0x00000008)
#define		P0_PACKET_RCVD		(0x00000004)
#define		P0_PACKET_SENT		(0x00000002)
#define		P0_LINK_DOWN_INT	(0x00000001)


#define PCIE_FC_VC0_P_LIMIT	 MMREG_WORD(PCIE_REG(0))
#define PCIE_FC_VC0_P_CONSUMED	 MMREG_WORD(PCIE_REG(1))
#define PCIE_FC_VC0_P_REQUEST	 MMREG_HALF(PCIE_REG_LSH(2))
#define PCIE_FC_VC0_NP_LIMIT	 MMREG_WORD(PCIE_REG(3))
#define PCIE_FC_VC0_NP_CONSUMED	 MMREG_WORD(PCIE_REG(4))
#define PCIE_FC_VC0_NP_REQUEST	 MMREG_HALF(PCIE_REG_LSH(5))
#define PCIE_FC_VC0_CPL_LIMIT	 MMREG_WORD(PCIE_REG(6))
#define PCIE_FC_VC0_CPL_CONSUMED MMREG_WORD(PCIE_REG(7))
#define PCIE_FC_VC0_CPL_REQUEST	 MMREG_HALF(PCIE_REG_LSH(8))

#define PCIE_FC_VC1_P_LIMIT	 MMREG_WORD(PCIE_REG(9))
#define PCIE_FC_VC1_P_CONSUMED	 MMREG_WORD(PCIE_REG(10))
#define PCIE_FC_VC1_P_REQUEST	 MMREG_HALF(PCIE_REG_LSH(11))
#define PCIE_FC_VC1_NP_LIMIT	 MMREG_WORD(PCIE_REG(12))
#define PCIE_FC_VC1_NP_CONSUMED	 MMREG_WORD(PCIE_REG(13))
#define PCIE_FC_VC1_NP_REQUEST	 MMREG_HALF(PCIE_REG_LSH(14))
#define PCIE_FC_VC1_CPL_LIMIT	 MMREG_WORD(PCIE_REG(15))
#define PCIE_FC_VC1_CPL_CONSUMED MMREG_WORD(PCIE_REG(16))
#define PCIE_FC_VC1_CPL_REQUEST	 MMREG_HALF(PCIE_REG_LSH(17))

#define PCIE_FC_LIMIT(v,t)	 MMREG_WORD(PCIE_REG(((v)*9)+((t)*3)+0))
#define PCIE_FC_CONSUMED(v,t)	 MMREG_WORD(PCIE_REG(((v)*9)+((t)*3)+1))
#define PCIE_FC_REQUEST(v,t)	 MMREG_HALF(PCIE_REG_LSH(((v)*9)+((t)*3)+2))

#define PCIE_FC_P_LIMIT(v)	 MMREG_WORD(PCIE_REG(((v)*9)+0))
#define PCIE_FC_P_CONSUMED(v)	 MMREG_WORD(PCIE_REG(((v)*9)+1))
#define PCIE_FC_P_REQUEST(v)	 MMREG_HALF(PCIE_REG_LSH(((v)*9)+2))
#define PCIE_FC_NP_LIMIT(v)	 MMREG_WORD(PCIE_REG(((v)*9)+3))
#define PCIE_FC_NP_CONSUMED(v)	 MMREG_WORD(PCIE_REG(((v)*9)+4))
#define PCIE_FC_NP_REQUEST(v)	 MMREG_HALF(PCIE_REG_LSH(((v)*9)+5))
#define PCIE_FC_CPL_LIMIT(v)	 MMREG_WORD(PCIE_REG(((v)*9)+6))
#define PCIE_FC_CPL_CONSUMED(v)	 MMREG_WORD(PCIE_REG(((v)*9)+7))
#define PCIE_FC_CPL_REQUEST(v)	 MMREG_HALF(PCIE_REG_LSH(((v)*9)+8))

#define		FC_DLLP_TYPE		(0xC0000000)
#define			INIT_FC1	(0x40000000)
#define			UPDATE_FC	(0x80000000)
#define			INIT_FC2	(0xC0000000)
#define		FC_HDR_INFINITE		(0x01000000)
#define		FC_HDR			(0x00FF0000)
#define		FC_DATA_INFINITE	(0x00004000)
#define		FC_DATA			(0x00003FFF)


#define	PISR			MMREG_WORD(PCIE_REG(18))
#define	PISR_MASK		MMREG_WORD(PCIE_REG(19))
#define		PCIE_WRITE_FIFO_PARITY_ERROR	(0x00200000)
#define		PCIE_WRITE_FIFO_OVERFLOW_ERROR	(0x00100000)
#define		PCIE_OVERFLOW_ERROR		(0x00080000)
#define		PCIE_POISONED_TLP		(0x00040000)
#define		PCIE_INACTIVE_TAG_ERROR		(0x00020000)
#define		PCIE_LOCKED_COMPLETION_ERROR	(0x00010000)
#define		PCIE_BAD_REQUESTER_ID_ERROR	(0x00008000)
#define		PCIE_MALFORMED_ERROR		(0x00004000)
#define		PCIE_ECRC_ERROR			(0x00002000)
#define		PCIE_BAD_TLP_ERROR		(0x00001000)
#define		PCIE_DLL_PROTOCOL_ERROR		(0x00000800)
#define		PCIE_BAD_DLLP_ERROR		(0x00000400)
#define		PCIE_RECEIVER_ERROR		(0x00000200)
#define		PCIE_VC1_MEMORY_WRITE_RCVD	(0x00000100)
#define		PCIE_VC0_MEMORY_WRITE_RCVD	(0x00000080)
#define		PCIE_GOOD_COMPLETION_RCVD	(0x00000040)
#define		PCIE_DUPLICATE_TLP_WARNING	(0x00000020)
#define		PCIE_RECEIVER_DROP_WARNING	(0x00000010)
#define		PCIE_OTHER_DLLP_RCVD		(0x00000008)
#define		PCIE_NAK_DLLP_RCVD		(0x00000004)
#define		PCIE_ACK_DLLP_RCVD		(0x00000002)
#define		PCIE_FC_DLLP_RCVD		(0x00000001)

#define	PCIE_SEND_DLLP		MMREG_WORD(PCIE_REG(20))

#define	PCIE_FC_UPDATE		MMREG_WORD(PCIE_REG(21))
#define		FC_SET			(0x80000000)
#define		FC_VC			(0x40000000)
#define		FC_NP			(0x20000000)

#define	PCIE_DLL_CONFIG		MMREG_WORD(PCIE_REG(22))
#define		DLL_REG			0xF0000000
#define			DLL_REG_UNBLOCK		0x10000000
#define			DLL_REG_CONTROL		0x20000000
#define			DLL_REG_SEND_FC		0x30000000
#define			DLL_REG_DLLP_ENABLE	0x40000000
#define			DLL_REG_CLEAR_CACHE	0x50000000
#define			DLL_REG_SEND_SEQ	0x60000000
#define			DLL_REG_RECV_SEQ	0x70000000
#define			DLL_REG_ACK_LATENCY	0x80000000
#define			DLL_REG_FC_LATENCY	0x90000000
#define			DLL_REG_WRITE_SPLIT	0xA0000000

#define	DLL_GENERATE_VC0_P_CREDIT	0x20000
#define	DLL_GENERATE_VC0_NP_CREDIT	0x10000
#define	DLL_GENERATE_VC1_P_CREDIT	0x08000
#define	DLL_GENERATE_VC1_NP_CREDIT	0x04000
#define	DLL_FILTER_PASS_ALL_GOOD_DLLP	0x00200
#define	DLL_FILTER_PASS_ALL_GOOD_PM	0x00100
#define	DLL_FILTER_PASS_ALL_GOOD_ACK	0x00080
#define	DLL_FILTER_PASS_ALL_GOOD_FC	0x00040
#define	DLL_FILTER_PASS_ALL_GOOD_VENDOR	0x00020
#define	DLL_FILTER_PASS_SHORT_TLP	0x00010
#define	DLL_FILTER_PASS_REDUNDANT_TLP	0x00008
#define	DLL_FILTER_PASS_OUT_OF_SEQ_TLP	0x00004
#define	DLL_FILTER_DONT_PASS_ALL_TLP	0x00002
#define	DLL_FC_BLOCK_VC0_P_TLP		0x00001

#define PCIE_DLLP_ENABLE(code)	PCIE_DLL_CONFIG = DLL_REG_DLLP_ENABLE | (code)
#define		DLL_CODE_FC_VC2_P			(0x00000000)
#define		DLL_CODE_FC_VC2_NP			(0x00000001)
#define		DLL_CODE_FC_VC2_CPL			(0x00000002)
#define		DLL_CODE_FC_VC3_P			(0x00000003)
#define		DLL_CODE_FC_VC3_NP			(0x00000004)
#define		DLL_CODE_FC_VC3_CPL			(0x00000005)
#define		DLL_CODE_FC_VC4_P			(0x00000006)
#define		DLL_CODE_FC_VC4_NP			(0x00000007)
#define		DLL_CODE_FC_VC4_CPL			(0x00000008)
#define		DLL_CODE_FC_VC5_P			(0x00000009)
#define		DLL_CODE_FC_VC5_NP			(0x0000000A)
#define		DLL_CODE_FC_VC5_CPL			(0x0000000B)
#define		DLL_CODE_FC_VC6_P			(0x0000000C)
#define		DLL_CODE_FC_VC6_NP			(0x0000000D)
#define		DLL_CODE_FC_VC6_CPL			(0x0000000E)
#define		DLL_CODE_FC_VC7_P			(0x0000000F)
#define		DLL_CODE_FC_VC7_NP			(0x00000010)
#define		DLL_CODE_FC_VC7_CPL			(0x00000011)
#define		DLL_CODE_FC_PM_ENTER_L1			(0x00000012)
#define		DLL_CODE_FC_PM_ENTER_L23		(0x00000013)
#define		DLL_CODE_FC_PM_ACTIVE_STATE_REQUEST_L1	(0x00000014)
#define		DLL_CODE_FC_PM_REQUEST_ACK		(0x00000015)
#define		DLL_CODE_FC_VENDOR_SPECIFIC		(0x00000016)
#define		DLL_CODE_FC_RESERVED			(0x00000017)
#define		DLL_CODE_FC(vc,type)			((vc-2)*3+(type))
#define		DLL_CODES				(0x00000018)

#define	PCIE_SEQUENCE		MMREG_HALF(PCIE_REG_MSH(23))
#define	PCIE_ACK_NAK		MMREG_HALF(PCIE_REG_LSH(23))
#define		TLP_NAK			(0x00001000)
#define		TLP_SEQUENCE		(0x00000FFF)

#define PCIE_SEND_CHECKSUM	MMREG_HALF(PCIE_REG_MSH(24))

#define	DISPATCH_STATE		MMREG_WORD(DISPATCH_REG(0))
#define	DISPATCH_INDEX		MMREG_HALF(DISPATCH_REG_MSH(0))
#define DISPATCH_ISR_ON		MMREG_WORD(DISPATCH_REG(1))
#define DISPATCH_ISR_OFF	MMREG_WORD(DISPATCH_REG(2))
#define DISPATCH_STATE_ON	MMREG_WORD(DISPATCH_REG(3))
#define DISPATCH_STATE_OFF	MMREG_WORD(DISPATCH_REG(4))
#define DISPATCH_CONFIG		MMREG_WORD(DISPATCH_REG(5))

#define		DISPATCH_AND		(0x00000100)
#define		DISPATCH_OR		(0x00000080)
#define		DISPATCH_INVERT		(0x00000040)
#define		DISPATCH_SELECT		(0x0000001F)

#define JTAG_MASTER     	MMREG_WORD(JTAG_REG(0))


// PCIE-Port SPD
#define		DESC_DLLP		(0x8000)
#define		DESC_TLP	       	(0x0000)
#define		DESC_USE_VC0_P_CREDIT	(0x4000)
#define		DESC_SPLIT_MWR		(0x1000)
#define		DESC_OFFSET		(0x00FF)


// Z-Port RPD, flags
#define		DESC_BAD		(0xC000)
#define		DESC_BAD_CRC		(0x8000)
#define		DESC_BAD_PHY		(0x4000)

// PCIE-Port RPD
#define		DESC_PCIE_RCODE		(0xF800)
#define		DESC_PCIE_LENGTH	(0x07FF)


// SPD, msw
#define		DESC_PAYLOAD		(0x80000000)
#define		DESC_POINTER		(0x7FFFFFFF)
// SPD, lsw
#define         DESC_INVALIDATE		(0x20000000)
#define         DESC_LAST		(0x10000000)
#define         DESC_CPL		(0x08000000)

// Completion SPD, msw
#define		CPL_CONFIG_ON_CHIP	(0x80000000)
#define		CPL_CONFIG_CANCEL	(0x40000000)
#define		CPL_CONFIG_ADDRESS	(0x0FFFFFFF)
// Completion SPD, lsw
#define         CPL_CONFIG_TAG		(0xF0000000)
#define		CPL_CONFIG_SPD		(0x08000000)
#define         CPL_CONFIG_OFFSET	(0x00FF0000)
#define         CPL_CONFIG_PCI_ADDRESS	(0x0000E000)
#define         CPL_CONFIG_LENGTH	(0x00001FFF)


// Completion Notification, msw
#define CPL_RCODE			(0xF8000000)
#define CPL_OVERFLOW			(0x02000000)
#define CPL_UNDERFLOW			(0x01000000)
#define CPL_POISONED			(0x00800000)
#define CPL_STATUS			(0x00700000)
#define CPL_TAG				(0x000F0000)
#define CPL_CHECKSUM			(0x0000FFFF)


// RCODEs
#define	RCODE_DLLP_PHY_ERROR		0
#define	RCODE_DLLP_3_WORD		1
#define	RCODE_DLLP_OTHER		2
#define	RCODE_TLP_PHY_ERROR		3
#define	RCODE_TLP_EDB			4
#define	RCODE_TLP_3_WORD		5
#define	RCODE_TLP_ERROR			6
#define	RCODE_TLP_DUPLICATE	        7
#define	RCODE_TLP_REG_WRITE_32_0	8
#define	RCODE_TLP_REG_WRITE_64_0	9
#define	RCODE_TLP_REG_WRITE_32_1	10
#define	RCODE_TLP_REG_WRITE_64_1	11
#define RCODE_TLP_REG_WRITE(_64,TD)	(8+_64+(TD*2))
#define	RCODE_TLP_EXT_WRITE_32_0	12
#define	RCODE_TLP_EXT_WRITE_64_0	13
#define	RCODE_TLP_EXT_WRITE_32_1	14
#define	RCODE_TLP_EXT_WRITE_64_1	15
#define RCODE_TLP_EXT_WRITE(_64,TD)	(12+_64+(TD*2))
#define	RCODE_TLP_COMPLETION		16
#define	RCODE_TLP_OTHER			17
#define	RCODE_CPL_END			18
#define	RCODE_CPL_ERROR			19

#define	NUMBER_OF_RCODES		20


#define	MYRINET_MODE		0
#define	MYRINET_SAFE_MODE	1
#define	ETHERNET_MODE		2


#define	REBOOT_ON_SRAM_PARITY_INT		0x01
#define	REBOOT_ON_P0_BUFFER_PARITY_INT		0x02
#define	REBOOT_ON_PCIE_BUFFER_PARITY_INT	0x04
#define	REBOOT_ON_SEND_BUFFER_PARITY_INT	0x08
#define	REBOOT_ON_MEMORY_INT			0x10
#define	REBOOT_ON_TIME3_INT			0x20

#define DEFAULT_CLOCK 0x0d42U

#define	WAIT_1_CYCLE	\
	{		\
	    asm("nop"); \
	}

#define	WAIT_2_CYCLES	\
	{		\
	    asm("nop"); \
	    asm("nop"); \
	}

#define	WAIT_5_CYCLES	\
	{		\
	    asm("nop"); \
	    asm("nop"); \
	    asm("nop"); \
	    asm("nop"); \
	    asm("nop"); \
	}


#define RPD trick gen_prefixed_headers into renaming RPD
#undef RPD
typedef	struct	RPD
{
    unsigned int	skip;
    unsigned short	flags;
    unsigned short	length;
} RPD;


#endif /* LANAI_Z8E_DEF_H */