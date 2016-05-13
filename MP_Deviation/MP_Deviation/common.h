/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <stdio.h>

#include "Arduino.h"

#define ctassert(cond, msg)
#define _UNUSED   __attribute__ ((unused))
#define NO_INLINE __attribute__ ((noinline))
#define _tr_noop(x) x

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef uint8_t u8;
typedef short unsigned int u16;	//typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#include "target.h"
#include "telemetry.h"

// from target_defs.h
#define NUM_OUT_CHANNELS 16
#define RXBUFFER_SIZE 25
#define TXBUFFER_SIZE 20
//
extern s32 Channels[NUM_OUT_CHANNELS];
extern volatile u16 PPM_data[NUM_OUT_CHANNELS];
extern volatile uint8_t rx_ok_buff[RXBUFFER_SIZE];
extern volatile uint8_t tx_buff[TXBUFFER_SIZE];

#define CHAN_MAX_VALUE 10000L
#define CHAN_MIN_VALUE -10000L

enum TxPower {
    TXPOWER_100uW,
    TXPOWER_300uW,
    TXPOWER_1mW,
    TXPOWER_3mW,
    TXPOWER_10mW,
    TXPOWER_30mW,
    TXPOWER_100mW,
    TXPOWER_150mW,
    TXPOWER_LAST,
};

u32 rand32_r(u32 *seed, u8 update); //LFSR based PRNG
u32 rand32(); //LFSR based PRNG

void CLOCK_StartTimer(u16 us, u16 (*cb)(void));
void CLOCK_StopTimer();
#define CLOCK_getms() millis()
#define CLOCK_ResetWatchdog()

void PROTOCOL_SetBindState(u32 msec);
void update_led_status(void);
void CheckTimer(u16 (*cb)(void));
void protocol_init(void);
void update_serial_data(void);

#define usleep delayMicroseconds

//******************
// Protocols
//******************
enum PROTOCOLS
{
	MODE_SERIAL = 0,		// Serial commands
	MODE_FLYSKY = 1,		// =>A7105
	MODE_HUBSAN = 2,		// =>A7105
	MODE_FRSKY = 3,			// =>CC2500
	MODE_HISKY = 4,			// =>NRF24L01
	MODE_V2X2 = 5,			// =>NRF24L01
	MODE_DSM2 = 6,			// =>CYRF6936
	MODE_DEVO =7,			// =>CYRF6936
	MODE_YD717 = 8,			// =>NRF24L01
	MODE_KN  = 9,			// =>NRF24L01
	MODE_SYMAX = 10,		// =>NRF24L01
	MODE_SLT = 11,			// =>NRF24L01
	MODE_CX10 = 12,			// =>NRF24L01
	MODE_CG023 = 13,		// =>NRF24L01
	MODE_BAYANG = 14,		// =>NRF24L01
	MODE_FRSKYX = 15,		// =>CC2500
	MODE_ESKY = 16,			// =>NRF24L01
	MODE_MT99XX=17,			// =>NRF24L01
	MODE_MJXQ=18,			// =>NRF24L01
	MODE_SHENQI=19,			// =>NRF24L01
	MODE_FY326=20,			// =>NRF24L01
	MODE_SFHSS=21			// =>CC2500
};

enum Flysky
{
	Flysky=0,
	V9X9=1,
	V6X6=2,
	V912=3
};
enum Hisky
{
	Hisky=0,
	HK310=1
};
enum DSM2{
	DSM2=0,
	DSMX=1
};
enum YD717
{       			
	YD717=0,
	SKYWLKR=1,
	XINXUN=2,
	NIHUI=3,
	SYMAX4=4
};
enum KN
{
	WLTOYS=0,
	FEILUN=1
};
enum SYMAX
{
	SYMAX=0,
	SYMAX5C=1
};
enum CX10
{
    CX10_GREEN = 0,
    CX10_BLUE=1,		// also compatible with CX10-A, CX12
    DM007=2,
	Q282=3,
	JC3015_1=4,
	JC3015_2=5,
	MK33041=6,
	Q242=7
	};
enum CG023
{
    CG023 = 0,
    YD829 = 1,
};
enum MT99XX
{
	MT99	= 0,
	H7		= 1,
	YZ		= 2
};
enum MJXQ
{
	WLH08	= 0,
	X600	= 1,
	X800	= 2,
	H26D	= 3
};

enum FRSKYX
{
	CH_16	= 0,
	CH_8	= 1,
};

#define NONE 		0
#define P_HIGH		1
#define P_LOW		0
#define AUTOBIND	1
#define NO_AUTOBIND	0

struct PPM_Parameters
{
	uint8_t protocol : 5;
	uint8_t sub_proto : 3;
	uint8_t rx_num : 4;
	uint8_t power : 1;
	uint8_t autobind : 1;
	uint8_t option;
};

//Serial flags definition
#define RX_FLAG_on			protocol_flags |= _BV(0)
#define RX_FLAG_off			protocol_flags &= ~_BV(0)
#define IS_RX_FLAG_on		( ( protocol_flags & _BV(0) ) !=0 )
//
#define CHANGE_PROTOCOL_FLAG_on		protocol_flags |= _BV(1)
#define CHANGE_PROTOCOL_FLAG_off		protocol_flags &= ~_BV(1)
#define IS_CHANGE_PROTOCOL_FLAG_on	( ( protocol_flags & _BV(1) ) !=0 )
//
#define POWER_FLAG_on		protocol_flags |= _BV(2)
#define POWER_FLAG_off		protocol_flags &= ~_BV(2)
#define IS_POWER_FLAG_on	( ( protocol_flags & _BV(2) ) !=0 )
//
#define RANGE_FLAG_on		protocol_flags |= _BV(3)
#define RANGE_FLAG_off		protocol_flags &= ~_BV(3)
#define IS_RANGE_FLAG_on	( ( protocol_flags & _BV(3) ) !=0 )
//
#define AUTOBIND_FLAG_on	protocol_flags |= _BV(4)
#define AUTOBIND_FLAG_off	protocol_flags &= ~_BV(4)
#define IS_AUTOBIND_FLAG_on	( ( protocol_flags & _BV(4) ) !=0 )
//
#define BIND_BUTTON_FLAG_on		protocol_flags |= _BV(5)
#define BIND_BUTTON_FLAG_off	protocol_flags &= ~_BV(5)
#define IS_BIND_BUTTON_FLAG_on	( ( protocol_flags & _BV(5) ) !=0 )

//PPM RX OK
#define PPM_FLAG_off			protocol_flags &= ~_BV(6)
#define PPM_FLAG_on			protocol_flags |= _BV(6)
#define IS_PPM_FLAG_on		( ( protocol_flags & _BV(6) ) !=0 )

//Bind flag for blinking
#define BIND_IN_PROGRESS	protocol_flags &= ~_BV(7)
#define BIND_DONE			protocol_flags |= _BV(7)
#define IS_BIND_DONE_on		( ( protocol_flags & _BV(7) ) !=0 )

#define BAD_PROTO_off		protocol_flags2 &= ~_BV(0)
#define BAD_PROTO_on		protocol_flags2 |= _BV(0)
#define IS_BAD_PROTO_on		( ( protocol_flags2 & _BV(0) ) !=0 )

#define BLINK_BIND_TIME	100
#define BLINK_SERIAL_TIME	500
#define BLINK_BAD_PROTO_TIME_LOW	1000
#define BLINK_BAD_PROTO_TIME_HIGH	50

//Power levels
#define POWER_BIND	0
#define POWER_RANGE	1
#define POWER_LOW	5
#define POWER_HIGH	7

//****************************************
//*** MULTI protocol serial definition ***
//****************************************
/*
**************************
16 channels serial protocol
**************************
Serial: 100000 Baud 8e2      _ xxxx xxxx p --
  Total of 26 bytes
  Stream[0]   = 0x55
   header
  Stream[1]   = sub_protocol|BindBit|RangeCheckBit|AutoBindBit;
   sub_protocol is 0..31 (bits 0..4)
   =>	Reserved	0
					Flysky		1
					Hubsan		2
					Frsky		3
					Hisky		4
					V2x2		5
					DSM2		6
					Devo		7
					YD717		8
					KN			9
					SymaX		10
					SLT			11
					CX10		12
					CG023		13
					Bayang		14
					FrskyX		15
					ESky		16
					MT99XX		17
					MJXQ		18
					SHENQI		19
					FY326		20
					SFHSS		21
   BindBit=>		0x80	1=Bind/0=No
   AutoBindBit=>	0x40	1=Yes /0=No
   RangeCheck=>		0x20	1=Yes /0=No
  Stream[2]   = RxNum | Power | Type;
   RxNum value is 0..15 (bits 0..3)
   Type is 0..7 <<4     (bit 4..6)
		sub_protocol==Flysky
			Flysky	0
			V9x9	1
			V6x6	2
			V912	3
		sub_protocol==Hisky
			Hisky	0
			HK310	1
		sub_protocol==DSM2
			DSM2	0
			DSMX	1
		sub_protocol==YD717
			YD717	0
			SKYWLKR	1
			XINXUN	2
			NIHUI	3
			SYMAX4	4
		sub_protocol==KN
			WLTOYS	0
			FEILUN	1
		sub_protocol==SYMAX
			SYMAX	0
			SYMAX5C	1
		sub_protocol==CX10
			CX10_GREEN	0
			CX10_BLUE	1	// also compatible with CX10-A, CX12
			DM007		2
			Q282		3
			JC3015_1	4
			JC3015_2	5
			MK33041		6
			Q242		7
		sub_protocol==CG023
			CG023		0
			YD829		1
		sub_protocol==MT99XX
			MT99		0
			H7			1
			YZ			2
		sub_protocol==MJXQ
			WLH08		0
			X600		1
			X800		2
			H26D		3
		sub_protocol==FRSKYX
			CH_16		0
			CH_8		1
   Power value => 0x80	0=High/1=Low
  Stream[3]   = option_protocol;
   option_protocol value is -127..127
  Stream[4] to [25] = Channels
   16 Channels on 11 bits (0..2047)
	0		-125%
    204		-100%
	1024	   0%
	1843	+100%
	2047	+125%
   Channels bits are concatenated to fit in 22 bytes like in SBUS protocol
*/

#include "_Config.h"

#endif
