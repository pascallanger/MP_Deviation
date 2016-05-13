/*********************************************************
					Multiprotocol Tx code
               by Midelic and Pascal Langer(hpnuts)
	http://www.rcgroups.com/forums/showthread.php?t=2165676
    https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/edit/master/README.md

	Thanks to PhracturedBlue, Hexfet, Goebish, Victzh and all protocol developers
				Ported  from deviation firmware 

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
//#define DEBUG_TX

// Deviation includes
#include "common.h"
#include "interface.h"
#include "protospi.h"
#include "config/model.h"
#include "config/tx.h"

//Global constants/variables
uint32_t MProtocol_id;
uint32_t MProtocol_id_master;
uint32_t blink=0;
struct Transmitter Transmitter;

// Protocol variables
uint8_t protocol_select;
uint8_t protocol_flags=0,protocol_flags2=0;
uint8_t main_protocol;
uint8_t sub_protocol;
uint8_t option;
uint8_t RX_num;

// PPM variable
volatile u16 PPM_data[NUM_OUT_CHANNELS];

// Serial variables
volatile uint8_t rx_ok_buff[RXBUFFER_SIZE];
volatile uint8_t tx_buff[TXBUFFER_SIZE];

//Serial protocol
uint8_t cur_protocol[2];

// Telemetry
#define MAX_PKT 27
uint8_t pkt[MAX_PKT];//telemetry receiving packets
#if defined(TELEMETRY)
	#if defined DSM2_CYRF6936_INO
		#define DSM_TELEMETRY	
	#endif
	#if defined FRSKYX_CC2500_INO
		#define SPORT_TELEMETRY	
	#endif
	#if defined FRSKY_CC2500_INO
		#define HUB_TELEMETRY
	#endif
	uint8_t pktt[MAX_PKT];//telemetry receiving packets
	volatile uint8_t tx_head=0;
	volatile uint8_t tx_tail=0;
	uint8_t v_lipo;
	int16_t RSSI_dBm;
	//const uint8_t RSSI_offset=72;//69 71.72 values db
	uint8_t telemetry_link=0; 
	uint8_t telemetry_counter=0;
#endif 

// Instead of complex macros processing of protocol.h we declare protocols
// external symbol here explicitly
const void *(*FUNC_Callback)(enum ProtoCmds cmd);
extern const void *FLYSKY_Cmds(enum ProtoCmds cmd);
extern const void *HUBSAN_Cmds(enum ProtoCmds cmd);
extern const void *SFHSS_Cmds(enum ProtoCmds cmd);
extern const void *Bayang_Cmds(enum ProtoCmds cmd);
extern const void *YD717_Cmds(enum ProtoCmds cmd);
extern const void *HiSky_Cmds(enum ProtoCmds cmd);
extern const void *CX10_Cmds(enum ProtoCmds cmd);
extern const void *CG023_Cmds(enum ProtoCmds cmd);
//extern const void *DSM2_Cmds(enum ProtoCmds cmd);

// Callback to call on timer
u16 (*TIMER_CallBack)(void);

void setup() 
{
	init_target();

	TIMER_CallBack=0;
	// Set servos positions
	for(uint8_t i=0;i<NUM_OUT_CHANNELS;i++)
		PPM_data[i]=1500;
	PPM_data[THROTTLE]=PPM_MIN_100;
	
	//Wait for all RF modules to start
	delay(100);	// 100ms

	//Reset all connected RF modules
	#ifdef	PROTO_HAS_CC2500
		CC2500_Reset();
	#endif
	#ifdef	PROTO_HAS_A7105
		A7105_Reset();
	#endif
	#ifdef	PROTO_HAS_CYRF6936
		CYRF_Reset();
	#endif
	#ifdef	PROTO_HAS_NFR24L01
		NRF24L01_Reset();
	#endif

	// Read status of bind button
	if( IS_BIND_on )
		BIND_BUTTON_FLAG_on;	// If bind button pressed save the status for protocol id reset under hubsan

	// Read status of protocol select binary switch
	// after this protocol_select will be one of {0000, 0001, ..., 1111}
	protocol_select=0x0F - ( ( (PINB>>2)&0x07 ) | ( (PINC<<3)&0x08) );//encoder dip switches 1,2,4,8=>B2,B3,B4,C0
//**********************************
//protocol_select=1;	// here to test PPM
//**********************************

	// Update LED
	LED_OFF;
	LED_SET_OUTPUT;

	// Read or create protocol id
	MProtocol_id_master = random_id(10,false);

	Model.num_channels=16;
	
	//Protocol and interrupts initialization
	if(protocol_select != MODE_SERIAL)
	{ // PPM
		protocol_select--;
		main_protocol	=	PPM_prot[protocol_select].protocol;
		sub_protocol   	=	PPM_prot[protocol_select].sub_proto;
		RX_num			=	PPM_prot[protocol_select].rx_num;
		option			=	PPM_prot[protocol_select].option;
		if(PPM_prot[protocol_select].power)		POWER_FLAG_on;
		if(PPM_prot[protocol_select].autobind)	AUTOBIND_FLAG_on;
		protocol_select++;

		protocol_init();

		//Configure PPM interrupt
		init_target_ppm();
	}
	else
	{ // Serial
		cur_protocol[0]=0;
		cur_protocol[1]=0;
		main_protocol=0;
		init_target_serial();
	}
} 

void loop() 
{
	if(protocol_select==MODE_SERIAL && IS_RX_FLAG_on)	// Serial mode and something has been received
	{
		update_serial_data();	// Update protocol and data
		if(IS_CHANGE_PROTOCOL_FLAG_on)
		{ // Protocol needs to be changed
			LED_OFF;									//led off during protocol init
			protocol_init();							//init new protocol
			CHANGE_PROTOCOL_FLAG_off;					//done
		}
	}
	if(protocol_select!=MODE_SERIAL && IS_PPM_FLAG_on)	// PPM mode and a full frame has been received
	{
		u16 val;
		for(uint8_t i=0;i<NUM_OUT_CHANNELS;i++)
		{ // update servo data without interrupts to prevent bad read in protocols
			cli();	// disable global int
			val=PPM_data[i];
			sei();	// enable global int
			Channels[i]=map(val,PPM_MIN_100,PPM_MAX_100,CHAN_MIN_VALUE,CHAN_MAX_VALUE);
		}
		PPM_FLAG_off;	// wait for next frame before update
	}
	update_led_status();
	update_power_value();
	#if defined(TELEMETRY)
	if( (main_protocol==MODE_FRSKY) || (main_protocol==MODE_HUBSAN) || (main_protocol==MODE_FRSKYX) || (main_protocol==MODE_DSM2) )
		frskyUpdate();
	#endif 
	if (TIMER_CallBack != 0)
		CheckTimer(TIMER_CallBack); 
}

// Update power status based on binding/range/flag
void update_power_value(void)
{
	if(IS_BIND_DONE_on)
	{
		if(IS_RANGE_FLAG_on)
			Model.tx_power=POWER_RANGE;
		else
			Model.tx_power=IS_POWER_FLAG_on?POWER_HIGH:POWER_LOW;
	}
	else
		Model.tx_power=POWER_BIND;
}

// Update led status based on binding and serial
void update_led_status(void)
{
	if(blink<millis())
	{
		if(main_protocol==0)	// No valid serial received at least once
			blink+=BLINK_SERIAL_TIME;					//blink slowly while waiting a valid serial input
		else
			if(TIMER_CallBack == 0)
			{ // Invalid protocol
				if(IS_LED_on)							//flash to indicate invalid protocol
					blink+=BLINK_BAD_PROTO_TIME_LOW;
				else
					blink+=BLINK_BAD_PROTO_TIME_HIGH;
			}
			else
				if(IS_BIND_DONE_on)
				{
					LED_OFF;							//bind completed -> led on
					blink+=1000;						//don't check too often from now on
				}
				else
					blink+=BLINK_BIND_TIME;				//blink fastly during binding
		LED_TOGGLE;
	}
}

// Protocol start
void protocol_init(void)
{
	if(TIMER_CallBack!=0)
		FUNC_Callback(PROTOCMD_RESET);
	TIMER_CallBack = 0;
	FUNC_Callback=0;

	blink=millis();					// Reset LED blink
	
	if(IS_BIND_BUTTON_FLAG_on)
		AUTOBIND_FLAG_on;

	Model.fixed_id	=	RX_num + MProtocol_id_master;
	Transmitter.txid=	Model.fixed_id;
	BIND_DONE;						// The protocol will indicate if bind is needed
	Model.tx_power = POWER_BIND;	// Start with bind power
	
	CTRL1_on;						// NRF24L01 antenna RF3 by default
	CTRL2_off;						// NRF24L01 antenna RF3 by default

	switch(main_protocol)	// Init the requested protocol
	{
#if defined(FLYSKY_A7105_INO)
		case MODE_FLYSKY:
			CTRL1_off;	//antenna RF1
			Model.proto_opts[0]=sub_protocol;
			FUNC_Callback=FLYSKY_Cmds;
			break;
#endif
#if defined(HUBSAN_A7105_INO)
		case MODE_HUBSAN:
			CTRL1_off;	//antenna RF1
			if(IS_BIND_BUTTON_FLAG_on) random_id(10,true);	// Generate new ID if bind button is pressed.
			Model.proto_opts[0]=sub_protocol;
			Model.proto_opts[1]=(s16)option+5795;			// VTX frequency
			Model.proto_opts[2]=0;							// Telemetry enabled
			FUNC_Callback=HUBSAN_Cmds;
			break;
#endif
/*#if defined(FRSKY_CC2500_INO)
		case MODE_FRSKY:
			CTRL1_off;	//antenna RF2
			CTRL2_on;
			next_callback = initFrSky_2way();
			remote_callback = ReadFrSky_2way;
			break;
#endif
#if defined(FRSKYX_CC2500_INO)
		case MODE_FRSKYX:
			CTRL1_off;	//antenna RF2
			CTRL2_on;
			next_callback = initFrSkyX();
			remote_callback = ReadFrSkyX;
			break;
#endif
*/
#if defined(SFHSS_CC2500_INO)
		case MODE_SFHSS:
			CTRL1_off;	//antenna RF2
			CTRL2_on;
			Model.proto_opts[0]=option;			// Fine freq tuning
			Model.proto_opts[1]=SFHSS_COARSE;	// Coarse freq tuning
			FUNC_Callback=SFHSS_Cmds;
			break;
#endif
/*#if defined(DSM2_CYRF6936_INO)
		case MODE_DSM2:
			CTRL2_on;	//antenna RF4
			FUNC_Callback=DSM2_Cmds;
			break;
#endif
#if defined(DEVO_CYRF6936_INO)
		case MODE_DEVO:
			CTRL2_on;	//antenna RF4
			next_callback = DevoInit();
			remote_callback = devo_callback;
			break;
#endif
*/
#if defined(HISKY_NRF24L01_INO)
		case MODE_HISKY:
			Model.proto_opts[0]=sub_protocol;
			FUNC_Callback=HiSky_Cmds;
			break;
#endif
/*
#if defined(V2X2_NRF24L01_INO)
		case MODE_V2X2:
			next_callback = initV2x2();
			remote_callback = ReadV2x2;
			break;
#endif
*/
#if defined(YD717_NRF24L01_INO)
		case MODE_YD717:
			Model.proto_opts[0]=sub_protocol;
			FUNC_Callback=YD717_Cmds;
			break;
#endif
/*
#if defined(KN_NRF24L01_INO)
		case MODE_KN:
			next_callback = initKN();
			remote_callback = kn_callback;
			break;
#endif
#if defined(SYMAX_NRF24L01_INO)
		case MODE_SYMAX:
			next_callback = initSymax();
			remote_callback = symax_callback;
			break;
#endif
#if defined(SLT_NRF24L01_INO)
		case MODE_SLT:
			next_callback=initSLT();
			remote_callback = SLT_callback;
			break;
#endif
*/
#if defined(CX10_NRF24L01_INO)
		case MODE_CX10:
			Model.proto_opts[0]=sub_protocol;
			FUNC_Callback=CX10_Cmds;
			break;
#endif
#if defined(CG023_NRF24L01_INO)
		case MODE_CG023:
			Model.proto_opts[0]=sub_protocol;
			Model.proto_opts[1]=option;			// Dynamic trim
			FUNC_Callback=CG023_Cmds;
			break;
#endif
#if defined(BAYANG_NRF24L01_INO)
		case MODE_BAYANG:
			FUNC_Callback=Bayang_Cmds;
			break;
#endif

/*#if defined(ESKY_NRF24L01_INO)
		case MODE_ESKY:
			next_callback=initESKY();
			remote_callback = ESKY_callback;
			break;
#endif
#if defined(MT99XX_NRF24L01_INO)
		case MODE_MT99XX:
			next_callback=initMT99XX();
			remote_callback = MT99XX_callback;
			break;
#endif
#if defined(MJXQ_NRF24L01_INO)
		case MODE_MJXQ:
			next_callback=initMJXQ();
			remote_callback = MJXQ_callback;
			break;
#endif
#if defined(SHENQI_NRF24L01_INO)
		case MODE_SHENQI:
			next_callback=initSHENQI();
			remote_callback = SHENQI_callback;
			break;
#endif
#if defined(FY326_NRF24L01_INO)
		case MODE_FY326:
			next_callback=initFY326();
			remote_callback = FY326_callback;
			break;
#endif
*/
	}
	if(FUNC_Callback!=0)
		FUNC_Callback(IS_AUTOBIND_FLAG_on?PROTOCMD_BIND:PROTOCMD_INIT);
	BIND_BUTTON_FLAG_off;			// do not bind/reset id anymore even if protocol change
}

void PROTOCOL_SetBindState(u32 msec)
{
	if(msec)
		BIND_IN_PROGRESS;
	else
		BIND_DONE;
}

void update_serial_data(void)
{
	if(rx_ok_buff[0]&0x20)						//check range
		RANGE_FLAG_on;
	else
		RANGE_FLAG_off;		
	if(rx_ok_buff[0]&0xC0)						//check autobind(0x40) & bind(0x80) together
		AUTOBIND_FLAG_on;
	else
		AUTOBIND_FLAG_off;
	if(rx_ok_buff[1]&0x80)						//if rx_ok_buff[1] ==1,power is low ,0-power high
		POWER_FLAG_off;	//power low
	else
		POWER_FLAG_on;	//power high
				
	option=rx_ok_buff[2];

	if( ((rx_ok_buff[0]&0x5F) != (cur_protocol[0]&0x5F)) || ( (rx_ok_buff[1]&0x7F) != cur_protocol[1] ) )
	{ // New model has been selected
		cur_protocol[1] = rx_ok_buff[1]&0x7F;	//store current protocol
		CHANGE_PROTOCOL_FLAG_on;				//change protocol
		main_protocol=rx_ok_buff[0]&0x1F;
		sub_protocol=(rx_ok_buff[1]>>4)& 0x07;	//subprotocol no (0-7) bits 4-6
		RX_num=rx_ok_buff[1]& 0x0F;
	}
	else
		if( ((rx_ok_buff[0]&0x80)!=0) && ((cur_protocol[0]&0x80)==0) )	// Bind flag has been set
			CHANGE_PROTOCOL_FLAG_on;			//restart protocol with bind
	cur_protocol[0] = rx_ok_buff[0];			//store current protocol

	// decode channel values
	volatile uint8_t *p=rx_ok_buff+2;
	uint8_t dec=-3;
	uint16_t val;
	for(uint8_t i=0;i<NUM_OUT_CHANNELS;i++)
	{
		dec+=3;
		if(dec>=8)
		{
			dec-=8;
			p++;
		}
		p++;
		val=(((*((uint32_t *)p))>>dec)&0x7FF);	//value range 0<->2047 -125%<->+125%, 205<->1843 -100%<->+100%
		Channels[i]=(((s32)val)-1024)*49/4;		//value range -12544<->12532 -125%<->+125%, -10032<->+10032 -100%<->+100%
	}
	RX_FLAG_off;								//data has been processed
	Channels[0]=-Channels[0];
	Channels[3]=-Channels[3];

	// Set protocol parameters
	switch(main_protocol)
	{
#if defined(HUBSAN_A7105_INO)
		case MODE_HUBSAN:
			Model.proto_opts[1]=(s16)option+5795;			// VTX frequency
			break;
#endif
#if defined(SFHSS_CC2500_INO)
		case MODE_SFHSS:
			Model.proto_opts[0]=option;			// Fine freq tuning
			//Model.proto_opts[1]=SFHSS_COARSE;	// Coarse freq tuning
			break;
#endif
#if defined(CG023_NRF24L01_INO)
		case MODE_CG023:
			Model.proto_opts[1]=option;			// Dynamic trim
			break;
#endif
	}
}

// Protocol scheduler
void CheckTimer(u16 (*cb)(void))
{ 
	uint16_t next_callback,diff;
	if( (TIFR1 & (1<<OCF1A)) != 0)
	{
		cli();			// disable global int
		OCR1A=TCNT1;	// Callback should already have been called... Use "now" as new sync point.
		sei();			// enable global int
	}
	else
		while((TIFR1 & (1<<OCF1A)) == 0); // wait before callback
	do
	{
		next_callback=cb();
		while(next_callback>4000)
		{ 										// start to wait here as much as we can...
			next_callback=next_callback-2000;
			cli();								// disable global int
			OCR1A+=2000*2;						// set compare A for callback
			TIFR1=(1<<OCF1A);					// clear compare A=callback flag
			sei();								// enable global int
			while((TIFR1 & (1<<OCF1A)) == 0);	// wait 2ms...
		}
		// at this point we have between 2ms and 4ms in next_callback
		cli();									// disable global int
		OCR1A+=next_callback*2;					// set compare A for callback
		TIFR1=(1<<OCF1A);						// clear compare A=callback flag
		diff=OCR1A-TCNT1;						// compare timer and comparator
		sei();									// enable global int
	}
	while(diff&0x8000);	 						// Callback did not took more than requested time for next callback
												// so we can let main do its stuff before next callback
}

void CLOCK_StartTimer(u16 us, u16 (*cb)(void))
{
	if(us>32000)
	{ // us should not be more than 32767 so we will wait here...
		usleep(us-2000);
		us=2000;
	}
	cli();							// disable global int
	OCR1A=TCNT1+us*2;				// set compare A for callback
	sei();							// enable global int
	TIFR1=(1<<OCF1A);				// clear compare A flag
	TIMER_CallBack = cb;
}

void CLOCK_StopTimer()
{
	TIMER_CallBack = 0;
}
