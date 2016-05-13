#include "common.h"
#include "target.h"
#include "config/model.h"

u8 MODULE_ENABLE[TX_MODULE_LAST]={ CYRF6936_CSN_PIN, A7105_CSN_PIN, CC2500_CSN_PIN, NRF24L01_CSN_PIN, MULTIMOD_CSN_PIN };

struct Model Model;

s32 Channels[NUM_OUT_CHANNELS];

void init_target()
{
	// General pinout
	DDRD = _BV(SDI_PIN)|_BV(SCK_PIN);		//output
	DDRC = _BV(CTRL1)|_BV(CTRL2); 			//output
	PORTB = _BV(2)|_BV(3)|_BV(4)|_BV(5);	//pullup 10,11,12 and bind button
	PORTC = _BV(0);//A0 high pullup

	// Initialize SPI bus
	SDI_on;
	SCK_off;

	// Initialize CSN pins
	for(int i=0;i<TX_MODULE_LAST;i++)
	{
		pinMode(MODULE_ENABLE[i], OUTPUT);
		digitalWrite(MODULE_ENABLE[i], HIGH);
	}

	// Timer1 config
	TCCR1A = 0;
	TCCR1B = (1 << CS11);	//prescaler8, set timer1 to increment every 0.5us(16Mhz) and start timer
}

void init_target_ppm()
{
	//Configure PPM interrupt
	EICRA |=(1<<ISC11);		// The rising edge of INT1 pin D3 generates an interrupt request
	EIMSK |= (1<<INT1);		// INT1 interrupt enable
#if defined(TELEMETRY)
// Configure serial port for telemetry
	//9600 bauds
	UBRR0H = 0x00;
	UBRR0L = 0x67;
	//Set frame format to 8 data bits, none, 1 stop bit
	UCSR0A = 0 ;			// Clear X2 bit
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B = (1<<TXEN0);	// TX enable
#endif
}

void init_target_serial()
{
	#define BAUD 100000
	#include <util/setbaud.h>	
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	UCSR0A = 0 ;	// Clear X2 bit
	//Set frame format to 8 data bits, even parity, 2 stop bits
	UCSR0C = (1<<UPM01)|(1<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00);
	while ( UCSR0A & (1 << RXC0) )		//Flush receive buffer
		UDR0;
	//enable reception and RC complete interrupt
	UCSR0B = (1<<RXEN0)|(1<<RXCIE0);	//RX enable and interrupt
#ifdef DEBUG_TX
	TX_SET_OUTPUT;
#else
	UCSR0B |= (1<<TXEN0);				//TX enable
#endif
}

#if defined(TELEMETRY)
void Serial_write(uint8_t data)
{
	cli();	// disable global int
	if(++tx_head>=TXBUFFER_SIZE)
		tx_head=0;
	tx_buff[tx_head]=data;
#ifdef XMEGA
	USARTC0.CTRLA = (USARTC0.CTRLA & 0xFC) | 0x01 ;
#else	
	UCSR0B |= (1<<UDRIE0);//enable UDRE interrupt
#endif
	sei();	// enable global int
}
#endif

uint8_t spi_transfer(uint8_t val)
{
	//Software SPI
	uint8_t i;
	uint8_t read=0;

	SCK_off;
	SDI_off;
	for(i=0;i<8;i++)
	{
		read<<=1;
		if(SDO_1)
			read|=0x01;
		if(val&0x80)
			SDI_on;
		else 
			SDI_off;
		SCK_on;
		NOP();
		SCK_off;
		val<<=1;
	}
	SDI_on;
	return read;
}

uint8_t spi_read3wire(void)
{// Used for A7105
	//Software SPI
	uint8_t i;
	uint8_t read=0;

	SCK_off;
	SDI_SET_INPUT;
	for(i=0;i<8;i++)
	{
		read<<=1;
		if(SDI_1)
			read|=0x01;
		SCK_on;
		NOP();
		SCK_off;
		NOP();
	}
	SDI_SET_OUTPUT;
	SDI_on;
	return read;
}

// from misc.c
static u32 rand_seed = 0xb2c54a2ful;
// Linear feedback shift register with 32-bit Xilinx polinomial x^32 + x^22 + x^2 + x + 1
static const u32 LFSR_FEEDBACK = 0x80200003ul;
static const u32 LFSR_INTAP = 32-1;
static void update_lfsr(u32 *lfsr, u8 b)
{
    for (int i = 0; i < 8; ++i) {
        *lfsr = (*lfsr >> 1) ^ ((-(*lfsr & 1u) & LFSR_FEEDBACK) ^ ~((uint32_t)(b & 1) << LFSR_INTAP));
        b >>= 1;
    }
}
u32 rand32_r(u32 *seed, u8 update)
{
    if(! seed)
        seed = &rand_seed;
    update_lfsr(seed, update);
    return *seed;
}
u32 rand32()
{
    return rand32_r(0, 0);
}
//The folloiwng code came from: http://notabs.org/winzipcrc/winzipcrc.c
// C99 winzip crc function, by Scott Duplichan
//We could use the internal CRC implementation in the STM32, but this is really small
//and perfomrance isn't really an issue
u32 Crc(const void *buffer, u32 size)
{
   u32 crc = ~0;
   const u8  *position = (u8 *)buffer;

   while (size--) 
      {
      int bit;
      crc ^= *position++;
      for (bit = 0; bit < 8; bit++) 
         {
         s32 out = crc & 1;
         crc >>= 1;
         crc ^= -out & 0xEDB88320;
         }
      }
   return ~crc;
}

// from tx_misc.c, modified
void MCU_SerialNumber(u8 *var, int len)
{
	int l = len > 16 ? 16 : len;
	u32 id[4];
	u32 seed = 0x4d3ab5d0ul;
	u32 txid = MProtocol_id_master;
	for(int i = 0; i < 4; i++)
		rand32_r(&seed, txid >> (8*i));
	for(int i = 0; i < 4; i++)
		id[i] = rand32_r(&seed, 0x00);
	memcpy(var, &id[0], l);
	return;
}
u32 random_id(u16 adress, u8 create_new)
{
	u32 id;

	if (eeprom_read_byte((uint8_t*)(adress+10))==0xf0 && !create_new)
	{  // TXID exists in EEPROM
		eeprom_read_block((void*)&id,(const void*)adress,4);
	}
	else
	{ // if not generate a random ID
		randomSeed((u32)analogRead(A6)<<10|analogRead(A7));//seed
		//
		id = random(0xfefefefe) + ((uint32_t)random(0xfefefefe) << 16);
		eeprom_write_block((const void*)&id,(void*)adress,4);
		eeprom_write_byte((uint8_t*)(adress+10),0xf0);//write id flag in eeprom.
	}
	return id;
}

/**************************/
/**************************/
/**  Interrupt routines  **/
/**************************/
/**************************/
volatile uint8_t idx = 0;
volatile uint8_t rx_buff[RXBUFFER_SIZE];

//PPM
#ifdef XMEGA
ISR(PORTD_INT0_vect)
#else
ISR(INT1_vect)
#endif
{	// Interrupt on PPM pin
	static int8_t chan=-1;
	static uint16_t Prev_TCNT1=0;
	uint16_t Cur_TCNT1;

#ifdef XMEGA
	Cur_TCNT1 = TCC1.CNT - Prev_TCNT1 ; // Capture current Timer1 value
#else
	Cur_TCNT1=TCNT1-Prev_TCNT1; // Capture current Timer1 value
#endif
	if(Cur_TCNT1<1000)
		chan=-1;				// bad frame
	else
		if(Cur_TCNT1>4840)
		{
			chan=0;				// start of frame
			PPM_FLAG_on;		// full frame present (even at startup since PPM_data has been initialized)
		}
		else
			if(chan!=-1)		// need to wait for start of frame
			{  //servo values between 500us and 2420us will end up here
				uint16_t a = Cur_TCNT1>>1;
				if(a<PPM_MIN) a=PPM_MIN;
				else if(a>PPM_MAX) a=PPM_MAX;
				PPM_data[chan]=a;
				if(chan++>=NUM_OUT_CHANNELS)
					chan=-1;	// don't accept any new channels
			}
	Prev_TCNT1+=Cur_TCNT1;
}

//Serial RX
#ifdef XMEGA
ISR(USARTC0_RXC_vect)
#else
ISR(USART_RX_vect)
#endif
{	// RX interrupt
#ifdef XMEGA
	if((USARTC0.STATUS & 0x1C)==0)			// Check frame error, data overrun and parity error
#else
	if((UCSR0A&0x1C)==0)			// Check frame error, data overrun and parity error
#endif
	{ // received byte is ok to process
		if(idx==0)
		{	// Let's try to sync at this point
#ifdef XMEGA
			if(USARTC0.DATA==0x55)			// If 1st byte is 0x55 it looks ok
#else
			if(UDR0==0x55)			// If 1st byte is 0x55 it looks ok
#endif
			{
				idx++;
#ifdef XMEGA
				TCC1.CCB = TCC1.CNT+(6500L) ;		// Full message should be received within timer of 3250us
				TCC1.INTFLAGS = TC1_CCBIF_bm ;		// clear OCR1B match flag
				TCC1.INTCTRLB = (TCC1.INTCTRLB & 0xF3) | 0x04 ;	// enable interrupt on compare B match
#else
				OCR1B=TCNT1+6500L;		// Full message should be received within timer of 3250us
				TIFR1=(1<<OCF1B);		// clear OCR1B match flag
				TIMSK1 |=(1<<OCIE1B);	// enable interrupt on compare B match
#endif
			}
		}
		else
		{
#ifdef XMEGA
			rx_buff[(idx++)-1]=USARTC0.DATA;	// Store received byte
#else
			rx_buff[(idx++)-1]=UDR0;			// Store received byte
#endif
			if(idx>RXBUFFER_SIZE)
			{	// A full frame has been received
#ifdef XMEGA
				TCC1.INTCTRLB &=0xF3;			// disable interrupt on compare B match
#else
				TIMSK1 &=~(1<<OCIE1B);			// disable interrupt on compare B match
#endif
				if(!IS_RX_FLAG_on)
				{ //Good frame received and main has finished with previous buffer
					for(idx=0;idx<RXBUFFER_SIZE;idx++)
						rx_ok_buff[idx]=rx_buff[idx];	// Duplicate the buffer
					RX_FLAG_on;					// flag for main to process servo data
				}
				idx=0; 							// start again
			}
		}
	}
	else
	{
#ifdef XMEGA
		idx = USARTC0.DATA ;	// Dummy read
#else
		idx=UDR0;	// Dummy read
#endif
		idx=0;		// Error encountered discard full frame...
	}
}

//Serial timer
#ifdef XMEGA
ISR(TCC1_CCB_vect)
#else
ISR(TIMER1_COMPB_vect)
#endif
{	// Timer1 compare B interrupt
	idx=0;
}

#if defined(TELEMETRY)
//Serial TX

#ifdef XMEGA
ISR(USARTC0_DRE_vect)
#else
ISR(USART_UDRE_vect)
#endif
{	// Transmit interrupt
	if(tx_head!=tx_tail)
	{
		if(++tx_tail>=TXBUFFER_SIZE)//head 
			tx_tail=0;
#ifdef XMEGA
		USARTC0.DATA = tx_buff[tx_tail] ;
#else
		UDR0=tx_buff[tx_tail];
#endif
	}
	if (tx_tail == tx_head)
#ifdef XMEGA
		USARTC0.CTRLA &= ~0x03 ;
#else
		UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
#endif
}
#endif
