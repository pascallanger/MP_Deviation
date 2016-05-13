#ifndef _TARGET_H_
#define _TARGET_H_

#if !defined(ARDUINO_AVR_PRO) && !defined(ARDUINO_AVR_MINI)
	#error You must select the board type "Arduino Pro or Pro Mini" or "Arduino Mini"
#endif
#if F_CPU != 16000000L || !defined(__AVR_ATmega328P__)
	#error You must select the processor type "ATmega328(5V, 16MHz)"
#endif

#define NOP() __asm__ __volatile__("nop")

#ifdef MODULAR
  #define MODULE_CALLTYPE __attribute__((__long_call__))
#else
  #define MODULE_CALLTYPE
#endif

#define FLASHBYTETABLE const uint8_t PROGMEM
#define FLASHWORDTABLE const uint16_t PROGMEM

#define MUSIC_Play(io)

enum {
    CYRF6936,
    A7105,
    CC2500,
    NRF24L01,
    MULTIMOD,
    TX_MODULE_LAST,
};

extern u8 protocol_flags,protocol_flags2;
extern u8 MODULE_ENABLE[TX_MODULE_LAST];
extern u32 MProtocol_id_master;

void init_target(void);
void init_target_ppm(void);
void init_target_serial(void);

uint8_t spi_transfer(u8 val);
uint8_t spi_read3wire(void);

void MCU_SerialNumber(u8 *var, int len);
u32 random_id(u16 adress, u8 create_new);
u32 Crc(const void *buffer, u32 size);

//*******************
//***   Pinouts   ***
//*******************
#define PPM_pin		3								//PPM D3

// SPI Pins
#define SCK_PIN		4								//D4
#define SDI_PIN		5								//D5 - SDIO
#define SDO_PIN		6								//D6
// SPI Controls
#define SCK_on PORTD |= _BV(SCK_PIN)				//PORTD.4
#define SCK_off PORTD &= ~_BV(SCK_PIN)				//PORTD.4
//
#define SDI_SET_INPUT DDRD &= ~_BV(SDI_PIN)			//PORTD.5
#define SDI_SET_OUTPUT DDRD |= _BV(SDI_PIN)			//PORTD.5
#define SDI_on PORTD |= _BV(SDI_PIN)				//PORTD.5
#define SDI_off PORTD &= ~_BV(SDI_PIN)				//PORTD.5
#define SDI_1 (PIND & (1<<SDI_PIN)) == (1<<SDI_PIN)	//PORTD.5
#define SDI_0 (PIND & (1<<SDI_PIN)) == 0x00			//PORTD.5
//
#define SDO_1 (PIND & (1<<SDO_PIN)) == (1<<SDO_PIN)	//PORTD.6
#define SDO_0 (PIND & (1<<SDO_PIN)) == 0x00			//PORTD.6

// CSN Pins
#define A7105_CSN_PIN		2
#define CC2500_CSN_PIN		7
#define NRF24L01_CSN_PIN	8
#define CYRF6936_CSN_PIN	9
#define MULTIMOD_CSN_PIN	0
// CSN Controls
#define A7105_CSN_on PORTD |= _BV(2)				//PORTD.2
#define A7105_CSN_off PORTD &= ~_BV(2)				//PORTD.2
//
#define CC2500_CSN_on PORTD |= _BV(7)				//PORTD.7
#define CC2500_CSN_off PORTD &= ~_BV(7)				//PORTD.7
//
#define NRF24L01_CSN_on PORTB |= _BV(0)				//PORTB.0
#define NRF24L01_CSN_off PORTB &= ~_BV(0)			//PORTB.0
//
#define CYRF6936_CSN_on PORTB |= _BV(1)				//PORTB.1
#define CYRF6936_CSN_off PORTB &= ~_BV(1)			//PORTB.1

// Antenna Pins
#define CTRL1_pin A1
#define CTRL2_pin A2
#define CTRL1 1										//PORTC.1
#define CTRL2 2										//PORTC.2
// Antenna Controls
#define CTRL1_on  PORTC |=  _BV(CTRL1)
#define CTRL1_off PORTC &= ~_BV(CTRL1)
//
#define CTRL2_on  PORTC |=  _BV(CTRL2)
#define CTRL2_off PORTC &= ~_BV(CTRL2)

// Cyrf Pins
#define HAS_CYRF_RESET 0
#define CYRF_RST_PIN A5					//reset pin
// Cyrf Controls
//#define RS_HI PORTC |=  _BV(5)
//#define RS_LO PORTC &= ~_BV(5)

// LED Pin
#define LED_PIN		13
// LED Controls
#define LED_ON		PORTB |= _BV(5)
#define LED_OFF		PORTB &= ~_BV(5)
#define LED_TOGGLE	PORTB ^= _BV(5)
#define LED_SET_OUTPUT	DDRB |= _BV(5)
#define IS_LED_on	( (PORTB & _BV(5)) != 0x00 )

// LED Pin
#define BIND_PIN	13
// LED Controls
#define IS_BIND_on	( (PORTB & _BV(5)) == 0x00 )

// TX
#define TX_ON  PORTD |= _BV(1)
#define TX_OFF  PORTD &= ~_BV(1)
#define TX_TOGGLE  PORTD ^= _BV(1)
#define TX_SET_OUTPUT DDRD |= _BV(1)

#endif
