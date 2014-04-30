#ifndef RF22_h
#define RF22_h

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define PORT_RF22_INT   PORTE
#define DDR_RF22_INT    DDRE
#define DD_RF22_INT     DDE6
#define PCINT_RF22_INT  INT6

#define RING_BUFF_SIZE ( ( 260 * 3 ) + 1 )
//#define RXAF_SIZE 48

#define PORT_RF22_EN	PORTB
#define DDR_RF22_EN	DDRB
#define DD_RF22_EN	DDB4

extern volatile uint8_t rf22_status[3];
extern volatile uint8_t rf22_ring_buff[RING_BUFF_SIZE];
extern volatile uint16_t rf22_ring_buff_head;

void rf22_get_reg_val( const uint8_t rf22_reg, uint8_t* data, uint8_t len );
void rf22_set_reg_val( const uint8_t rf22_reg, const uint8_t* data, uint8_t len );
void rf22_sw_reset( void );
inline void rf22_packet_done( void );
void rf22_init( void );
void rf22_get_status( void );
inline uint8_t rf22_packet_ready( void );
int8_t rf22_send_bulk( uint8_t *ptr );

void rf22_read_reg( const uint8_t idx, uint8_t *data, const uint8_t len );
void rf22_write_reg( const uint8_t idx, const uint8_t *data, const uint8_t len );
void rf22_write_reg_temp( const uint8_t idx, const uint8_t *data, const uint8_t len );


#endif //RF22_h
