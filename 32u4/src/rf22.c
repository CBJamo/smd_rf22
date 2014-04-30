#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#include "spi.h"
#include "eeprom.h"
#include "rf22.h"
#include "rf22_priv.h"
#include "debug.h"

volatile uint8_t rf22_status[3];
volatile uint8_t rf22_ring_buff[RING_BUFF_SIZE];
volatile uint16_t rf22_ring_buff_head;
volatile uint16_t rf22_ring_buff_tail;
volatile uint16_t rf22_ring_buff_temp_tail;

static volatile const uint8_t *rf22_send_buff;
static volatile uint16_t rf22_send_buff_head;
static volatile uint8_t rf22_ack_stat;
static volatile uint8_t rf22_ack_sent;
static volatile uint8_t rf22_ack_wait;

volatile uint8_t rf22_cached_hdr[5];
volatile uint8_t rf22_cached_ier[2];

volatile uint8_t rf22_cached_rxaf;
volatile uint8_t rf22_cached_txae;

uint8_t rf22_cached_callsign_len;
char rf22_cached_callsign[7];

static inline void push( uint8_t data );
static inline uint16_t buff_left( void );

const uint8_t rx_mode = RF22_XTON | RF22_RXON | RF22_PLLON;
const uint8_t tx_mode = RF22_XTON | RF22_TXON | RF22_PLLON;
const uint8_t idle_mode = RF22_XTON | RF22_PLLON;

void rf22_dbg( const void *data, uint8_t len )
{
	push( 0xFF );
	push( 0xFF );
	push( 0x00 );
	push( 0x07 );
	push( len );
	while( len-- )
		push( *(uint8_t *)data++ );
	rf22_ring_buff_tail = rf22_ring_buff_temp_tail;
}

inline void rf22_dbg_str( const char *str )
{
	rf22_dbg( str, strlen( str ) );
}

void rf22_read_reg( const uint8_t idx, uint8_t *data, const uint8_t len )
{
	rf22_get_reg_val( idx, data, len );
}

void rf22_write_reg( const uint8_t idx, const uint8_t *data, const uint8_t len )
{
	eeprom_store_setting( idx, data, len );
	rf22_write_reg_temp( idx, data, len );
}

void rf22_write_reg_temp( const uint8_t idx, const uint8_t *data, const uint8_t len )
{
	rf22_set_reg_val( idx, data, len );
}

void rf22_get_reg_val( const uint8_t rf22_reg, uint8_t *data, uint8_t len )
{
	PORT_SS &= ~( 1 << DD_SS );
	spi_fast_shift( rf22_reg );
	switch( len )
	{
	default:
		while( len-- > 5 )
			*data++ = spi_fast_shift( 0xFF );
	case 5:	*data++ = spi_fast_shift( 0xFF );
	case 4:	*data++ = spi_fast_shift( 0xFF );
	case 3:	*data++ = spi_fast_shift( 0xFF );
	case 2:	*data++ = spi_fast_shift( 0xFF );
	case 1:	*data = spi_fast_shift( 0xFF );
	}
	PORT_SS |= ( 1 << DD_SS );
}

void rf22_set_reg_val( const uint8_t rf22_reg, const uint8_t *data, uint8_t len )
{
	PORT_SS &= ~( 1 << DD_SS );
	spi_fast_shift( RF22_SPI_WRITE_MASK | rf22_reg );
	switch( len )
	{
	default:
		while( len-- > 5 )
			spi_fast_shift( *data++ );
	case 5: spi_fast_shift( *data++ );
	case 4: spi_fast_shift( *data++ );
	case 3: spi_fast_shift( *data++ );
	case 2: spi_fast_shift( *data++ );
	case 1: spi_fast_shift( *data );
	}
	PORT_SS |= ( 1 << DD_SS );
}

void rf22_sw_reset( void )
{
	const uint8_t swres = RF22_SWRES;
	rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &swres, 1 );

	_delay_ms( 1 );
}

void rf22_queue_reset( void )
{
	rf22_ring_buff_head = 0;
	rf22_ring_buff_tail = 0;
	rf22_ring_buff_temp_tail = 0;
	rf22_send_buff = 0;
	rf22_send_buff_head = 0;
}

inline void rf22_dint( void )
{
	EIMSK &= ~( 1 << PCINT_RF22_INT );
}

inline void rf22_eint( void )
{
	EIMSK |= ( 1 << PCINT_RF22_INT );
}

inline void rf22_dtmr( void )
{
	TIMSK0 &= ~( 1 << OCIE0A );
}

inline void rf22_etmr( void )
{
	TCNT0 = 0x00;
	TIFR0 = ( 1 << OCF0A );
	TIMSK0 |= ( 1 << OCIE0A );
}

void rf22_init( void )
{
	rf22_dint( );
	// Disable/configure the pin interrupt
	DDR_RF22_INT &= ~( 1 << DD_RF22_INT );
	PORT_RF22_INT |= ( 1 << DD_RF22_INT );
	EICRB = ( 1 << ISC61 );

	// Initialize some variables
	rf22_queue_reset( );
	memset( (void *)rf22_cached_hdr, 0x00, 5 );
	rf22_cached_ier[0] = 0x00;
	rf22_cached_ier[1] = 0x03;
	rf22_cached_rxaf = 0x36;
	rf22_cached_txae = 0x36;
	rf22_ack_stat = 0x00;
	rf22_ack_sent = 0x00;

	// Read the callsign
	eeprom_read_block( rf22_cached_callsign, &system_settings.CALLSIGN, sizeof( system_settings.CALLSIGN ) );
	rf22_cached_callsign_len = strlen( rf22_cached_callsign );

	// Enable the chip
	DDR_RF22_EN |= ( 1 << DD_RF22_EN );
	PORT_RF22_EN &= ~( 1 << DD_RF22_EN );

	/// \note This delay only needs to be 16 by the data sheet, but the
	/// device was not consistently ready until I increased this to about
	/// 30. Oh well, its an init routine. It should only be called once anyway.
	_delay_ms( 30 );

	// SW reset for sanity
	rf22_sw_reset( );

	// debug
	{
		uint8_t dev_id[2];
		rf22_get_reg_val( RF22_REG_00_DEVICE_TYPE, dev_id, 2 );
		//dbg_str( "RESET" );
		//dbg( dev_id, 2 );
	}

	// Set up Timer0 for frame retries
	OCR0A = 0x88;
	TCCR0A = ( 1 << WGM01 );
	TCCR0B = ( 1 << CS00 ) | ( 0 << CS01 ) | ( 1 << CS02 );

	// Enable the pin interrupt
	rf22_eint( );
}

inline uint8_t rf22_packet_ready( void )
{
	return ( rf22_ring_buff_head != rf22_ring_buff_tail );
}

inline void rf22_packet_done( void )
{
	// Move headptr up by length of the packet
	rf22_ring_buff_head = ( rf22_ring_buff_head + ( rf22_ring_buff[( rf22_ring_buff_head + 4 ) % RING_BUFF_SIZE] + 5 ) ) % RING_BUFF_SIZE;
}

int8_t rf22_send_bulk( uint8_t *ptr )
{
	uint8_t len = 64; // Only first fragment

	if( rf22_send_buff != 0 )
	{
		if( rf22_send_buff_head == 0 )
		{
			rf22_send_buff = 0;
			return 0; // Finished
		}
		else
			return -1; // In Progress
	}

	rf22_dint( );

	// Check for "carrier" (RX)
	if( rf22_status[2] & RF22_IPREAVAL || rf22_status[2] & RF22_ISWDET || rf22_status[1] & RF22_IRXFFAFULL || rf22_ring_buff_tail != rf22_ring_buff_temp_tail || rf22_ack_sent != 0x00 )
	{
		rf22_eint( );
		return -2; // Try again later
	}

	// Go idle (so we won't RX)
	rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &idle_mode, 1 );

	memcpy( &ptr[5 + ptr[4]], rf22_cached_callsign, rf22_cached_callsign_len );
	ptr[4] += rf22_cached_callsign_len;
	ptr[2] = rf22_cached_callsign_len;
	if( ptr[4] < 64 )
		len = ptr[4];

	rf22_send_buff = ptr;

	// Set header
	rf22_set_reg_val( RF22_REG_3A_TRANSMIT_HEADER3, &ptr[0], 5 );
	memcpy( (void *)&rf22_cached_hdr[0], &ptr[0], 5 );
	rf22_send_buff_head = 5;

	// Send first fragment
	PORT_SS &= ~( 1 << DD_SS );
	spi_fast_shift( RF22_SPI_WRITE_MASK | RF22_REG_7F_FIFO_ACCESS );
	while( len-- )
		spi_fast_shift( rf22_send_buff[rf22_send_buff_head++] );
	PORT_SS |= ( 1 << DD_SS );

	// We will now wait for ack
	rf22_ack_stat = 1;
	rf22_dtmr( );

	// Start TX
	rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &tx_mode, 1 );
	rf22_eint( );

	return -1; // In Progress
}

static inline void push( const uint8_t data )
{
	rf22_ring_buff[ rf22_ring_buff_temp_tail++ ] = data;
	rf22_ring_buff_temp_tail %= RING_BUFF_SIZE;
}

static inline uint16_t buff_left( void )
{
	return ( ( ( RING_BUFF_SIZE + rf22_ring_buff_head ) - rf22_ring_buff_tail ) % RING_BUFF_SIZE ) - 1;
}

ISR(INT6_vect)
{
	// Update our status
	rf22_get_reg_val( RF22_REG_02_DEVICE_STATUS, (uint8_t *)rf22_status, 3 );

	//if( rf22_ring_buff_tail == rf22_ring_buff_temp_tail )
	//	rf22_dbg( (uint8_t *)rf22_status, 3 );

	if( rf22_status[2] & RF22_ICHIPRDY ) // Chip Ready
	{
		struct EEPROM_SETTINGS settings;

		// Read settings from EEPROM
		eeprom_read_settings( &settings );

		// Sneak in our callsign length
		settings.REG_41_CHECK_HEADER1 = rf22_cached_callsign_len;

		// Send settings to RF22
		rf22_get_reg_val( RF22_REG_05_INTERRUPT_ENABLE1, &settings.REG_05_INTERRUPT_ENABLE1, 12 );
		rf22_set_reg_val( RF22_REG_12_TEMPERATURE_SENSOR_CALIBRATION, &settings.REG_12_TEMPERATURE_SENSOR_CALIBRATION, 5);
		rf22_set_reg_val( RF22_REG_19_LDC_MODE_DURATION, &settings.REG_19_LDC_MODE_DURATION, 2);
		rf22_set_reg_val( RF22_REG_1C_IF_FILTER_BANDWIDTH, &settings.REG_1C_IF_FILTER_BANDWIDTH, 10);
		rf22_set_reg_val( RF22_REG_27_RSSI_THRESHOLD, &settings.REG_27_RSSI_THRESHOLD, 1);
		rf22_set_reg_val( RF22_REG_2A_AFC_LIMITER, &settings.REG_2A_AFC_LIMITER, 1);
		rf22_set_reg_val( RF22_REG_2C_OOK_COUNTER_VALUE_1, &settings.REG_2C_OOK_COUNTER_VALUE_1, 3);
		rf22_set_reg_val( RF22_REG_30_DATA_ACCESS_CONTROL, &settings.REG_30_DATA_ACCESS_CONTROL, 1);
		rf22_set_reg_val( RF22_REG_32_HEADER_CONTROL1, &settings.REG_32_HEADER_CONTROL1, 21);
		rf22_set_reg_val( RF22_REG_58_CHARGE_PUMP_CURRENT_TRIMMING, &settings.REG_58_CHARGE_PUMP_CURRENT_TRIMMING, 1);
		rf22_set_reg_val( RF22_REG_60_CHANNEL_FILTER_COEFFICIENT_ADDRESS, &settings.REG_60_CHANNEL_FILTER_COEFFICIENT_ADDRESS, 1);
		rf22_set_reg_val( RF22_REG_62_CRYSTAL_OSCILLATOR_POR_CONTROL, &settings.REG_62_CRYSTAL_OSCILLATOR_POR_CONTROL, 1);
		rf22_set_reg_val( RF22_REG_69_AGC_OVERRIDE1, &settings.REG_69_AGC_OVERRIDE1, 1);
		rf22_set_reg_val( RF22_REG_6D_TX_POWER, &settings.REG_6D_TX_POWER, 11);
		rf22_set_reg_val( RF22_REG_79_FREQUENCY_HOPPING_CHANNEL_SELECT, &settings.REG_79_FREQUENCY_HOPPING_CHANNEL_SELECT, 2);
		rf22_set_reg_val( RF22_REG_7C_TX_FIFO_CONTROL1, &settings.REG_7C_TX_FIFO_CONTROL1, 3);

		// Sync our cached vars
		memcpy( (void *)rf22_cached_hdr, &settings.REG_3A_TRANSMIT_HEADER3, 5 );
		rf22_cached_ier[0] = settings.REG_05_INTERRUPT_ENABLE1;
		rf22_cached_ier[1] = settings.REG_06_INTERRUPT_ENABLE2;
		rf22_cached_rxaf = settings.REG_7E_RX_FIFO_CONTROL + 1;
		rf22_cached_txae = 65 - settings.REG_7D_TX_FIFO_CONTROL2;

		// Some initialization
		rf22_queue_reset( );

		// Enable the following interrupts:
		// - RXFFAFULL: RX FIFO Almost Full
		// - PKVALID: Valid packet received
		// - CRCERROR: CRC Error
		// - FFERR: FIFO overflow/underflow
		// - TXFFAEM: TX FIFO Almost Empty
		// - PKSENT: TX Packet Successfully Sent
		//
		// - SWDET: Sync Word Detect
		// - PREAVAL: Valid preamble detected
		rf22_cached_ier[0] |= ( RF22_ENCRCERROR | RF22_ENRXFFAFULL | RF22_ENPKVALID | RF22_ENFFERR | RF22_ITXFFAEM | RF22_ENPKSENT );
		rf22_cached_ier[1] |= ( RF22_ENSWDET | RF22_ENPREAVAL );
		rf22_set_reg_val( RF22_REG_05_INTERRUPT_ENABLE1, (const uint8_t *)rf22_cached_ier, 2 );

		// Debug
		//{
		//	rf22_get_reg_val( RF22_REG_02_DEVICE_STATUS, (uint8_t *)rf22_status, 1 );
		//	if( rf22_status[0] & RF22_FREQERR )
		//		rf22_dbg_str( "Frequency Error" );
		//}

		// Seed rand with our MAC
		srand( settings.REG_3F_CHECK_HEADER3 );

		PORTB |= ( 1 << 0 ); // LED

		return;
	}

	if( rf22_status[2] & RF22_ISWDET ) // Sync Word Detect
	{
		// Disable the following interrupts:
		// - PREAINVAL: Invalid preamble detected
		rf22_cached_ier[1] &= ~RF22_ENPREAINVAL;
		rf22_set_reg_val( RF22_REG_06_INTERRUPT_ENABLE2, (const uint8_t *)&rf22_cached_ier[1], 1 );

		rf22_ring_buff_temp_tail = rf22_ring_buff_tail;
	}
	else if( rf22_status[2] & RF22_IPREAVAL ) // Valid preamble detected
	{
		// Enable the following interrupts:
		// - PREAINVAL: Invalid preamble detected
		rf22_cached_ier[1] |= RF22_ENPREAINVAL;
		rf22_set_reg_val( RF22_REG_06_INTERRUPT_ENABLE2, (const uint8_t *)&rf22_cached_ier[1], 1 );
	}
	else if( rf22_status[2] & RF22_IPREAINVAL && rf22_cached_ier[1] & RF22_ENPREAINVAL ) // Invalid Preamble Detected
	{
		// Disable the following interrupts:
		// - PREAINVAL: Invalid preamble detected
		rf22_cached_ier[1] &= ~RF22_ENPREAINVAL;
		rf22_set_reg_val( RF22_REG_06_INTERRUPT_ENABLE2, (const uint8_t *)&rf22_cached_ier[1], 1 );
	}

	if( rf22_status[1] & RF22_ICRCERROR ) // CRC Error
	{
		uint8_t tmp;

		// Process this as an RX error
		rf22_ring_buff_temp_tail = rf22_ring_buff_tail;

		// Enter idle mode
		rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &idle_mode, 1 );

		// Flush the RX FIFO
		rf22_get_reg_val( RF22_REG_08_OPERATING_MODE2, &tmp, 1 );
		//tmp |= RF22_FFCLRRX;
		tmp |= RF22_FFCLRRX | RF22_FFCLRTX;
		rf22_set_reg_val( RF22_REG_08_OPERATING_MODE2, &tmp, 1 );
		//tmp &= ~RF22_FFCLRRX;
		tmp &= ~( RF22_FFCLRRX | RF22_FFCLRTX );
		rf22_set_reg_val( RF22_REG_08_OPERATING_MODE2, &tmp, 1 );

		// Re-enter RX mode
		rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &rx_mode, 1 );

		rf22_dbg( (uint8_t *)rf22_status, 3 );

		// Invalidate some other flags for this interrupt
		rf22_status[1] &= ~( RF22_IRXFFAFULL | RF22_ITXFFAEM | RF22_ITXFFAFULL | RF22_IPKVALID );
		rf22_status[2] &= ~( RF22_ISWDET | RF22_IPREAVAL );

		rf22_dbg_str( "CRC Error" );
		PORTB |= ( 1 << 0 ); // LED
	}
	else if( rf22_status[1] & RF22_IFFERROR ) // FIFO overflow/underflow
	{
		uint8_t tmp;

		// Enter idle mode
		rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &idle_mode, 1 );

		// Clear both FIFOs
		rf22_get_reg_val( RF22_REG_08_OPERATING_MODE2, &tmp, 1 );
		tmp |= RF22_FFCLRRX | RF22_FFCLRTX;
		rf22_set_reg_val( RF22_REG_08_OPERATING_MODE2, &tmp, 1 );
		tmp &= ~( RF22_FFCLRRX | RF22_FFCLRTX );
		rf22_set_reg_val( RF22_REG_08_OPERATING_MODE2, &tmp, 1 );

		// Re-enter RX mode (if in TX, this attempt is aborted)
		rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &rx_mode, 1 );

		// If we were sending an ACK, abort that
		rf22_ack_sent = 0;

		// This is a little aggressive, but we'll try it
		{
			rf22_dtmr( );
			rf22_send_buff_head = 0;
			rf22_ack_stat = 0;
		}

		// Abort RX
		tmp = rf22_ring_buff_temp_tail;
		rf22_ring_buff_temp_tail = rf22_ring_buff_tail;

		if( !( rf22_status[0] & RF22_CPS_TX ) )
		{
			PORTB |= ( 1 << 0 ); // LED

			rf22_dbg( (uint8_t *)rf22_status, 3 );
			rf22_dbg( (uint8_t *)&rf22_send_buff_head, 1 );
			rf22_dbg( (uint8_t *)&tmp, 1 );
			rf22_dbg( (uint8_t *)&rf22_ring_buff_temp_tail, 1 );
			rf22_dbg_str( "Possible RX Buffer Error" );

			PORTF &= ~( 1 << 7 );
		}
		else if( !( rf22_status[0] & RF22_CPS_RX ) )
		{
			rf22_dbg( (uint8_t *)rf22_status, 3 );
			rf22_dbg_str( "Possible TX Buffer Error" );
		}

		// Invalidate some other flags for this interrupt
		rf22_status[1] &= ~( RF22_IRXFFAFULL | RF22_ITXFFAEM | RF22_ITXFFAFULL | RF22_IPKVALID );
		rf22_status[2] &= ~( RF22_ISWDET | RF22_IPREAVAL );

	}
	else if( rf22_status[1] & RF22_IPKVALID && rf22_cached_ier[0] & RF22_ENPKVALID ) // Valid packet RX complete
	{
		uint8_t len;

		rf22_status[1] &= ~RF22_IRXFFAFULL;

		if ( rf22_ring_buff_tail == rf22_ring_buff_temp_tail ) // Check if we have the header 
		{
			PORTB &= ~( 1 << 0 ); // LED

			rf22_get_reg_val( RF22_REG_4B_RECEIVED_PACKET_LENGTH, &len, 1 );

			if( len + 5 > buff_left( ) )
			{
				// Buffer Full, Abort
				dbg_str( "PKVALID Abort" );

				len = 0;

				PORTF &= ~( 1 << 7 );
			}
			else
			{
				PORT_SS &= ~( 1 << DD_SS );
				spi_fast_shift( RF22_REG_47_RECEIVED_HEADER3 );
				push( spi_fast_shift( 0xFF ) );
				push( spi_fast_shift( 0xFF ) );
				push( spi_fast_shift( 0xFF ) );
				push( spi_fast_shift( 0xFF ) );
				PORT_SS |= ( 1 << DD_SS );
				push( len );
			}
		}
		else
			len = ( rf22_ring_buff[( rf22_ring_buff_tail + 4 ) % RING_BUFF_SIZE] + 5 ) - ( ( ( RING_BUFF_SIZE + rf22_ring_buff_temp_tail ) - rf22_ring_buff_tail ) % RING_BUFF_SIZE );

		if( len )
		{
	 		PORT_SS &= ~( 1 << DD_SS );
			spi_fast_shift( RF22_REG_7F_FIFO_ACCESS );
			while( len-- )
				push( spi_fast_shift( 0xFF ) );
			PORT_SS |= ( 1 << DD_SS );
		}

		// RX is complete, update the tailptr
		len = rf22_ring_buff[( rf22_ring_buff_tail + 2 ) % RING_BUFF_SIZE];
		rf22_ring_buff[( rf22_ring_buff_tail + 2 ) % RING_BUFF_SIZE] = 0x00;
		rf22_ring_buff[( rf22_ring_buff_tail + 4 ) % RING_BUFF_SIZE] -= len;

		rf22_ring_buff_temp_tail = ( RING_BUFF_SIZE + rf22_ring_buff_temp_tail - len ) % RING_BUFF_SIZE;

                // Check if this is a datalink packet
                if( rf22_ring_buff[rf22_ring_buff_tail] == rf22_cached_hdr[1] && 
			rf22_ring_buff[( rf22_ring_buff_tail + 3 ) % RING_BUFF_SIZE] == 0x00 &&
			rf22_ring_buff[( rf22_ring_buff_tail + 4 ) % RING_BUFF_SIZE] >= 0x01 )
		{
			// Do not forward the packet
			rf22_ring_buff_temp_tail = rf22_ring_buff_tail;

			switch( rf22_ring_buff[( rf22_ring_buff_tail + 5 ) % RING_BUFF_SIZE] )
			{
			case 0x01: // ACK
				// Move the send buffer head to indicate packet send complete
				if( rf22_ack_stat != 0 )
				{
					rf22_dtmr( );
					rf22_send_buff_head = 0;
					rf22_ack_stat = 0;
					/*{
						const uint8_t tmp = 0xFF;
						rf22_dbg( &tmp, 1 );
					}*/
				}
				break;
			default: // ...?
				{
					const uint8_t tmp[7] = {
						0xEE,
						rf22_ring_buff[rf22_ring_buff_tail],
						rf22_ring_buff[( rf22_ring_buff_tail + 1 ) % RING_BUFF_SIZE],
						rf22_ring_buff[( rf22_ring_buff_tail + 2 ) % RING_BUFF_SIZE],
						rf22_ring_buff[( rf22_ring_buff_tail + 3 ) % RING_BUFF_SIZE],
						rf22_ring_buff[( rf22_ring_buff_tail + 4 ) % RING_BUFF_SIZE],
						rf22_ring_buff[( rf22_ring_buff_tail + 5 ) % RING_BUFF_SIZE]
					};
					rf22_dbg( tmp, 7 );
				}
				break;
			}

			// Re-enter RX mode
			rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &rx_mode, 1 );
		}
		// Check if it is unicast for us (and send ACK)
		else if( rf22_ring_buff[rf22_ring_buff_tail] == rf22_cached_hdr[1] )
		{
			unsigned char idx = 0;
			rf22_cached_hdr[0] = rf22_ring_buff[( rf22_ring_buff_tail + 1 ) % RING_BUFF_SIZE];
			rf22_cached_hdr[2] = rf22_cached_callsign_len;
			rf22_cached_hdr[3] = 0x00;
			rf22_cached_hdr[4] = rf22_cached_callsign_len + 1;

			// Go idle for kicks
			rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &idle_mode, 1 );

			// Set header
			rf22_set_reg_val( RF22_REG_3A_TRANSMIT_HEADER3, (uint8_t *)&rf22_cached_hdr[0], 5 );

			// Set payload (incl. callsign)
			PORT_SS &= ~( 1 << DD_SS );
			spi_fast_shift( RF22_SPI_WRITE_MASK | RF22_REG_7F_FIFO_ACCESS );
			spi_fast_shift( 0x01 ); // PAYLOAD
			while( idx < rf22_cached_callsign_len )
				spi_fast_shift( rf22_cached_callsign[idx++] );
			PORT_SS |= ( 1 << DD_SS );

			// Start TX
			rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &tx_mode, 1 );

			rf22_ack_sent = 0x01;

			/*{
				const uint8_t tmp[14] = {
					0xCC,
					rf22_ring_buff[rf22_ring_buff_tail],
					rf22_ring_buff[( rf22_ring_buff_tail + 1 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 2 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 3 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 4 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 5 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 6 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 7 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 8 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 9 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 10 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 11 ) % RING_BUFF_SIZE],
					rf22_ring_buff[( rf22_ring_buff_tail + 12 ) % RING_BUFF_SIZE]
				};
				rf22_dbg( tmp, 14 );
			}*/

			// Make the packet available
			rf22_ring_buff_tail = rf22_ring_buff_temp_tail;
		}
		else // Not datalink or unicast...must be broadcast
		{
			// Re-enter RX mode
			rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &rx_mode, 1 );

			// Make the packet available
			rf22_ring_buff_tail = rf22_ring_buff_temp_tail;
		}

		PORTB |= ( 1 << 0 ); // LED
	}
	else if( rf22_status[1] & RF22_IRXFFAFULL && rf22_cached_ier[0] & RF22_ENRXFFAFULL ) // RX FIFO Almost Full
	{
		uint8_t len;

		if ( rf22_ring_buff_tail == rf22_ring_buff_temp_tail ) // Check if we have the header 
		{
			PORTB &= ~( 1 << 0 ); // LED

			rf22_get_reg_val( RF22_REG_4B_RECEIVED_PACKET_LENGTH, &len, 1 );

			if( len + 5 > buff_left( ) )
			{
				// Buffer Full, Abort
				// Process this as an RX error

				dbg_str( "RXFFAFULL Abort" );

				// Enter idle mode
				//rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &idle_mode, 1 );

				// Re-enter RX mode
				//rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &rx_mode, 1 );

				len = 0;

				PORTB |= ( 1 << 0 ); // LED

				PORTF &= ~( 1 << 7 );
			}
			else
			{
				PORT_SS &= ~( 1 << DD_SS );
				spi_fast_shift( RF22_REG_47_RECEIVED_HEADER3 );
				push( spi_fast_shift( 0xFF ) );
				push( spi_fast_shift( 0xFF ) );
				push( spi_fast_shift( 0xFF ) );
				push( spi_fast_shift( 0xFF ) );
				PORT_SS |= ( 1 << DD_SS );
				push( len );
			}
		}
		else
			len = ( rf22_ring_buff[( rf22_ring_buff_tail + 4 ) % RING_BUFF_SIZE] + 5 ) - ( ( ( RING_BUFF_SIZE + rf22_ring_buff_temp_tail ) - rf22_ring_buff_tail ) % RING_BUFF_SIZE );
		if( len > rf22_cached_rxaf )
			len = rf22_cached_rxaf;

		if( len )
		{
	 		PORT_SS &= ~( 1 << DD_SS );
			spi_fast_shift( RF22_REG_7F_FIFO_ACCESS );
			while( len-- )
				push( spi_fast_shift( 0xFF ) );
			PORT_SS |= ( 1 << DD_SS );
		}
	}

	if( rf22_status[1] & RF22_IPKSENT && rf22_cached_ier[0] & RF22_ENPKSENT )
	{
		// If it was not unicast, we're done, so move the send buffer
		// head to indicate packet send complete. Same for datalink.
		if( rf22_ack_sent || rf22_cached_hdr[3] == 0x00 )
			rf22_ack_sent = 0x00;
		else if( rf22_cached_hdr[0] == 0x00 || rf22_cached_hdr[0] == 0xFF )
		{
			rf22_send_buff_head = 0;
			rf22_ack_stat = 0;
		}
		else // wait for ack or timeout
		{
			if( rf22_ack_stat < 1 ) // SHOULD NOT HAPPEN
				rf22_ack_wait = 1;
			else
				rf22_ack_wait = 1 + rand( ) % ( (int)pow( 2, rf22_ack_stat ) - 1 );
			rf22_etmr( );
		}

		// Re-enter RX mode
		rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &rx_mode, 1 );

		rf22_ring_buff_temp_tail = rf22_ring_buff_tail;
	}
	else if( rf22_status[1] & RF22_ITXFFAEM && rf22_cached_ier[0] & RF22_ENTXFFAEM )
	{
		uint8_t len = ( rf22_send_buff[4] + 5 ) - rf22_send_buff_head;
		if( len > rf22_cached_txae )
			len = rf22_cached_txae;

 		PORT_SS &= ~( 1 << DD_SS );
		spi_fast_shift( RF22_SPI_WRITE_MASK | RF22_REG_7F_FIFO_ACCESS );
		while( len-- )
			spi_fast_shift( rf22_send_buff[rf22_send_buff_head++] );
		PORT_SS |= ( 1 << DD_SS );
	}
}

ISR( TIMER0_COMPA_vect )
{
	if( rf22_ack_wait > 0 )
	{
		rf22_ack_wait--;
	}
	else if( rf22_ack_stat == 0 )
	{
		// Not supposed to be here
		rf22_dtmr( );
	}
	else if( rf22_ack_stat > 2 ) // GIVE UP
	{
		// TODO: Error
		rf22_dtmr( );
		rf22_send_buff_head = 0;
		rf22_ack_stat = 0;
		/*{
			const uint8_t tmp[2] = { 0xFD, rf22_ack_wait };
			rf22_dbg( (void *)tmp, 2 );
		}*/
	}
	else
	{
		uint8_t len;

		// Check for "carrier" (RX)
		if( rf22_status[2] & RF22_IPREAVAL || rf22_status[2] & RF22_ISWDET || rf22_status[1] & RF22_IRXFFAFULL || rf22_ack_sent != 0 )
		{
			rf22_ack_stat++;
			/*{
				const uint8_t tmp[2] = { 0xFE, rf22_ack_stat };
				rf22_dbg( (void *)tmp, 2 );
			}*/
			return;
		}

		// Go idle (so we won't RX)
		rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &idle_mode, 1 );

		// Set header
		rf22_set_reg_val( RF22_REG_3A_TRANSMIT_HEADER3, (void *)&rf22_send_buff[0], 5 );
		memcpy( (void *)&rf22_cached_hdr[0], (void *)&rf22_send_buff[0], 5 );
		rf22_send_buff_head = 5;

                len = rf22_send_buff[4] > 64 ? 64 : rf22_send_buff[4];

		// Re-send first fragment
		PORT_SS &= ~( 1 << DD_SS );
		spi_fast_shift( RF22_SPI_WRITE_MASK | RF22_REG_7F_FIFO_ACCESS );
		while( len-- )
			spi_fast_shift( rf22_send_buff[rf22_send_buff_head++] );
		PORT_SS |= ( 1 << DD_SS );

		// We will now wait for ack
		rf22_ack_stat++;
		/*{
			const uint8_t tmp[2] = { 0xFF, rf22_ack_stat };
			rf22_dbg( (void *)tmp, 2 );
		}*/

		// Start TX
		rf22_set_reg_val( RF22_REG_07_OPERATING_MODE1, &tx_mode, 1 );
	}
}
