#include "debug.h"
#include "usb.h"

#include <string.h>

void dbg( const void *data, const uint8_t len )
{
	uint8_t pkt[260] = { 0xFF, 0xFF, 0x00, 0x07, len };
	memcpy( pkt + 5, data, len );
	if( usb_ready( ) )
		usb_send_bulk( pkt, pkt[4] + 5 );
}

void dbg_str( const char *str )
{
	dbg( str, strlen( str ) + 1 );
}

