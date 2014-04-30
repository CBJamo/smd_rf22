#include <avr/io.h>

#include "pll.h"

void pll_start( void )
{
	PLLFRQ = ( 1 << PDIV2 );

	#if F_CPU == 8000000UL
	PLLCSR &= ~( 1 << PINDIV );
	#elif F_CPU == 16000000UL
	PLLCSR |= ( 1 << PINDIV );
	#else
	#error Invalid F_CPU
	#endif

	PLLCSR |= ( 1 << PLLE );
	while( !( PLLCSR & ( 1 << PLOCK ) ) );
}

void pll_stop( void )
{
	PLLCSR &= ~( 1 << PLLE );
	while( PLLCSR & ( 1 << PLOCK ) );
}
