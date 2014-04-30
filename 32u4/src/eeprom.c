#include <avr/io.h>
#include <avr/eeprom.h>

#include "eeprom.h"

// Stores the given register settings to the eeprom
void eeprom_store_settings( const struct EEPROM_SETTINGS *settings )
{
	eeprom_update_block( (const void *)settings, &rf22_settings, sizeof( rf22_settings ) );
}

// Reads the register settings from the eeprom
void eeprom_read_settings( struct EEPROM_SETTINGS *settings )
{
	eeprom_read_block( (void *)settings, &rf22_settings, sizeof( rf22_settings ) );
}

// Stores the specific setting in eeprom
void eeprom_store_setting( const uint8_t index, const uint8_t *setting, const uint8_t len )
{
	eeprom_update_block( (const void *)setting, &rf22_settings.REG_00_DEVICE_TYPE + index, len );
}

// Reads the specific setting in eeprom
void eeprom_read_setting( const uint8_t index, uint8_t *setting, const uint8_t len )
{
	eeprom_read_block( (void *)setting, &rf22_settings.REG_00_DEVICE_TYPE + index, len );
}

