#include "eeprom.h"

struct EEPROM_SETTINGS EEMEM rf22_settings =
{
	// 12
	.REG_05_INTERRUPT_ENABLE1                   = 0x00,
	.REG_06_INTERRUPT_ENABLE2                   = 0x03,
	.REG_07_OPERATING_MODE1                     = 0x01,
	.REG_08_OPERATING_MODE2                     = 0x00,
	.REG_09_OSCILLATOR_LOAD_CAPACITANCE         = 0x7F,
	.REG_0A_UC_OUTPUT_CLOCK                     = 0x06,
	.REG_0B_GPIO_CONFIGURATION0                 = 0x12,
	.REG_0C_GPIO_CONFIGURATION1                 = 0x15,
	.REG_0D_GPIO_CONFIGURATION2                 = 0x00,
	.REG_0E_IO_PORT_CONFIGURATION               = 0x00,
	.REG_0F_ADC_CONFIGURATION                   = 0x00,
	.REG_10_ADC_SENSOR_AMP_OFFSET               = 0x00,

	// 5
	.REG_12_TEMPERATURE_SENSOR_CALIBRATION      = 0x20,
	.REG_13_TEMPERATURE_VALUE_OFFSET            = 0x00,
	.REG_14_WAKEUP_TIMER_PERIOD1                = 0x03,
	.REG_15_WAKEUP_TIMER_PERIOD2                = 0x00,
	.REG_16_WAKEUP_TIMER_PERIOD3                = 0x01,

	// 2
	.REG_19_LDC_MODE_DURATION                   = 0x01,
	.REG_1A_LOW_BATTERY_DETECTOR_THRESHOLD      = 0x14,

	// 10
	.REG_1C_IF_FILTER_BANDWIDTH                 = 0x01,
	.REG_1D_AFC_LOOP_GEARSHIFT_OVERRIDE         = 0x44,
	.REG_1E_AFC_TIMING_CONTROL                  = 0x0A,
	.REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE   = 0x03,
	.REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE    = 0x64,
	.REG_21_CLOCK_RECOVERY_OFFSET2              = 0x01,
	.REG_22_CLOCK_RECOVERY_OFFSET1              = 0x47,
	.REG_23_CLOCK_RECOVERY_OFFSET0              = 0xAE,
	.REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1    = 0x02,
	.REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0    = 0x8F,

	// 1
	.REG_27_RSSI_THRESHOLD                      = 0x1E,

	// 1
	.REG_2A_AFC_LIMITER                         = 0x00,

	// 3
	.REG_2C_OOK_COUNTER_VALUE_1                 = 0x18,
	.REG_2D_OOK_COUNTER_VALUE_2                 = 0xBC,
	.REG_2E_SLICER_PEAK_HOLD                    = 0x2C,

	// 1
	.REG_30_DATA_ACCESS_CONTROL                 = 0x8D,

	// 21
	.REG_32_HEADER_CONTROL1                     = 0x0C,
	.REG_33_HEADER_CONTROL2                     = 0x22,
	.REG_34_PREAMBLE_LENGTH                     = 0x08,
	.REG_35_PREAMBLE_DETECTION_CONTROL1         = 0x2A,
	.REG_36_SYNC_WORD3                          = 0x2D,
	.REG_37_SYNC_WORD2                          = 0xD4,
	.REG_38_SYNC_WORD1                          = 0x00,
	.REG_39_SYNC_WORD0                          = 0x00,
	.REG_3A_TRANSMIT_HEADER3                    = 0x00,
	.REG_3B_TRANSMIT_HEADER2                    = 0x00,
	.REG_3C_TRANSMIT_HEADER1                    = 0x00,
	.REG_3D_TRANSMIT_HEADER0                    = 0x00,
	.REG_3E_PACKET_LENGTH                       = 0x00,
	.REG_3F_CHECK_HEADER3                       = 0x00,
	.REG_40_CHECK_HEADER2                       = 0x00,
	.REG_41_CHECK_HEADER1                       = 0x00,
	.REG_42_CHECK_HEADER0                       = 0x00,
	.REG_43_HEADER_ENABLE3                      = 0xFF,
	.REG_44_HEADER_ENABLE2                      = 0xFF,
	.REG_45_HEADER_ENABLE1                      = 0xFF,
	.REG_46_HEADER_ENABLE0                      = 0xFF,

	// 1
	.REG_58_CHARGE_PUMP_CURRENT_TRIMMING        = 0x86,

	// 1
	.REG_60_CHANNEL_FILTER_COEFFICIENT_ADDRESS  = 0x00,

	// 1
	.REG_62_CRYSTAL_OSCILLATOR_POR_CONTROL      = 0x04,

	// 1
	.REG_69_AGC_OVERRIDE1                       = 0x20,

	// 11
	.REG_6D_TX_POWER                            = 0x18,
	.REG_6E_TX_DATA_RATE1                       = 0x0A,
	.REG_6F_TX_DATA_RATE0                       = 0x3D,
	.REG_70_MODULATION_CONTROL1                 = 0x0C,
	.REG_71_MODULATION_CONTROL2                 = 0x00,
	.REG_72_FREQUENCY_DEVIATION                 = 0x20,
	.REG_73_FREQUENCY_OFFSET1                   = 0x00,
	.REG_74_FREQUENCY_OFFSET2                   = 0x00,
	.REG_75_FREQUENCY_BAND_SELECT               = 0x75,
	.REG_76_NOMINAL_CARRIER_FREQUENCY1          = 0xBB,
	.REG_77_NOMINAL_CARRIER_FREQUENCY0          = 0x80,

	// 2
	.REG_79_FREQUENCY_HOPPING_CHANNEL_SELECT    = 0x00,
	.REG_7A_FREQUENCY_HOPPING_STEP_SIZE         = 0x00,

	// 3
	.REG_7C_TX_FIFO_CONTROL1                    = 0x37,
	.REG_7D_TX_FIFO_CONTROL2                    = 0x04,
	.REG_7E_RX_FIFO_CONTROL                     = 0x37,
};

struct EEPROM_SETTINGS2 EEMEM system_settings =
{
	.TX_RETRIES = 3,
	.CALLSIGN = "",
};

