/*
	Application defines for bldcdisp.c
*/
#ifndef __BMS_APP__
#define __BMS_APP__
/*
	Target is ATxmega32A4U with the following fuse settings:
	FUSEBYTE1 = 0x00
	FUSEBYTE2 = 0xBE
	FUSEBYTE4 = 0xFF
	FUSEBYTE5 = 0xE2
	
	WDWP = 8CLK
	WDP = 8CLK
	BOOTRST = BOOTLDR
	TOSCSEL = XTAL
	BODPD = CONTINUOUS
	RSTDISBL = [ ]
	SUT = 0MS
	WDLOCK = [ ]
	BODACT = CONTINUOUS
	EESAVE = [X]
	BODLVL = 2V6
	
	Also make sure BOW pullup is disabled on Motor PCB
*/

#include <stdint.h>

//Defines for different hardware versions (PCBs)
#define HW_NIMH_BMS_REV1	1		// First revision of original banana NiMH BMS replacement.

//Define for different display version (9600 or 19200 baud..)
#define HW_DISP_ORIG		0
#define HW_DISP_CU3			1

//Set this to the version you have.
#define HARDWARE_VER		HW_NIMH_BMS_REV1
#define DISPLAY_VER			HW_DISP_ORIG

//Define pin configuration of hardware.
#if(HARDWARE_VER == HW_NIMH_BMS_REV1)
	#define MIDPOINT_V_PORT		PORTA
	#define MIDPOINT_V_PIN		PIN0_bm
	#define VIN_SENSE_PORT		PORTA
	#define VIN_SENSE_PIN		PIN1_bm
	#define CHG_PRESENT_PORT	PORTA
	#define CHG_PRESENT_PIN		PIN2_bm
	#define PACK_TEMP_PORT		PORTA
	#define PACK_TEMP_PIN		PIN3_bm
	#define PCB_TEMP_PORT		PORTA
	#define PCB_TEMP_PIN		PIN4_bm

	#define SDA_C_PIN			PIN0_bm
	#define SCL_C_PIN			PIN1_bm			
	#define BQ34Z100_ALERT_PORT	PORTC
	#define BQ34Z100_ALERT_PIN	PIN2_bm
	#define LIGHT_EN_PORT		PORTC
	#define LIGHT_EN_PIN		PIN3_bm
	#define MOT_PWR_EN_PORT		PORTC
	#define MOT_PWR_EN_PIN		PIN4_bm
	#define CHG_PWR_EN_PORT		PORTC
	#define CHG_PWR_EN_PIN		PIN5_bm
	#define LED1_PORT			PORTC
	#define LED1_PIN			PIN6_bm
	#define LED2_PORT			PORTC
	#define LED2_PIN			PIN7_bm

	#define BQ34Z100_I2C		TWIC

	#define RTC_INT_PORT		PORTD
	#define RTC_INT_PIN			PIN3_bm
	
	#define SDA_E_PIN			PIN0_bm
	#define SCL_E_PIN			PIN1_bm
	#define BOW_PORT			PORTE
	#define BOW_TX_PIN			PIN3_bm
	#define BOW_RX_PIN			PIN2_bm
	
	#define BOW_UART			USARTE0
	
#else	
	#error Unknown hardware defined, please edit app.h
#endif

//Common in all hardware, yet.:
#if(HARDWARE_VER == HW_NIMH_BMS_REV1)
	#define red_led_on()				LED1_PORT.OUTSET = LED1_PIN;
	#define red_led_off()				LED1_PORT.OUTCLR = LED1_PIN;
	#define green_led_on()				LED2_PORT.OUTSET = LED2_PIN;
	#define green_led_off()				LED2_PORT.OUTCLR = LED2_PIN;
	#define motor_on()					MOT_PWR_EN_PORT.OUTSET = MOT_PWR_EN_PIN;
	#define motor_off()					MOT_PWR_EN_PORT.OUTCLR = MOT_PWR_EN_PIN;
	#define charge_on()					CHG_PWR_EN_PORT.OUTSET = CHG_PWR_EN_PIN;
	#define charge_off()				CHG_PWR_EN_PORT.OUTCLR = CHG_PWR_EN_PIN;
#else
#endif

// Global
#define F_CPU       32000000UL
#define BQ34Z100_ADDR 0x55

//EEPROM
#define EEBLOCK_SETTINGS1					2		//Contains user settings.
#define EEMEM_MAGIC_HEADER_SETTINGS			0x1339	//Magic header.

// I2C

/*! BAUDRATE 100kHz and Baudrate Register Settings */
#define I2C_BAUDRATE	100000
#define I2C_BAUDSETTING TWI_BAUD(F_CPU, I2C_BAUDRATE)


// ADC
#define PACK_V_MIN 17000
#define PACK_V_MAX 31500
#define	PACK_D_MAX 1.10			// Pack delta in V
#define MID_V_MIN  9.50
#define MID_V_MAX  15.25 

#define PACK_C_MAX 40.00
#define PCB_C_MAX 40.00

// To be modified
typedef struct settings_t settings_t;
struct settings_t{
	uint16_t straincal;
	uint16_t uvcal;
	uint16_t ovcal;
	uint16_t tempcal;
	uint8_t  straingain;
};

typedef struct adc_readings_t adc_readings_t;
struct adc_readings_t {
	float midpointV;
	float vinSenseV;
	int16_t chgPresentV;
	float packTempC;
	float pcbTempC;
};

typedef struct bq_readings_t bq_readings_t;
struct bq_readings_t {
	uint16_t packVoltage, statusFlags, fullCap, remainingCap;
	double icTemp, packTemp;
	int16_t current;
	uint8_t SOC;
};

extern settings_t eepsettings;


#endif