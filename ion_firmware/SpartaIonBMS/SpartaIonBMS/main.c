/*
 * SpartaIonBMS.c
 *
 * Created: 29-8-2020 14:41:45
 * Author : T. Ligeon
 *	
 *  BQ34Z100 = C0/C1
 *	BOW_UART = PE2/PE3
 *	RTC I2C = PE0/PE1
 */ 

// Defines

// Includes
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdio.h>
#include "app.h"
#include "uart.h"
#include "twi_master_driver.h"
#include "bq34z100.h"



//For Bowbus
bowbus_net_s bus;
bool bus_check_for_message(void);
char bow_buffer[1024];
uint32_t bowStrSize=0;

// Main Variables
TWI_Master_t i2c_handler_BQ34Z100;
TWI_Master_t i2c_handler_RTC;

adc_readings_t adcData;
bq_readings_t bqData;

static uint8_t cnt_tm = 0;			//Used by TCC1
static uint8_t sleepDelayTimer=0;	// Used for sleeping

static uint8_t oldDisplayStatus=0;

// Read signature byte for ADC Calibration
uint8_t ReadSignatureByte(uint16_t Address)
{
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t Result;
	Result = pgm_read_byte((uint8_t *)Address);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return Result;
}

//Initialize all IO.
void _init_GPIO(void){
	
	PORTA.DIRCLR = MIDPOINT_V_PIN | VIN_SENSE_PIN | CHG_PRESENT_PIN | PACK_TEMP_PIN | PCB_TEMP_PIN;
	PORTC.DIRCLR = BQ34Z100_ALERT_PIN;
	PORTC.DIRSET = LIGHT_EN_PIN | MOT_PWR_EN_PIN | CHG_PWR_EN_PIN | LED1_PIN | LED2_PIN | SDA_C_PIN | SCL_C_PIN;
	PORTD.DIRCLR = RTC_INT_PIN;	
	PORTE.DIRSET = SDA_E_PIN | SCL_E_PIN | BOW_TX_PIN;
	PORTE.DIRCLR = BOW_RX_PIN;
	
	
	// Enable interrrupt for Charge Detection
	//CHG_PRESENT_PORT.PIN2CTRL = PORT_ISC_RISING_gc;
	//CHG_PRESENT_PORT.INT1MASK = CHG_PRESENT_PIN;
	//CHG_PRESENT_PORT.INTCTRL = PORT_INT1LVL_LO_gc;
	
	
	// Enable pullups on all unused pins to reduce sleep power consumption.
	PORTA.PIN5CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN7CTRL = PORT_OPC_PULLUP_gc;
	PORTB.PIN0CTRL = PORT_OPC_PULLUP_gc;
	PORTB.PIN1CTRL = PORT_OPC_PULLUP_gc;
	PORTB.PIN2CTRL = PORT_OPC_PULLUP_gc;
	PORTB.PIN3CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN1CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN3CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN4CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN5CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN7CTRL = PORT_OPC_PULLUP_gc;
	
}

void _init_adc(void){
	
	ADCA.CALL = ReadSignatureByte(0x20) ; //ADC Calibration Byte 0
	ADCA.CALH = ReadSignatureByte(0x21) ; //ADC Calibration Byte 1

	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm; //12 Bit, Signed Conversion
	ADCA.REFCTRL = 	ADC_REFSEL_INTVCC_gc; // VCC/1.6
	ADCA.PRESCALER   = ADC_PRESCALER_DIV4_gc; 
	
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;		//Midpoint Channel
	ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	
	ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;		//Pack Voltage Channel
	ADCA.CH1.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	
	ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;		//Pack Temp Channel
	ADCA.CH2.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	
	ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;		//PCB Temp Channel
	ADCA.CH3.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	
	ADCA.CTRLA  = ADC_ENABLE_bm;
}

void _init_peripherals(void){
	//Setup a timer, just for counting. Used for bus message and timeouts.
	TCC1_CTRLA = TC_CLKSEL_DIV1024_gc;
	
	//Init USART for BOW bus
	uart_init();
	bus_init(&bus);
	
	//Init I2C for BQ34Z100,
	TWI_MasterInit(&i2c_handler_BQ34Z100,&BQ34Z100_I2C,TWI_MASTER_INTLVL_LO_gc,I2C_BAUDSETTING);
	//Init I2C for RTC,
	//TWI_MasterInit(&i2c_handler_RTC,&BQ34Z100_I2C,TWI_MASTER_INTLVL_LO_gc,I2C_BAUDSETTING);
	
}

void clock_switch32MInt(void){
	/*Setup clock to run from Internal 32MHz oscillator.*/
	OSC.CTRL |= OSC_RC32MEN_bm;
	while (!(OSC.STATUS & OSC_RC32MRDY_bm));
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
	OSC.CTRL &= (~OSC_RC2MEN_bm);
	PORTCFG.CLKEVOUT = (PORTCFG.CLKEVOUT & (~PORTCFG_CLKOUT_gm)) | PORTCFG_CLKOUT_OFF_gc;	// disable peripheral clock
}

void clock_switch32MExt(void){
	/*Setup 32MHz clock to run from External 16MHz oscillator.*/
	OSC.XOSCCTRL |=  OSC_FRQRANGE1_bm | OSC_FRQRANGE0_bm | OSC_XOSCSEL3_bm | OSC_XOSCSEL1_bm | OSC_XOSCSEL0_bm;
	OSC.PLLCTRL =  OSC_PLLSRC1_bm | OSC_PLLSRC0_bm |OSC_PLLFAC1_bm ;
	OSC.CTRL |= OSC_XOSCEN_bm;
	while (!(OSC.STATUS & OSC_XOSCRDY_bm));
	OSC.CTRL |= OSC_PLLEN_bm;
	while (!(OSC.STATUS & OSC_PLLRDY_bm));
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;
	OSC.CTRL &= (~OSC_RC2MEN_bm);
	PORTCFG.CLKEVOUT = (PORTCFG.CLKEVOUT & (~PORTCFG_CLKOUT_gm)) | PORTCFG_CLKOUT_OFF_gc;	// disable peripheral clock
}

void _unit_Test(void){
	/*
		Functional Description:
		Used to test PCB actuators.
		Toggle Relay 1,
		Toggle LED 1
		Toggle Relay 2,
		Toggle Led 2,
	*/ 	
	LED1_PORT.OUTSET = LED1_PIN;
	_delay_ms(500);
	LED1_PORT.OUTCLR = LED1_PIN;
	_delay_ms(500);
	LED2_PORT.OUTSET = LED2_PIN;
	_delay_ms(500);
	LED2_PORT.OUTCLR = LED2_PIN;
	_delay_ms(500);
	MOT_PWR_EN_PORT.OUTSET = MOT_PWR_EN_PIN;
	_delay_ms(500);
	MOT_PWR_EN_PORT.OUTCLR = MOT_PWR_EN_PIN;
	_delay_ms(500);
	CHG_PWR_EN_PORT.OUTSET = CHG_PWR_EN_PIN;
	_delay_ms(500);
	CHG_PWR_EN_PORT.OUTCLR = CHG_PWR_EN_PIN;
	_delay_ms(500);
}

uint8_t _read_adc(struct adc_readings_t* adc_data){
	// THIS IS NOT YET FUNCTIONAL
	//uint8_t errorFlag=0;
	
	ADCA.CTRLA |= ADC_CH0START_bm | ADC_CH1START_bm | ADC_CH2START_bm | ADC_CH3START_bm;
	while(ADCA.INTFLAGS != 0x0F){printf("ADC Not done!\n"); _delay_ms(100);} // Wait for ADC channel conversions
	
	int16_t adc0Res = ADCA.CH0RES;
	int16_t adc1Res = ADCA.CH1RES;
	int16_t adc2Res = ADCA.CH2RES;
	int16_t adc3Res = ADCA.CH3RES;
	
	float adcREF = 3.3/1.6;
	uint16_t adcRES = 2047;
	
	
	//Run scalings on these values.
	adc_data->midpointV = (adcREF/adcRES)*adc0Res * 11.0;// / 2047 ;
	adc_data->vinSenseV = (adcREF/adcRES)*adc1Res * 22.28;// * v_divider / 2047; 
	adc_data->packTempC = adc2Res/22.7;//INT16_MAX) * pack_c_divider; 
	adc_data->pcbTempC = adc3Res/56.3;//INT16_MAX) * pcb_c_divider;
	
	//float ntcR = adc2Res / ((adcRES/adcREF) * 3.3) 
	
	//float steinhart;
	//steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
	//steinhart = log(steinhart);                  // ln(R/Ro)
	//steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
	//steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	//steinhart = 1.0 / steinhart;                 // Invert
	//steinhart -= 273.15;                         // convert to C
	
	//Run Sanity/Error checks on values, and return error flags accordingly
	//if(adc_data->vinSenseV < PACK_V_MIN){}
	//if(adc_data->vinSenseV > PACK_V_MAX){}
	//if(adc_data->midpointV < MID_V_MIN){}
	//if(adc_data->midpointV > MID_V_MAX){}
	//if((adc_data->vinSenseV - adc_data->midpointV) - adc_data->midpointV > PACK_D_MAX || (adc_data->vinSenseV - adc_data->midpointV) - adc_data->midpointV < -PACK_D_MAX ){}
	
	//if(adc_data->packTempC > PACK_C_MAX){}
	//if(adc_data->pcbTempC > PCB_C_MAX){}
	
	//Watchdog on ADC reading.
	
	
	return 0;
}

void _setup_FuelGauge(void){
	//_setup_FuelGauge() is used to setup the Fuel Gauge
	// reset
	//bq_Reset(&i2c_handler_BQ34Z100);
	// unseal.
	bq_unseal(&i2c_handler_BQ34Z100);
	
	// Set VoltSel to 1
	//bq_setVoltSel(&i2c_handler_BQ34Z100, true);
	
	// Enable Sleep mode
	//bq_setSleepMode(&i2c_handler_BQ34Z100, true);
	
	// Set Number of cells in series.
	//bq_setCellCount(&i2c_handler_BQ34Z100,20);
	
	//Change Voltage Divider
	//TODO: This function's value doesnt fully match value written to BQ register
	//bq_setVoltageDivider(&i2c_handler_BQ34Z100, 38525); //38450
	
	// Set Pack Capacity.
	//bq_setDesignCapacity(&i2c_handler_BQ34Z100,11000);
	//bq_setDesignEnergyScale(&i2c_handler_BQ34Z100,10);	//wH
	//bq_setDesignEnergy(&i2c_handler_BQ34Z100,24200);		//22V*11aH

	// Calibrate Current
	//bq_calibrateCurrent(&i2c_handler_BQ34Z100,-150);
	
	// Set Loadmode & LoadSelect
	//bq_setLoadMode(&i2c_handler_BQ34Z100, 0);	// Dont use Constant-Power Model.
	//bq_setLoadSelect(&i2c_handler_BQ34Z100,1);	//Present average discharge current
	
	// Set Cell terminate voltage
	//bq_setCellTerminateVoltage(&i2c_handler_BQ34Z100, 1000);	//1000 mV per cell
	
	// Set Quit current (Still to be measured);
	//bq_setQuitCurrent(&i2c_handler_BQ34Z100, 40); //40mA
	
	// Set QmaxCell0 (Theoretical Maximum cell capacity)
	//bq_setQmaxCell0(&i2c_handler_BQ34Z100, 11000);
	
	// Set Cell Charge Termination Voltage (1.5V for NiMH)
	// TODO: Not the same value, BQ thing.. And Dont know the usage.
	//bq_setCellChargeTerminationVoltage(&i2c_handler_BQ34Z100, 1500);
	
	// Set Nimh Charge Termination Indicators
	//bq_setNi_dTterm(&i2c_handler_BQ34Z100, true);
	//bq_setNi_dVterm(&i2c_handler_BQ34Z100, true);
	
	
	// Set cell Thresholds
	//bq_setCellBLsetThreshold(&i2c_handler_BQ34Z100, 1000);
	//bq_setCellBLclearThreshold(&i2c_handler_BQ34Z100,1100);
	//bq_setCellBHsetThreshold(&i2c_handler_BQ34Z100,1500);
	//bq_setCellBHclearThreshold(&i2c_handler_BQ34Z100,1300);
	
	// Set Capacity Thresholds
	//bq_setSOC1setThreshold(&i2c_handler_BQ34Z100,1000);
	//bq_setSOC1clearThreshold(&i2c_handler_BQ34Z100,1500);
	//bq_setSOCFsetThreshold(&i2c_handler_BQ34Z100,500);
	//bq_setSOCFclearThreshold(&i2c_handler_BQ34Z100,1000);
	
	//Enable ImpedanceTrack
	//bq_enableIT(&i2c_handler_BQ34Z100);
	
	
	// Reset BQ34z100
	//bq_Reset(&i2c_handler_BQ34Z100);
}

void _update_FuelGauge(void){
	// _update_FuelGauge() is used to retrieve the FuelGauge parameters
	bqData.packVoltage = bq_ReadVoltage(&i2c_handler_BQ34Z100);
	bqData.current = bq_getCurrent(&i2c_handler_BQ34Z100);
	bqData.SOC = bq_getSOC(&i2c_handler_BQ34Z100);
	bqData.icTemp = (bq_getChipTemperature(&i2c_handler_BQ34Z100)/10.0)-273.15;
	bqData.packTemp = (bq_getPackTemperature(&i2c_handler_BQ34Z100)/10.0)-273.15;
	bqData.statusFlags = bq_getFlags(&i2c_handler_BQ34Z100);
	//bqData.fullCap = bq_getFullCapacity(&i2c_handler_BQ34Z100);
	//bqData.remainingCap = bq_getRemainingCapacity(&i2c_handler_BQ34Z100);	
}

void _run_optimizationCycle(void){
// Step 1, Enable IT, Reset and Discharge completely
uint16_t controlStatus=0;
//bq_enableIT(&i2c_handler_BQ34Z100);
//_delay_ms(1000);
//bq_Reset(&i2c_handler_BQ34Z100);
//_delay_ms(1000);
_update_FuelGauge();
bowStrSize = snprintf(bow_buffer, 1024 ,"FuelGauge Optimization Cycle\r\n Start discharging until 19.8V...\r\n");
bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
while(bus_isbusy(&bus) == 0){};
MOT_PWR_EN_PORT.OUTSET= MOT_PWR_EN_PIN;
LED2_PORT.OUTSET = LED2_PIN;
while(bqData.packVoltage >= 19800){
	_update_FuelGauge();
	bowStrSize = snprintf(bow_buffer, 1024 ,"Voltage: %d, Current: %d, Flags: %u,ControlStatus: %u\r\n", bqData.packVoltage, bqData.current, bqData.statusFlags ,bq_GetStatus(&i2c_handler_BQ34Z100));
	bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
	while(bus_isbusy(&bus) == 0){};
	_delay_ms(5000);
}
MOT_PWR_EN_PORT.OUTCLR= MOT_PWR_EN_PIN;
LED2_PORT.OUTCLR = LED2_PIN;
bowStrSize = snprintf(bow_buffer, 1024 ,"Discharging Done. Wait	until VOK and RUP_DIS are cleared.\r\n");
bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
while(bus_isbusy(&bus) == 0){};
// Step 2,  relax for 5 hours.
_delay_ms(5000);
controlStatus = bq_GetStatus(&i2c_handler_BQ34Z100);
while((controlStatus & 0x0002) || (controlStatus & 0x0004)){ 
	controlStatus = bq_GetStatus(&i2c_handler_BQ34Z100);
	bowStrSize = snprintf(bow_buffer, 1024 ,"ControlStatus: %u\r\n", controlStatus);
	bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
	while(bus_isbusy(&bus) == 0){};
	_delay_ms(5000);
}
bowStrSize = snprintf(bow_buffer, 1024 ," 5 Hour Wait Complete\r\n");
bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
while(bus_isbusy(&bus) == 0){};
// Step 3, Charge again to full cap, do this attended!
while(1){}
bowStrSize = snprintf(bow_buffer, 1024 ," Charge Bat to Full!\r\n");
bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
while(bus_isbusy(&bus) == 0){};
_update_FuelGauge();
charge_on();
red_led_on();
while(bqData.packVoltage <= 31400 && !(bqData.statusFlags & 0x0200)){
	_update_FuelGauge();
	bowStrSize = snprintf(bow_buffer, 1024 ,"Voltage: %d, Current: %d, Flags: %u,ControlStatus: %u\r\n", bqData.packVoltage, bqData.current, bqData.statusFlags ,bq_GetStatus(&i2c_handler_BQ34Z100));
	bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
	while(bus_isbusy(&bus) == 0){};
	_delay_ms(5000);
}
CHG_PWR_EN_PORT.OUTCLR = CHG_PWR_EN_PIN;
LED1_PORT.OUTCLR = LED1_PIN;
bowStrSize = snprintf(bow_buffer, 1024 ," Full charge detected!\r\n");
bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
while(bus_isbusy(&bus) == 0){}
// Step 4, Wait 2 hours
bowStrSize = snprintf(bow_buffer, 1024 ,"Charging Done. Wait 2 HRS or VOK RUPDIS before discharging\r\n");
bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
while(bus_isbusy(&bus) == 0){};
_delay_ms(5000);
//Delay 2 Hours
controlStatus = bq_GetStatus(&i2c_handler_BQ34Z100);
while((controlStatus & 0x0002) || (controlStatus & 0x0004)){
	controlStatus = bq_GetStatus(&i2c_handler_BQ34Z100);
	bowStrSize = snprintf(bow_buffer, 1024 ,"ControlStatus: %u\r\n", controlStatus);
	bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
	while(bus_isbusy(&bus) == 0){};
	_delay_ms(5000);
}
bowStrSize = snprintf(bow_buffer, 1024 ," 2 Hour Wait Complete\r\n");
bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
while(bus_isbusy(&bus) == 0){};
_delay_ms(5000);
// Step 5, Discharge again
_update_FuelGauge();
bowStrSize = snprintf(bow_buffer, 1024 ,"Start Discharge again\r\n");
bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
while(bus_isbusy(&bus) == 0){};
MOT_PWR_EN_PORT.OUTSET= MOT_PWR_EN_PIN;
LED2_PORT.OUTSET = LED2_PIN;
while(bqData.packVoltage >= 19700){
	_update_FuelGauge();
	bowStrSize = snprintf(bow_buffer, 1024 ,"Voltage: %d, Current: %d, Flags: %u,ControlStatus: %u\r\n", bqData.packVoltage, bqData.current, bqData.statusFlags ,bq_GetStatus(&i2c_handler_BQ34Z100));
	bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
	while(bus_isbusy(&bus) == 0){};
	_delay_ms(5000);
}
MOT_PWR_EN_PORT.OUTCLR= MOT_PWR_EN_PIN;
LED2_PORT.OUTCLR = LED2_PIN;
bowStrSize = snprintf(bow_buffer, 1024 ,"Discharge Complete, Starting 5 hour wait again\r\n");
bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
while(bus_isbusy(&bus) == 0){};
// Step 6, Wait 5 Hours again.
controlStatus = bq_GetStatus(&i2c_handler_BQ34Z100);
while((controlStatus & 0x0002) || (controlStatus & 0x0004)){
	controlStatus = bq_GetStatus(&i2c_handler_BQ34Z100);
	bowStrSize = snprintf(bow_buffer, 1024 ,"ControlStatus: %u\r\n", controlStatus);
	bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
	while(bus_isbusy(&bus) == 0){};
	_delay_ms(5000);
}
bowStrSize = snprintf(bow_buffer, 1024 ," 5 Hour Wait Complete, Optimization done!\r\n");
bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
while(bus_isbusy(&bus) == 0){};
while(1){
	bowStrSize = snprintf(bow_buffer, 1024 ,"Calibration Complete, SOC: %d, FullCap: %d, RemCap: %d, Flags: %u\r\n", bqData.SOC, bqData.fullCap, bqData.remainingCap, bqData.statusFlags);
	bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
	while(bus_isbusy(&bus) == 0){};
	_delay_ms(5000);
}	
	
}

void _setup_lowPower(void){
	
	NVM.CTRLB |= NVM_FPRM_bm | NVM_EPRM_bm;  
	
	PR.PRGEN = PR_AES_bm | PR_USB_bm | PR_EVSYS_bm | PR_DMA_bm;		//Leave RTC on
	PR.PRPA = PR_DAC_bm | PR_AC_bm;									//Leave ADC on
	PR.PRPB = PR_DAC_bm | PR_AC_bm;	
	PR.PRPC = PR_USART1_bm | PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm  ; //Leave TWI and TCC1 on
	PR.PRPD = PR_TWI_bm | PR_USART1_bm | PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC1_bm | PR_TC0_bm  ; //Turn completely off
	PR.PRPE = PR_USART1_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC1_bm | PR_TC0_bm  ; // Leave TWI and UART on.
	
	// Setup RTC for sleep and waking
	//CLK.RTCCTRL = CLK_RTCSRC_ULP_gc;
	//while(RTC.STATUS & RTC_SYNCBUSY_bm){}
	//RTC.PER = 0; // 1S with 1024 div.
	//RTC.INTFLAGS = RTC_OVFIF_bm;
	//RTC.INTCTRL = RTC_OVFINTLVL_HI_gc;
	//RTC.CTRL = RTC_PRESCALER_OFF_gc;
	
	// Setup UART pin for waking
	BOW_PORT.PIN2CTRL = PORT_ISC_FALLING_gc;
	BOW_PORT.INT0MASK = BOW_RX_PIN;
	BOW_PORT.INTCTRL = PORT_INT0LVL_OFF_gc;
	
	//Disable JTAG
	CCP = CCP_IOREG_gc;
	MCU.MCUCR = MCU_JTAGD_bm;
	
	//Disable WDT
	//CCP = CCP_IOREG_gc;
	//WDT.CTRL |= WDT_CEN_bm;
	//WDT.CTRL &= ~(WDT_ENABLE_bm); 
	
}

void _enter_Sleep(void){
	
	// Disable all outputs
	motor_off(); green_led_off();
	charge_off(); red_led_off();
	LIGHT_EN_PORT.OUTCLR = LIGHT_EN_PIN;
		
	// Turn off ADC Perip
	ADCA.CTRLA  &= ~(ADC_ENABLE_bm);
	
	// Enable UART interrupt for waking
	BOW_PORT.INTCTRL = PORT_INT0LVL_HI_gc;
	
	_delay_ms(2000);
	
	// Enable RTC for wakeup.
	//CLK.RTCCTRL |= CLK_RTCEN_bm;
	//while(RTC.STATUS & RTC_SYNCBUSY_bm){}
	//RTC.CTRL = RTC_PRESCALER_DIV1024_gc;
	NVM.CTRLB &= ~(NVM_FPRM_bm);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sei();
	sleep_cpu();
	//SLEEEPING!!!!
	sleep_disable();
	NVM.CTRLB |= NVM_FPRM_bm | NVM_EPRM_bm; 
	//while(RTC.STATUS & RTC_SYNCBUSY_bm){}
	//RTC.CTRL = RTC_PRESCALER_OFF_gc;
	//Enable ADC
	ADCA.CTRLA  = ADC_ENABLE_bm;
	CLK.RTCCTRL &= ~(CLK_RTCEN_bm);
	
	//Disable UART waking interrupt
	BOW_PORT.INTCTRL = PORT_INT0LVL_OFF_gc;
	
	sei();
	
}

int main(void)
{	
	//Run of the external 32MHz oscillator.
	//clock_switch32MInt();
	clock_switch32MExt();
	
	_init_GPIO();
	_init_peripherals();
	_init_adc();
	_setup_lowPower();
	
	//Read EEPROM
	//eemem_read_block(EEMEM_MAGIC_HEADER_SETTINGS,(uint8_t*)&eepsettings, sizeof(eepsettings), EEBLOCK_SETTINGS1);
	
	//Make sure to move the reset vectors to application flash.
	//Set the interrupts we want to listen to.
	CCP = CCP_IOREG_gc;
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm |PMIC_HILVLEN_bm | PMIC_RREN_bm;

	//Enable interrupts.
	sei();

	// Fuel Gauge Initial Setup
	// Still have to write function to support Flashstream
	//_setup_FuelGauge();
	//_run_optimizationCycle();

	// Setup done
	// Only for testing:
	//red_led_on();
	//green_led_on();
	//motor_on();
	//while(1){}
	//MOT_PWR_EN_PORT.OUTSET = MOT_PWR_EN_PIN;
	//display.online = true;
	//TCC1_CTRLA = TC_CLKSEL_OFF_gc;
	// (Print) Starting Fuel Gauge Info:
	//_update_FuelGauge();
	//printf("Voltage: %d, Current: %d, SOC: %d, DesignCap: %d, FullCap: %d, RemCap: %d \r\n", packVoltage, bq_getCurrent(&i2c_handler_BQ34Z100), bq_getSOC(&i2c_handler_BQ34Z100), bq_getDesignCapacity(&i2c_handler_BQ34Z100),bq_getFullCapacity(&i2c_handler_BQ34Z100), bq_getRemainingCapacity(&i2c_handler_BQ34Z100));
	//printf("PackTemp: %d, ICtemp: %d, Flags: %d\r\n", bq_getChipTemperature(&i2c_handler_BQ34Z100), bq_getPackTemperature(&i2c_handler_BQ34Z100), bq_getFlags(&i2c_handler_BQ34Z100));
	//_setup_FuelGauge();
	
	while(1){
		//Always run FuelGauge Update
		_update_FuelGauge();
		_read_adc(&adcData);
		//bowStrSize = snprintf(bow_buffer, 1024 ,"V: %.2f,FGV: %d,MV: %.2f,I: %d,PCBT: %.2f,ICT: %.2lf,PACKT: %.2f, FGPT: %.2lf, CTRL: %d\r\n", adcData.vinSenseV,packVoltage,adcData.midpointV,current,adcData.pcbTempC,icTemp,adcData.packTempC,packTemp,bq_GetStatus(&i2c_handler_BQ34Z100));
		//bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
		//while(bus_isbusy(&bus) == 0){};
		//Check if charging routine can be started:
		if((CHG_PRESENT_PORT.IN & CHG_PRESENT_PIN)){
			uint8_t noChargeCurrent=0;
			// Do Safety Checks, See if this needs to be changed to do Optimization cycle.
			if((bqData.packVoltage < PACK_V_MAX) && (bqData.statusFlags & 0x0100) && !(bqData.statusFlags & 0x8000)){
				display.chargingState=true;
				bus_cu3_display_update(&bus);
				motor_off(); green_led_off();
				charge_on(); red_led_on();
				// Lets be safe here.
				while((bqData.packVoltage < PACK_V_MAX) && (bqData.statusFlags & 0x0100) && !(bqData.statusFlags & 0x8000)){
					_update_FuelGauge();
					_read_adc(&adcData);
					// Hard Charging exits.
					if(bqData.current < 100){noChargeCurrent++;}
					if(noChargeCurrent > 10){break;}
					bus_cu3_display_update(&bus);
					_delay_ms(200);
				}
				display.chargingState=false;
				charge_off(); red_led_off();
			}
		}
		
		//14ms timer
		if (TCC1.CNT > 450){
			cnt_tm++;
			TCC1.CNT = 0;

			//Last character in message
			if (wait_for_last_char){
				wait_for_last_char = false;
			}else{
				bus_endmessage(&bus);
			}
			// Check BOW-BUS for data
			oldDisplayStatus = display.online;
			if(bus_check_for_message()){
				display.offline_cnt = 0;
				//Declare it online
				display.online = true;
				if(display.online != oldDisplayStatus){
					LIGHT_EN_PORT.OUTTGL = LIGHT_EN_PIN;
					_delay_ms(70);
					LIGHT_EN_PORT.OUTTGL = LIGHT_EN_PIN;
					_delay_ms(130);
					LIGHT_EN_PORT.OUTTGL = LIGHT_EN_PIN;
					_delay_ms(40);
					LIGHT_EN_PORT.OUTTGL = LIGHT_EN_PIN;
					_delay_ms(130);
				}
			}else{
				display.offline_cnt++;
				//Display offline for too long
				if (display.offline_cnt > 50){
					display.online = false;
					display.backlight = false;
					display.offline_cnt=0;
				}
			}
		}
		
		// Update Lights
		if(display.backlight){LIGHT_EN_PORT.OUTSET = LIGHT_EN_PIN;}
		else{LIGHT_EN_PORT.OUTCLR=LIGHT_EN_PIN;}

		// Send error to display if needed
		if(bqData.statusFlags & 0x0002){bus_cu3_send_status(&bus, 1);}
		if(bqData.statusFlags & 0x0800){bus_cu3_send_status(&bus, 2);}
		if(bqData.statusFlags & 0x8000){bus_cu3_send_status(&bus, 19);}

		// Check if Motor power out can be enabled:
		//bowStrSize = snprintf(bow_buffer, 1024 ,"Volt: %d, disp: %d, status: %u, DelaySLP: %u ,DISPOFFCNT: %u\r\n", bqData.packVoltage, display.online, bqData.statusFlags, sleepDelayTimer, display.offline_cnt);
		//bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
		//while(bus_isbusy(&bus) == 0){};
		if(bqData.packVoltage > PACK_V_MIN && display.online && !(bqData.statusFlags & 0x4000) && !(bqData.statusFlags & 0x1000)){
			motor_on(); green_led_on();
			sleepDelayTimer=0;
		}else{
			sleepDelayTimer++;
		}
		// Dont run CPU at loco speed
		_delay_ms(100);

		// If not, Check if display is present and go to sleep.
		if(sleepDelayTimer > 30){
			// Experimental
			//_delay_ms(100);
			_enter_Sleep();
			//_delay_ms(100);
			display.online = true;
			display.offline_cnt=0;
			sleepDelayTimer=0;
		}
	}
}

/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&i2c_handler_BQ34Z100);
}

// Only used for sleep mode wakeup.
ISR(RTC_OVF_vect)
{
}

// Charger interrupt vector to wake from sleep
// This CAN block CPU under circumstance, more testing needed.
// ISR(PORTC_INT1_vect){}

// UART Wake interrupt
// Send garbage data to disp to check for ack.
ISR(PORTE_INT0_vect)
{	
}

//Fuel Gauge interrupt
ISR(PORTD_INT0_vect){
	bqData.statusFlags = bq_getFlags(&i2c_handler_BQ34Z100);
}

bool bus_check_for_message(void){
	if (bus.new_mesage){
		if (bus_parse_BMS(&bus,bus.msg_buff,bus.msg_len)){
				bus.msg_len = 0;
				bus.new_mesage = false;
				return true;
				}else{
				bus.msg_len = 0;
				bus.new_mesage = false;
				return false;
			}
		}
		return false;
}

// Maybe useful code,
/*
	//Read 32 Bytes from Data Flash.
	uint8_t *flashBytes = bq_ReadFlashBlock(&i2c_handler_BQ34Z100, 48, 0x00);
	
	bowStrSize = snprintf(bow_buffer, 1024 ,"Pack Config Flash: \r\n");
	bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
	while(bus_isbusy(&bus) == 0){};
	for (uint8_t i = 0; i < 32; i++)
	{
		bowStrSize = snprintf(bow_buffer, 1024 ,"%d ",*(flashBytes+i));
		bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
		while(bus_isbusy(&bus) == 0){};
	}
	bowStrSize = snprintf(bow_buffer, 1024 ,"\r\n");
	bus_send(&bus, (uint8_t*) bow_buffer, bowStrSize);
	while(bus_isbusy(&bus) == 0){};
*/

