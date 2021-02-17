/*
 * bq34z100.c
 *
 * Created: 19-9-2020 14:00:07
 *  Author: T. Ligeon
 * Driver for TI BQ34Z100 Fuel Gauge IC.
 */ 


#include "app.h"
#include "twi_master_driver.h"
#define bq_CMD_control 0x00
#define bq_CMD_status 0x00
#define bq_CMD_SOC	0x02
#define bq_CMD_designCap	0x3C
#define bq_CMD_remainingCap 0x04
#define bq_CMD_fullCap	0x06
#define bq_CMD_voltage 0x08
#define bq_CMD_temperature 0x0C
#define bq_CMD_intTemperature 0x2A
#define bq_CMD_enableIT	0x21
#define bq_CMD_current 0x10
#define bq_CMD_ChemID 0x08
#define bq_CMD_avgCurrent 0x0A
#define bq_CMD_flags 0x0E
#define bq_CMD_flagsB 0x12
#define bq_CMD_reset 0x41

void bq_Reset(struct TWI_Master *i2cHandler)
{
	uint8_t sendBuf[3] = {0x00, bq_CMD_reset, 0x00};
	TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],3);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
}

uint8_t * bq_ReadFlashBlock(struct TWI_Master *i2cHandler, uint8_t class, uint8_t offset){
	
		uint8_t sendBuf[2];
		uint8_t *dataFlashBytes = malloc (sizeof(uint8_t) * 32);
		
		// Enable Block Data control
		sendBuf[0] = 0x61;
		sendBuf[1] = 0x00;
		TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],2);
		while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
		_delay_ms(1);	// This delay is necessary, otherwise its too fast for BQ34Z100
		// Write Pack Configuration Subclass using  DataFlashClass()
		sendBuf[0] = 0x3E;
		sendBuf[1] = class;
		TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],2);
		while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
		_delay_ms(1);	// Delay is also necessary.
		// Write block offset location using   DataFlashBlock()
		sendBuf[0] = 0x3F;
		sendBuf[1] = offset; // change this to 0x01 if offset is >31
		TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],2);
		while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
		_delay_ms(1);	// Delay is also necessary.
		for (uint8_t i = 0; i < 32; i++)
		{
			sendBuf[0] = 0x40 + i;
			TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,1);
			while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
			dataFlashBytes[i] = i2cHandler->readData[0];
		}
		
		return dataFlashBytes;
	}
	
void bq_UpdateChecksum(struct TWI_Master *i2cHandler, uint8_t *dataFlashBytes){
	
	uint8_t sendBuf[2];
	
	// Calculate and write new checksum to save config
	uint8_t chkSum = 0;
	for (uint8_t i = 0; i < 32; i++)
	{
		chkSum += dataFlashBytes[i];
	}
	uint8_t chkSumTemp = chkSum / 256;
	chkSum = chkSum - (chkSumTemp * 256);
	chkSum = 255 - chkSum;

	// Write new checksum
	sendBuf[0] = 0x60;
	sendBuf[1] = chkSum;
	TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	_delay_ms(1);	// Lets also keep this in because of BQ speed
	//bq_Reset(i2cHandler);	//Reset after every modifcation. To prevent false Checksum Calc.
}

void bq_ModifyFlash(struct TWI_Master *i2cHandler, uint8_t *dataFlashBytes ,uint8_t index, int value){
	
	uint8_t sendBuf[2];
	//*(dataFlashBytes + index) = value;
	dataFlashBytes[index] = value;
	
	// Write block offset location using   DataFlashBlock()
	sendBuf[0] = 0x40 + index;
	sendBuf[1] = dataFlashBytes[index];//*(dataFlashBytes + index);
	TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.  
	_delay_ms(1);
	// Update checksum to save changes
	bq_UpdateChecksum(i2cHandler, dataFlashBytes);
}

float bq_xemicsToFloat(uint32_t X)
{
	bool bIsPositive = false;
	float fExponent, fResult;
	uint8_t vMSByte = (uint8_t) (X >> 24);
	uint8_t vMidHiByte = (uint8_t) (X >> 16);
	uint8_t vMidLoByte = (uint8_t) (X >> 8);
	uint8_t vLSByte = (uint8_t) X;
	// Get the sign, its in the 0x00 80 00 00 bit
	if ((vMidHiByte & 128) == 0)
	{ bIsPositive = true; }

	// Get the exponent, it's 2^(MSbyte - 0x80)
	fExponent = pow(2, (vMSByte - 128));
	// Or in 0x80 to the MidHiByte
	vMidHiByte = (uint8_t)(vMidHiByte | 128);
	// get value out of midhi byte
	fResult = (vMidHiByte) * 65536;
	// add in midlow byte
	fResult = fResult + (vMidLoByte * 256);
	// add in LS byte
	fResult = fResult + vLSByte;
	// multiply by 2^-24 to get the actual fraction
	fResult = fResult * pow(2, -24);
	// multiply fraction by the ‘exponent’ part
	fResult = fResult * fExponent;
	// Make negative if necessary
	if (bIsPositive)
	return fResult;
	else
	return -fResult;
}

uint32_t bq_floatToXemics(float X)
{
	int iByte1, iByte2, iByte3, iByte4, iExp;
	bool bNegative = false;
	float fMantissa;
	// Don't blow up with logs of zero
	if (X == 0) X = 0.00001F;
	if (X < 0)
	{
		bNegative = true;
		X = -X;
	}
	// find the correct exponent
	iExp = (int)((log(X) / log(2)) + 1);// remember - log of any base is ln(x)/ln(base)

	// MS byte is the exponent + 0x80
	iByte1 = iExp + 128;
	
	// Divide input by this exponent to get mantissa
	fMantissa = X / (pow(2, iExp));
	
	// Scale it up
	fMantissa = fMantissa / (pow(2, -24));
	
	// Split the mantissa into 3 bytes
	iByte2 = (int)(fMantissa / (pow(2, 16)));
	
	iByte3 = (int)((fMantissa - (iByte2 * (pow(2, 16)))) / (pow(2, 8)));
	
	iByte4 = (int)(fMantissa - (iByte2 * (pow(2, 16))) - (iByte3 * (pow(2, 8))));
	
	// subtract the sign bit if number is positive
	if (bNegative == false)
	{
		iByte2 = iByte2 & 0x7F;
	}
	return (uint32_t)((uint32_t)iByte1 << 24 | (uint32_t)iByte2 << 16 | (uint32_t)iByte3 << 8 | (uint32_t)iByte4);
}

uint16_t bq_ReadVoltage(struct TWI_Master *i2cHandler)
{
	uint8_t sendBuf = bq_CMD_voltage;
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf,1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	uint16_t bqVoltage = (uint16_t) (i2cHandler->readData[1] << 8) | (uint16_t) i2cHandler->readData[0] ;
	return bqVoltage;
}

int16_t bq_getCurrent(struct TWI_Master *i2cHandler)
{
	uint8_t sendBuf = bq_CMD_current;
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf,1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	int16_t bqCurrent = (int16_t) (i2cHandler->readData[1] << 8) | (int16_t) i2cHandler->readData[0] ;
	return bqCurrent;
}

uint8_t bq_getSOC(struct TWI_Master *i2cHandler)
{
	uint8_t sendBuf[1] = {bq_CMD_SOC};
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,1);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	return i2cHandler->readData[0];
}

uint16_t bq_getDesignCapacity(struct TWI_Master *i2cHandler)
{
	uint8_t sendBuf[1] = {bq_CMD_designCap};
	TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],3);
	while (i2cHandler->status != TWIM_STATUS_READY) {}// Wait until transaction is complete.
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	uint16_t designCap = (uint16_t) (i2cHandler->readData[1] << 8) | i2cHandler->readData[0];
	return designCap;
}

uint16_t bq_getFullCapacity(struct TWI_Master *i2cHandler)
{
	uint8_t sendBuf[1] = {bq_CMD_fullCap};
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	return (uint16_t) (i2cHandler->readData[1] << 8) | (uint16_t) i2cHandler->readData[0] ;
	//return bqFullCap;
}

uint16_t bq_getRemainingCapacity(struct TWI_Master *i2cHandler)
{
	uint8_t sendBuf[1] = {bq_CMD_remainingCap};
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	int16_t bqRemainingCap = (uint16_t) (i2cHandler->readData[1] << 8) | (uint16_t) i2cHandler->readData[0] ;
	return bqRemainingCap;
}

uint16_t bq_getChemID(struct TWI_Master *i2cHandler){
	uint8_t sendBuf[3] = {0x00, 0x08, 0x00};
	TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],3);
	while (i2cHandler->status != TWIM_STATUS_READY) {}// Wait until transaction is complete.
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	uint16_t chemID = (uint16_t) (i2cHandler->readData[1] << 8) | i2cHandler->readData[0];
	return chemID;
}

uint16_t bq_getPackTemperature(struct TWI_Master *i2cHandler){
	uint8_t sendBuf[1] = {bq_CMD_temperature};
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	uint16_t packTempK = (uint16_t) (i2cHandler->readData[1] << 8) | i2cHandler->readData[0];
	return packTempK;
}

uint16_t bq_getChipTemperature(struct TWI_Master *i2cHandler){
	uint8_t sendBuf[1] = {bq_CMD_intTemperature};
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	uint16_t icTempK = (uint16_t) (i2cHandler->readData[1] << 8) | i2cHandler->readData[0];
	return icTempK;
}

uint16_t bq_getFlags(struct TWI_Master *i2cHandler){
	uint8_t sendBuf[1] = {bq_CMD_flags};
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	uint16_t bqFlags = (uint16_t) (i2cHandler->readData[1] << 8) | i2cHandler->readData[0];
	return bqFlags;
}

uint16_t bq_getFlagsB(struct TWI_Master *i2cHandler){
	uint8_t sendBuf[1] = {bq_CMD_flagsB};
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	uint16_t bqFlags = (uint16_t) (i2cHandler->readData[1] << 8) | i2cHandler->readData[0];
	return bqFlags;
}

uint16_t bq_GetStatus(struct TWI_Master *i2cHandler){
	uint8_t sendBuf[3] = {bq_CMD_control, bq_CMD_status, 0x00};
	TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],3);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	
	uint16_t controlStatus = (uint16_t) (i2cHandler->readData[1] << 8) | (uint16_t) i2cHandler->readData[0] ;
	return controlStatus;
}

uint16_t bq_getFirmwareVersion(struct TWI_Master *i2cHandler){
	uint8_t sendBuf[3] = {0x00, 0x02, 0x00};
	TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],3);
	while (i2cHandler->status != TWIM_STATUS_READY) {}// Wait until transaction is complete.
	TWI_MasterWriteRead(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],1,2);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	uint16_t firmwareVersion = (uint16_t) (i2cHandler->readData[1] << 8) | i2cHandler->readData[0];
	return firmwareVersion;
}

void bq_setDesignCapacity(struct TWI_Master *i2cHandler, uint16_t capacity){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 48, 0x00);
	uint8_t packCapacityLSB = 0xF8;
	uint8_t packCapacityMSB = 0x2A;
	
	//uint8_t packCapacityMSB = (capacity >> 8);
	//uint8_t packCapacityLSB = (capacity & 0xFF);
	bq_ModifyFlash(i2cHandler, flashBytes, 11, packCapacityMSB);
	bq_ModifyFlash(i2cHandler, flashBytes, 12, packCapacityLSB);
}

void bq_setDesignEnergy(struct TWI_Master *i2cHandler, uint16_t energy){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 48, 0x00);
	uint8_t energyMSB = (energy >> 8);
	uint8_t energyLSB = (energy & 0xFF);
	bq_ModifyFlash(i2cHandler, flashBytes, 13, energyMSB);
	bq_ModifyFlash(i2cHandler, flashBytes, 14, energyLSB);
}

void bq_setDesignEnergyScale(struct TWI_Master *i2cHandler, uint8_t scale){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 48, 0x00);
	bq_ModifyFlash(i2cHandler, flashBytes, 30, scale);
}

void bq_setVoltageDivider(struct TWI_Master *i2cHandler, uint16_t divider){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 104, 0x00);
	uint8_t voltageDividerMSB = (divider >> 8);
	uint8_t voltageDividerLSB = (divider & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 14, voltageDividerMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 15, voltageDividerLSB);
}

void bq_setExternalTempOffset(struct TWI_Master *i2cHandler, uint8_t offset){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 104, 0x00);
	bq_ModifyFlash(i2cHandler,flashBytes, 12, offset);
}

void bq_setCellCount(struct TWI_Master *i2cHandler, uint8_t count){
	bq_ModifyFlash(i2cHandler, bq_ReadFlashBlock(i2cHandler, 64, 0x00), 7, count);
}

void bq_setSleepMode(struct TWI_Master *i2cHandler, bool sleepMode){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 64, 0x00);
	uint8_t config = flashBytes[1];
	if(sleepMode){ config  |= (sleepMode << 6);}
	else{config &= ~(!sleepMode << 6);}
	bq_ModifyFlash(i2cHandler, flashBytes, 1, config);	
}

void bq_setTempExternal(struct TWI_Master *i2cHandler, bool TS){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 64, 0x00);
	uint8_t config = *(flashBytes+1) | TS; 
	bq_ModifyFlash(i2cHandler, flashBytes, 1, config);
}

void bq_setNi_dTterm(struct TWI_Master *i2cHandler, bool nidt){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 64, 0x00);
	uint8_t config = flashBytes[1];
	if(nidt){ config  |= (nidt << 4);}
	else{config &= ~(!nidt << 4);}
	bq_ModifyFlash(i2cHandler, flashBytes, 1, config);
}

void bq_setNi_dVterm(struct TWI_Master *i2cHandler, bool nidv){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 64, 0x00);
	uint8_t config = flashBytes[1];
	if(nidv){ config  |= (nidv << 3);}
	else{config &= ~(!nidv << 3);}
	bq_ModifyFlash(i2cHandler, flashBytes, 1, config);
}

void bq_setLoadMode(struct TWI_Master *i2cHandler, uint8_t loadmode){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 80, 0x00);
	bq_ModifyFlash(i2cHandler, flashBytes, 1, loadmode);
}

void bq_setLoadSelect(struct TWI_Master *i2cHandler, uint8_t loadselect){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 80, 0x00);
	bq_ModifyFlash(i2cHandler, flashBytes, 0, loadselect);
}

void bq_setCellTerminateVoltage(struct TWI_Master *i2cHandler, uint16_t ctv){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 80, 0x01);
	uint8_t ctvMSB = (ctv >> 8);
	uint8_t ctvLSB = (ctv & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 21, ctvMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 22, ctvLSB);
}

void bq_setQuitCurrent(struct TWI_Master *i2cHandler, uint16_t quitCurrent){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 81, 0x00);
	uint8_t quitCurrentMSB = (quitCurrent >> 8);
	uint8_t quitCurrentLSB = (quitCurrent & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 4, quitCurrentMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 5, quitCurrentLSB);
}

void bq_setQmaxCell0(struct TWI_Master *i2cHandler, uint16_t qMaxCell0){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 82, 0x00);
	uint8_t qMaxCell0MSB = (qMaxCell0 >> 8);
	uint8_t qMaxCell0LSB = (qMaxCell0 & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 0, qMaxCell0MSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 1, qMaxCell0LSB);
}

void bq_setCellBLsetThreshold(struct TWI_Master *i2cHandler, uint16_t cellBLsetthres){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 49, 0x00);
	uint8_t cellBLsetthresMSB = (cellBLsetthres >> 8);
	uint8_t cellBLsetthresLSB = (cellBLsetthres & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 8, cellBLsetthresMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 9, cellBLsetthresLSB);
}

void bq_setCellBLclearThreshold(struct TWI_Master *i2cHandler, uint16_t cellBLclearthres){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 49, 0x00);
	uint8_t cellBLclearthresMSB = (cellBLclearthres >> 8);
	uint8_t cellBLclearthresLSB = (cellBLclearthres & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 11, cellBLclearthresMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 12, cellBLclearthresLSB);
}

void bq_setCellBHsetThreshold(struct TWI_Master *i2cHandler, uint16_t cellBHsetthres){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 49, 0x00);
	uint8_t cellBHsetthresMSB = (cellBHsetthres >> 8);
	uint8_t cellBHsetthresLSB = (cellBHsetthres & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 13, cellBHsetthresMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 14, cellBHsetthresLSB);
}

void bq_setCellBHclearThreshold(struct TWI_Master *i2cHandler, uint16_t cellBHclearthres){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 49, 0x00);
	uint8_t cellBHclearthresMSB = (cellBHclearthres >> 8);
	uint8_t cellBHclearthresLSB = (cellBHclearthres & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 16, cellBHclearthresMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 17, cellBHclearthresLSB);
}

void bq_setSOC1setThreshold(struct TWI_Master *i2cHandler, uint16_t soc1setthres){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 49, 0x00);
	uint8_t soc1setthresMSB = (soc1setthres >> 8);
	uint8_t soc1setthresLSB = (soc1setthres & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 0, soc1setthresMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 1, soc1setthresLSB);
}

void bq_setSOC1clearThreshold(struct TWI_Master *i2cHandler, uint16_t soc1clearthres){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 49, 0x00);
	uint8_t soc1clearthresMSB = (soc1clearthres >> 8);
	uint8_t soc1clearthresLSB = (soc1clearthres & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 2, soc1clearthresMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 3, soc1clearthresLSB);
}

void bq_setSOCFsetThreshold(struct TWI_Master *i2cHandler, uint16_t socfsetthres){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 49, 0x00);
	uint8_t socfsetthresMSB = (socfsetthres >> 8);
	uint8_t socfsetthresLSB = (socfsetthres & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 4, socfsetthresMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 5, socfsetthresLSB);
}

void bq_setSOCFclearThreshold(struct TWI_Master *i2cHandler, uint16_t socfclearthres){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 49, 0x00);
	uint8_t socfclearthresMSB = (socfclearthres >> 8);
	uint8_t socfclearthresLSB = (socfclearthres & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 6, socfclearthresMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 7, socfclearthresLSB);
}

void bq_setCellChargeTerminationVoltage(struct TWI_Master *i2cHandler, uint16_t cellVterm){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 82, 0x00);
	uint8_t cellVtermMSB = (cellVterm >> 8);
	uint8_t cellVtermLSB = (cellVterm & 0xFF);
	bq_ModifyFlash(i2cHandler,flashBytes, 5, cellVtermMSB);
	bq_ModifyFlash(i2cHandler,flashBytes, 6, cellVtermLSB);
}

void bq_setVoltSel(struct TWI_Master *i2cHandler, bool voltsel){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 64, 0x00);
	uint8_t config = flashBytes[0];
	if(voltsel){ config  |= (voltsel << 3);}
	else{config &= ~(!voltsel << 3);}
	bq_ModifyFlash(i2cHandler, flashBytes, 0, config);
}

void bq_calibrateCurrent(struct TWI_Master *i2cHandler, int16_t current){
	uint8_t *flashBytes = bq_ReadFlashBlock(i2cHandler, 104, 0x00);
	uint8_t ccgainbytes[4];
	uint8_t ccdeltabytes[4];
	
	uint32_t curentGain = ((uint32_t) *(flashBytes+3))<<24 | ((uint32_t) *(flashBytes +2))<<16|((uint32_t) *(flashBytes+1))<<8|(uint32_t) *(flashBytes);
	float currentGainResistance = (4.768/bq_xemicsToFloat(curentGain));

	float newGain = (((float)bq_getCurrent(i2cHandler))/((float)current)) * currentGainResistance;
	
	float GainDF = 4.768/newGain;
	float DeltaDF = 5677445/newGain;

	uint32_t newCCgainByte = bq_floatToXemics(GainDF);
	ccgainbytes[0]= newCCgainByte & 0xFF;
	ccgainbytes[1]= (newCCgainByte & 0xFF00) >> 8;
	ccgainbytes[2]= (newCCgainByte & 0xFF0000) >> 16;
	ccgainbytes[3]= (newCCgainByte & 0xFF000000) >> 24;
	bq_ModifyFlash(i2cHandler, flashBytes, 0, ccgainbytes[3]);
	bq_ModifyFlash(i2cHandler, flashBytes, 1, ccgainbytes[2]);
	bq_ModifyFlash(i2cHandler, flashBytes, 2, ccgainbytes[1]);
	bq_ModifyFlash(i2cHandler, flashBytes, 3, ccgainbytes[0]);
	
	uint32_t newCCdeltaByte = bq_floatToXemics(DeltaDF);
	ccdeltabytes[0]= newCCdeltaByte & 0xFF;
	ccdeltabytes[1]= (newCCdeltaByte & 0xFF00) >> 8;
	ccdeltabytes[2]= (newCCdeltaByte & 0xFF0000) >> 16;
	ccdeltabytes[3]= (newCCdeltaByte & 0xFF000000) >> 24;
	bq_ModifyFlash(i2cHandler, flashBytes, 4, ccdeltabytes[3]);
	bq_ModifyFlash(i2cHandler, flashBytes, 5, ccdeltabytes[2]);
	bq_ModifyFlash(i2cHandler, flashBytes, 6, ccdeltabytes[1]);
	bq_ModifyFlash(i2cHandler, flashBytes, 7, ccdeltabytes[0]);
}

void bq_enableIT(struct TWI_Master *i2cHandler){
	uint8_t sendBuf[3] = {bq_CMD_control, bq_CMD_enableIT, 0x00};
	TWI_MasterWrite(i2cHandler,BQ34Z100_ADDR,&sendBuf[0],3);
	while (i2cHandler->status != TWIM_STATUS_READY) {} //Wait until transaction is complete.
	
}

void bq_unseal(struct TWI_Master *i2cHandler){
	//Unseal Fuel Gauge
	uint8_t sendBuf[3] = {0x00, 0x14, 0x04};
	TWI_MasterWrite(i2cHandler, BQ34Z100_ADDR,&sendBuf[0],3);
	while (i2cHandler->status != TWIM_STATUS_READY) {}// Wait until transaction is complete.
	sendBuf[0] = 0x00;
	sendBuf[1] = 0x72;
	sendBuf[2] = 0x36;
	TWI_MasterWrite(i2cHandler, BQ34Z100_ADDR,&sendBuf[0],3);
	while (i2cHandler->status != TWIM_STATUS_READY) {}// Wait until transaction is complete.
}

void bq_fullUnseal(struct TWI_Master *i2cHandler){
	//Full unseal Fuel Gauge
	uint8_t sendBuf[3] = {0xFF, 0xFF, 0xFF};
	TWI_MasterWrite(i2cHandler, BQ34Z100_ADDR,&sendBuf[0],3);
	while (i2cHandler->status != TWIM_STATUS_READY) {}// Wait until transaction is complete.
	sendBuf[0] = 0x00;
	sendBuf[1] = 0xFF;
	sendBuf[2] = 0xFF;
	TWI_MasterWrite(i2cHandler, BQ34Z100_ADDR,&sendBuf[0],3);
	while (i2cHandler->status != TWIM_STATUS_READY) {}// Wait until transaction is complete.
}