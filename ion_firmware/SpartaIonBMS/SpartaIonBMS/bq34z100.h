/*
 * bq34z100.h
 *
 * Created: 19-9-2020 14:01:06
 *  Author: T. Ligeon
 * Driver for TI BQ34Z100 Fuel Gauge IC
 */ 


#ifndef BQ34Z100_H_
#define BQ34Z100_H_


uint16_t bq_ReadVoltage(struct TWI_Master *i2cHandler);

int16_t bq_getCurrent(struct TWI_Master *i2cHandler);

uint16_t bq_GetStatus(struct TWI_Master *i2cHandler);

uint16_t bq_getFirmwareVersion(struct TWI_Master *i2cHandler);

uint16_t bq_getChemID(struct TWI_Master *i2cHandler);

uint16_t bq_getPackTemperature(struct TWI_Master *i2cHandler);

uint16_t bq_getChipTemperature(struct TWI_Master *i2cHandler);

uint16_t bq_getFlags(struct TWI_Master *i2cHandler);

uint16_t bq_getFlagsB(struct TWI_Master *i2cHandler);

uint8_t bq_getSOC(struct TWI_Master *i2cHandler);

uint16_t bq_getDesignCapacity(struct TWI_Master *i2cHandler);

uint16_t bq_getFullCapacity(struct TWI_Master *i2cHandler);

uint16_t bq_getRemainingCapacity(struct TWI_Master *i2cHandler);

void bq_setDesignCapacity(struct TWI_Master *i2cHandler, uint16_t capacity);

void bq_setDesignEnergy(struct TWI_Master *i2cHandler, uint16_t energy);

void bq_setDesignEnergyScale(struct TWI_Master *i2cHandler, uint8_t scale);

void bq_setVoltageDivider(struct TWI_Master *i2cHandler, uint16_t divider);

void bq_setExternalTempOffset(struct TWI_Master *i2cHandler, uint8_t offset);

void bq_setCellCount(struct TWI_Master *i2cHandler, uint8_t count);

void bq_setSleepMode(struct TWI_Master *i2cHandler, bool sleepMode);

void bq_setTempExternal(struct TWI_Master *i2cHandler, bool TS);

void bq_setNi_dTterm(struct TWI_Master *i2cHandler, bool nidt);

void bq_setNi_dVterm(struct TWI_Master *i2cHandler, bool nidv);

void bq_setLoadMode(struct TWI_Master *i2cHandler, uint8_t loadmode);

void bq_setLoadSelect(struct TWI_Master *i2cHandler, uint8_t loadselect);

void bq_setVoltSel(struct TWI_Master *i2cHandler, bool voltsel);

void bq_setCellTerminateVoltage(struct TWI_Master *i2cHandler, uint16_t ctv);

void bq_setQuitCurrent(struct TWI_Master *i2cHandler, uint16_t quitCurrent);

void bq_setQmaxCell0(struct TWI_Master *i2cHandler, uint16_t qMaxCell0);

void bq_setCellChargeTerminationVoltage(struct TWI_Master *i2cHandler, uint16_t cellVterm);

void bq_setCellBLsetThreshold(struct TWI_Master *i2cHandler, uint16_t cellBLsetthres);

void bq_setCellBLclearThreshold(struct TWI_Master *i2cHandler, uint16_t cellBLclearthres);

void bq_setCellBHsetThreshold(struct TWI_Master *i2cHandler, uint16_t cellBHsetthres);

void bq_setCellBHclearThreshold(struct TWI_Master *i2cHandler, uint16_t cellBHclearthres);

void bq_setSOC1setThreshold(struct TWI_Master *i2cHandler, uint16_t soc1setthres);

void bq_setSOC1clearThreshold(struct TWI_Master *i2cHandler, uint16_t soc1clearthres);

void bq_setSOCFsetThreshold(struct TWI_Master *i2cHandler, uint16_t socfsetthres);

void bq_setSOCFclearThreshold(struct TWI_Master *i2cHandler, uint16_t socfclearthres);

void bq_calibrateCurrent(struct TWI_Master *i2cHandler, int16_t current);

void bq_enableIT(struct TWI_Master *i2cHandler);

void bq_unseal(struct TWI_Master *i2cHandler);

void bq_fullUnseal(struct TWI_Master *i2cHandler);

void bq_Reset(struct TWI_Master *i2cHandler);

uint8_t * bq_ReadFlashBlock(struct TWI_Master *i2cHandler, uint8_t class, uint8_t offset);

void bq_ModifyFlash(struct TWI_Master *i2cHandler, uint8_t* dataFlashBytes ,uint8_t index, int value);

void bq_UpdateChecksum(struct TWI_Master *i2cHandler, uint8_t* dataFlashBytes);

float bq_xemicsToFloat(uint32_t X);

uint32_t bq_floatToXemics(float X);

#endif /* BQ34Z100_H_ */