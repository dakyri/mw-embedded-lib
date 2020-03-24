#pragma once
/**
 * I2cInterface.h - Useful I2c wrapper for Wire
 */
#include "Arduino.h"

#include <SPI.h>

class Arduispi {
public:
	Arduispi(const uint8_t p_csPin): csPin(p_csPin) {}

	void begin(void);

	void writeByte(uint8_t reg, uint8_t value);
	void writeByte(uint8_t value);
	uint8_t readByte(uint8_t reg);
	uint8_t readByte();
	uint8_t fastReadByte(uint8_t reg);
	uint8_t fastReadByte();
	int16_t readInt(uint8_t reg);
	void readBuf(uint8_t reg, size_t n, uint8_t *buf);
	uint8_t writeBit(uint8_t reg, uint8_t pos, bool state);
	bool readBit(uint8_t reg, uint8_t pos);

	const uint8_t csPin;
private:
	static bool isSpiBegun;
};


