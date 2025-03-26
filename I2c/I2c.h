#pragma once
/**
 * I2cInterface.h - Useful I2c wrapper for Wire
 */
#include "Arduino.h"

class I2c {
public:
	I2c(const uint8_t p_address): address(p_address) {}

	bool begin();

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

	static bool check(const uint8_t addr);
	static void scan(uint8_t ports[], uint8_t &nFound);

	const uint8_t address;
private:
	static bool isWireBegun;
};

void print_hex_byte(uint8_t x, bool andEol=true);


