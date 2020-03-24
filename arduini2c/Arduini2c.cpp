/**
 * I2cInterface.cpp - Useful I2c wrapper for Wire
 */

#include "Arduini2c.h"

#include <Wire.h>

bool Arduini2c::isWireBegun = false;

void Arduini2c::begin()
{
	if (!isWireBegun) {
		Wire.begin();
		isWireBegun = true;
	}
}

/*!
 *  Write byte to register
 */
void Arduini2c::writeByte(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

void Arduini2c::writeByte(uint8_t value)
{
	Wire.beginTransmission(address);
	Wire.write(value);
	Wire.endTransmission();
}


/*!
 *  Read byte from register with no checking. I'm not sure this is ok. It works on the adxl though.
 */
uint8_t Arduini2c::fastReadByte(uint8_t reg)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(address, uint8_t(1));
	return Wire.read();
}

uint8_t Arduini2c::fastReadByte()
{
	Wire.requestFrom(address, uint8_t(1));
	return Wire.read();
}


/*!
 *  Read byte from register
 */
uint8_t Arduini2c::readByte(uint8_t reg)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(address, uint8_t(1));
	while(!Wire.available()) {};
	return Wire.read();
}

uint8_t Arduini2c::readByte()
{
	Wire.requestFrom(address, uint8_t(1));
	while(!Wire.available()) {};
	return Wire.read();
}


/*!
 *  Read 16 bit signed int from register
 */
int16_t Arduini2c::readInt(uint8_t reg)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();

	Wire.requestFrom(address, uint8_t(2));
	while(!Wire.available()) {};
	uint8_t vla = Wire.read();
	uint8_t vha = Wire.read();
	return vha << 8 | vla;
}

/*!
 * Read a whole heap of stuff! Unsafe! Useful!
 * \param reg the register (or operation on some chips)
 * \param n number of bytes to read
 * \param buf where to sticl the results
 */
void Arduini2c::readBuf(uint8_t reg, size_t n, uint8_t *buf)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();

	Wire.requestFrom(address, n);
	while(!Wire.available()) {};
	for (uint8_t i=0; i<n; i++) {
		buf[i] = Wire.read();
	}
}

uint8_t Arduini2c::writeBit(uint8_t reg, uint8_t pos, bool state)
{
	uint8_t value = readByte(reg);
	if (state) {
		value |= (1 << pos);
	} else {
		value &= ~(1 << pos);
	}
	writeByte(reg, value);
	return value;
}

bool Arduini2c::readBit(uint8_t reg, uint8_t pos)
{
	uint8_t value = readByte(reg);
	return ((value >> pos) & 1);
}


