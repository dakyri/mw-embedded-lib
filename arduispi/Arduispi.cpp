/**
 * Arduispi. An Arduino spi interface that parallels the Arduini2c interface. Might not work in all cases.
 */

#include "Arduinispi.h"

bool Arduispi::isSpiBegun = false;

void Arduispi::begin()
{
	if (!isSpiBegun) {
		isSpiBegun = true;
		pinMode(csPin, OUTPUT);
   	    digitalWrite(csPin, HIGH);
		SPI.setDataMode(SPI_MODE0);
		SPI.setClockDivider(SPI_CLOCK_DIV4);
		SPI.setBitOrder(MSBFIRST);
		SPI.begin();
		//SPI.setClockDivider(32);
	}
}

/*!
 *  Write byte to register
 */
void Arduispi::writeByte(uint8_t reg, uint8_t value)
{
	::digitalWrite(csPin, LOW);
	delayMicroseconds(10);
	SPI.transfer(address<<3);
	SPI.transfer(value);
	delayMicroseconds(10);
	::digitalWrite(csPin, HIGH);
}

void Arduispi::writeByte(uint8_t value)
{
}


/*!
 *  Read byte from register with no checking. I'm not sure this is ok. It works on the adxl though.
 */
uint8_t Arduispi::fastReadByte(uint8_t reg)
{
	return readByte(reg);
}

uint8_t Arduispi::fastReadByte()
{
	return readByte();
}


/*!
 *  Read byte from register
 */
uint8_t Arduispi::readByte(uint8_t reg)
{
	digitalWrite(csPin, LOW);
	delayMicroseconds(10);
	SPI.transfer(0x80|(reg<<3));
	const uin8_t result = SPI.transfer(0xff);
	delayMicroseconds(10);
	digitalWrite(csPin, HIGH);
	return result;
}

uint8_t Arduispi::readByte()
{
	return 0;
}


/*!
 *  Read 16 bit signed int from register
 */
int16_t Arduispi::readInt(uint8_t reg)
{
	return 0;
}

/*!
 * Read a whole heap of stuff! Unsafe! Useful!
 * \param reg the register (or operation on some chips)
 * \param n number of bytes to read
 * \param buf where to sticl the results
 */
void Arduispi::readBuf(uint8_t reg, size_t n, uint8_t *buf)
{
}

uint8_t Arduispi::writeBit(uint8_t reg, uint8_t pos, bool state)
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

bool Arduispi::readBit(uint8_t reg, uint8_t pos)
{
	uint8_t value = readByte(reg);
	return ((value >> pos) & 1);
}


