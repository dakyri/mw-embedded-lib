#pragma once

#ifdef HAS_WIRING_PI
#include <wiringpi.h>
#include <wiringPiI2C.h>
#endif

/*!
 * simple wrapper around wiring pi i2c so we can use the arduino code on raspberry pi
 */
class Pi2c {
public:
	Pi2c(const uint8_t _address): address(_address) {}

	~Pi2c() {
#ifdef HAS_WIRING_PI
		if (dfd >= 0) {
			wiringPiI2CClose(dfd);
		}
#endif
	}

	bool begin() {
#ifdef HAS_WIRING_PI
		if (!hasBegun) {
			wiringPiSetup();
			hasBegun = true;
		}
		dfd = wiringPiI2CSetup(address);
#endif
		return dfd >= 0;
	}


	/*!
	 *  Write byte to register
	 */
	void writeByte(uint8_t reg, uint8_t value) {
#ifdef HAS_WIRING_PI
		wiringPiI2CWriteReg8(dfd, reg, value);
#endif
	}
	void writeByte(uint8_t value) {
#ifdef HAS_WIRING_PI
		wiringPiI2CWrite(dfd, value);
#endif
	}

	/*!
	 *  Read byte from register
	 */
	uint8_t readByte(uint8_t reg) {
#ifdef HAS_WIRING_PI
		return wiringPiI2CRead8(dfd, reg);
#endif
}
	uint8_t readByte() {
#ifdef HAS_WIRING_PI
		return wiringPiI2CRead(dfd);
#endif
	}

	/*!
	 * Read/write whole heap of stuff!
	 * \param reg the register (or operation on some chips)
	 * \param n number of bytes to read
	 * \param buf where to sticl the results
	 */
	int readBuf(uint8_t reg, size_t n, uint8_t *buf) {
#ifdef HAS_WIRING_PI
		return wiringPiI2CReadBlockData (fd, reg, *buf, n);
#endif
	}
	int writeBuf(uint8_t reg, size_t n, uint8_t *buf) {
#ifdef HAS_WIRING_PI
		return wiringPiI2CWriteBlockData (fd, reg, *buf, n); 
#endif
	}

	/*
	uint8_t fastReadByte(uint8_t reg);
	uint8_t fastReadByte();
	int16_t readInt(uint8_t reg);
	uint8_t writeBit(uint8_t reg, uint8_t pos, bool state);
	bool readBit(uint8_t reg, uint8_t pos);
	
	static bool check(const uint8_t addr);
	static void scan(uint8_t ports[], uint8_t &nFound);
	*/

	const uint8_t address;
	const int dfd = -1;
private:
	static bool hasBegun;
};



/*!
 *  Read byte from register with no checking. I'm not sure this is ok. It works on the adxl though.
 */
/*
uint8_t Pi2c::fastReadByte(uint8_t reg)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(address, uint8_t(1));
	return Wire.read();
}

uint8_t Pi2c::fastReadByte()
{
	Wire.requestFrom(address, uint8_t(1));
	return Wire.read();
}
*/


/*!
 *  Read 16 bit signed int from register
 */
/*
int16_t Pi2c::readInt(uint8_t reg)
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
*/

/*
uint8_t Pi2c::writeBit(uint8_t reg, uint8_t pos, bool state)
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

bool Pi2c::readBit(uint8_t reg, uint8_t pos)
{
	uint8_t value = readByte(reg);
	return ((value >> pos) & 1);
}
*/