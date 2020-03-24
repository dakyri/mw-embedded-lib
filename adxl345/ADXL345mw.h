#pragma once

/**
 * ADXL345mw.h - Header file for stripped back access to ADXL345 for anything that we can make a common interface for
 * (SPI, I2C and any reasonable platform)
 */

#include <stdint.h> // arduino doesn't recognise <cstdint>

#include "../mw-common/defs.h"

namespace adxl345 {

class Vector
{
public:
	Vector(int16_t _x=0, int16_t _y=0, int16_t _z=0, float n=1) : x(_x * n), y(_y * n), z(_z * n) {}
	void lowPassFilter(Vector vector, float alpha = 0.5) {
		x = vector.x * alpha + (x * (1.0 - alpha));
		y = vector.y * alpha + (y * (1.0 - alpha));
		z = vector.z * alpha + (z * (1.0 - alpha));
	}
	float x;
	float y;
	float z;
};

static const uint8_t kAddress1 = 0x53;
static const uint8_t kAddress2 = 0x1d;
static const uint8_t kRegDevid = 0x00;
static const uint8_t kRegThreshTap = 0x1D; // 1
static const uint8_t kRegOfsX = 0x1E;
static const uint8_t kRegOfsY = 0x1F;
static const uint8_t kRegOfsZ = 0x20;
static const uint8_t kRegDur = 0x21; // 2
static const uint8_t kRegLatent = 0x22; // 3
static const uint8_t kRegWindow = 0x23; // 4
static const uint8_t kRegThreshAct = 0x24; // 5
static const uint8_t kRegThreshInact = 0x25; // 6
static const uint8_t kRegTimeInact = 0x26; // 7
static const uint8_t kRegActInactCtl = 0x27;
static const uint8_t kRegThreshFreefall = 0x28; // 8
static const uint8_t kRegTimeFreefall = 0x29; // 9
static const uint8_t kRegTapAxes = 0x2A;
static const uint8_t kRegActTapStatus = 0x2B;
static const uint8_t kRegActBwCtl = 0x2C;
static const uint8_t kRegActPowerCtl = 0x2D;
static const uint8_t kRegIntEnable = 0x2E;
static const uint8_t kRegIntMap = 0x2F;
static const uint8_t kRegIntSource = 0x30; // A
static const uint8_t kRegDataFormat = 0x31;
static const uint8_t kRegDataX0 = 0x32;
static const uint8_t kRegDataX1 = 0x33;
static const uint8_t kRegDataY0 = 0x34;
static const uint8_t kRegDataY1 = 0x35;
static const uint8_t kRegDataZ0 = 0x36;
static const uint8_t kRegDataZ1 = 0x37;
static const uint8_t kRegFifoCtl = 0x38;
static const uint8_t kRegFifoStatus = 0x39;

static const float kGravitySun = 273.95f;
static const float kGravityEarth = 9.80665f;
static const float kGravityMoon = 1.622f;
static const float kGravityMars = 3.69f;
static const float kGravityNone = 1.00f;

enum dataRate_t
{
	kDatarate3200HZ	= 0b1111,
	kDatarate1600HZ	= 0b1110,
	kDatarate800HZ	= 0b1101,
	kDatarate400HZ	= 0b1100,
	kDatarate200HZ	= 0b1011,
	kDatarate100HZ	= 0b1010,
	kDatarate50HZ	= 0b1001,
	kDatarate25HZ	= 0b1000,
	kDatarate12_5HZ	= 0b0111,
	kDatarate6_25HZ	= 0b0110,
	kDatarate3_13HZ	= 0b0101,
	kDatarate1_56HZ	= 0b0100,
	kDatarate0_78HZ	= 0b0011,
	kDatarate0_39HZ	= 0b0010,
	kDatarate0_20HZ	= 0b0001,
	kDatarate0_10HZ	= 0b0000
};

enum int_t
{
	kInt2 = 0b01,
	kInt1 = 0b00
};

enum activity_t
{
	kDataReady	= 1 << 7,
	kSingleTap	= 1 << 6,
	kDoubleTap	= 1 << 5,
	kActivity	= 1 << 4,
	kInactivity	= 1 << 3,
	kFreeFall	= 1 << 2,
	kWatermark	= 1 << 1,
	kOverrun	= 1 << 0
};

enum range_t
{
	kRange16G	= 0b11,
	kRange8G	= 0b10,
	kRange4G	= 0b01,
	kRange2G	= 0b00
};

struct Activites
{
	bool isOverrun;
	bool isWatermark;
	bool isFreeFall;
	bool isInactivity;
	bool isActivity;
	bool isActivityOnX;
	bool isActivityOnY;
	bool isActivityOnZ;
	bool isDoubleTap;
	bool isTap;
	bool isTapOnX;
	bool isTapOnY;
	bool isTapOnZ;
	bool isDataReady;
};

template <typename T> class Interface
{
public:
	template <typename ... IOArgs> Interface(IOArgs ... args): isValid(false), io(args ...) {}

	bool begin()  {
		io.begin();
		if (io.fastReadByte(kRegDevid) != 0xE5) { // Check ADXL345 DEVID reg. returns a fixed value 0xe5
			isValid = false;
		} else {
			isValid = true;
			io.writeByte(kRegActPowerCtl, 0x08);	// Enable measurement mode (0b00001000)
			clearSettings();
		}
		return isValid;

	void clearSettings(void) {
		setRange(kRange2G);
		setDataRate(kDatarate100HZ);

		io.writeByte(kRegThreshTap, 0x00);
		io.writeByte(kRegDur, 0x00);
		io.writeByte(kRegLatent, 0x00);
		io.writeByte(kRegWindow, 0x00);
		io.writeByte(kRegThreshAct, 0x00);
		io.writeByte(kRegThreshInact, 0x00);
		io.writeByte(kRegTimeInact, 0x00);
		io.writeByte(kRegThreshFreefall, 0x00);
		io.writeByte(kRegTimeFreefall, 0x00);

		uint8_t value = io.readByte(kRegActInactCtl);
		value &= 0b10001000;
		io.writeByte(kRegActInactCtl, value);

		value = io.readByte(kRegTapAxes);
		value &= 0b11111000;
		io.writeByte(kRegTapAxes, value);
	}

	void readRaw(int16_t &x, int16_t &y, int16_t &z) {
		x = io.readInt(kRegDataX0);
		y = io.readInt(kRegDataY0);
		z = io.readInt(kRegDataZ0);
	}

	Vector readNormalize(float gravityFactor = kGravityEarth) {
		int16_t x, y, z;
		readRaw(x, y, z);
		return Vector(x, y, z, 0.004f * gravityFactor);
	}

	Vector readScaled(void) {
		int16_t x, y, z;
		readRaw(x, y, z);
		return Vector(x, y, z, 0.004f);
	}

	Activites readActivites(void) {
		Activites a;
		uint8_t data = io.readByte(kRegIntSource);
		a.isOverrun = (data & kOverrun);
		a.isWatermark = (data & kWatermark);
		a.isFreeFall = (data & kFreeFall);
		a.isInactivity = (data & kInactivity);
		a.isActivity = (data & kActivity);
		a.isDoubleTap = (data & kDoubleTap);
		a.isTap = (data & kSingleTap);
		a.isDataReady = (data & kDataReady);

		data = io.readByte(kRegActTapStatus);
		a.isActivityOnX = (data & (6 << 1));
		a.isActivityOnY = (data & (5 << 1));
		a.isActivityOnZ = (data & (4 << 1));
		a.isTapOnX = (data & (2 << 1));
		a.isTapOnY = (data & (1 << 1));
		a.isTapOnZ = (data & (0 << 1));

		return a;
	}

	void setRange(range_t range) {
		uint8_t value = io.readByte(kRegDataFormat);
		// Update the data rate: (& 0xF0 - Leave HSB, | 0b000xx - range, | 0x08 set full res
		value = (value & 0xF0) | range | 0x08;
		io.writeByte(kRegDataFormat, value);
	}
	range_t getRange(void) { return (range_t)(io.readByte(kRegDataFormat) & 0x03); }

	void setDataRate(dataRate_t dataRate) { io.writeByte(kRegActBwCtl, dataRate); }
	dataRate_t getDataRate(void) { return (dataRate_t)(io.readByte(kRegActBwCtl) & 0x0F); }
	/*! Set Tap Threshold (62.5mg / LSB) */
	void setTapThreshold(float threshold) {
		io.writeByte(kRegThreshTap, mw::pin<uint8_t>(threshold / 0.0625f, 0, 255)); }
	float getTapThreshold(void) { return io.readByte(kRegThreshTap) * 0.0625f; } /*!< Set Tap Threshold (62.5mg / LSB) */
	/*! Set Tap Duration (625us / LSB) */
	void setTapDuration(float duration) { io.writeByte(kRegDur, mw::pin<uint8_t>(duration / 0.000625f, 0, 255)); }
	float getTapDuration(void) { return io.readByte(kRegDur) * 0.000625f; } /*!< Get Tap Duration (625us / LSB) */
	/*! Set Double Tap Latency (1.25ms / LSB) */
	void setDoubleTapLatency(float latency) { io.writeByte(kRegLatent, mw::pin<uint8_t>(latency / 0.00125f, 0, 255)); }
	float getDoubleTapLatency() { return io.readByte(kRegLatent) * 0.00125f; } /*!<  Get Double Tap Latency (1.25ms / LSB) */
	/*! Set Double Tap Window (1.25ms / LSB) */
	void setDoubleTapWindow(float window) { io.writeByte(kRegWindow, mw::pin<uint8_t>(window / 0.00125f, 0, 255)); }
	float getDoubleTapWindow(void) { return io.readByte(kRegWindow) * 0.00125f; } /*!< Get Double Tap Window (1.25ms / LSB) */
	/*! Set Activity Threshold (62.5mg / LSB) */
	void setActivityThreshold(float threshold) { io.writeByte(kRegThreshAct, mw::pin<uint8_t>(threshold / 0.0625f, 0, 255)); }
	float getActivityThreshold(void) { return io.readByte(kRegThreshAct) * 0.0625f; } /*!< Get Activity Threshold (65.5mg / LSB) */
	/*! Set Inactivity Threshold (65.5mg / LSB) */
	void setInactivityThreshold(float threshold) { io.writeByte(kRegThreshInact, mw::pin<uint8_t>(threshold / 0.0625f, 0, 255)); }
	float getInactivityThreshold(void) { return io.readByte(kRegThreshInact) * 0.0625f; } /*!< Get Incactivity Threshold (65.5mg / LSB) */
	/*! Set Inactivity Time (s / LSB) */
	void setTimeInactivity(uint8_t time) { io.writeByte(kRegTimeInact, time); }
	uint8_t getTimeInactivity(void) { return io.readByte(kRegTimeInact); } /*!< Get Inactivity Time (s / LSB) */
	/*! Set Free Fall Threshold (65.5mg / LSB) */
	void setFreeFallThreshold(float threshold) { io.writeByte(kRegThreshFreefall, mw::pin<uint8_t>(threshold / 0.0625f, 0, 255)); }
	float getFreeFallThreshold(void) { return io.readByte(kRegThreshFreefall) * 0.0625f; } /*!< Get Free Fall Threshold (65.5mg / LSB) */
	/*! Set Free Fall Duratiom (5ms / LSB) */
	void setFreeFallDuration(float duration) { io.writeByte(kRegTimeFreefall, mw::pin<uint8_t>(duration / 0.005f, 0, 255)); }
	float getFreeFallDuration() { return io.readByte(kRegTimeFreefall) * 0.005f; }

	void setActivityX(bool state) { io.writeBit(kRegActInactCtl, 6, state); }
	bool getActivityX(void) { return io.readBit(kRegActInactCtl, 6); }
	void setActivityY(bool state) { io.writeBit(kRegActInactCtl, 5, state); }
	bool getActivityY(void) { return io.readBit(kRegActInactCtl, 5); }
	void setActivityZ(bool state) { io.writeBit(kRegActInactCtl, 4, state); }
	bool getActivityZ(void) { return io.readBit(kRegActInactCtl, 4); }
	void setActivityXYZ(bool state) {
		uint8_t value = io.readByte(kRegActInactCtl);
		if (state) {
			value |= 0b00111000;
		} else {
			value &= 0b11000111;
		}
		io.writeByte(kRegActInactCtl, value);
	}

	void setInactivityX(bool state) { io.writeBit(kRegActInactCtl, 2, state); }
	bool getInactivityX(void) { return io.readBit(kRegActInactCtl, 2); }
	void setInactivityY(bool state) { io.writeBit(kRegActInactCtl, 1, state); }
	bool getInactivityY(void) { return io.readBit(kRegActInactCtl, 1); }
	void setInactivityZ(bool state) { io.writeBit(kRegActInactCtl, 0, state); }
	bool getInactivityZ(void) { return io.readBit(kRegActInactCtl, 0); }
	void setInactivityXYZ(bool state) {
		uint8_t value = io.readByte(kRegActInactCtl);
		if (state) {
			value |= 0b00000111;
		} else {
			value &= 0b11111000;
		}
		io.writeByte(kRegActInactCtl, value);
	}

	void setTapDetectionX(bool state) { io.writeBit(kRegTapAxes, 2, state); }
	bool getTapDetectionX(void) { return io.readBit(kRegTapAxes, 2); }
	void setTapDetectionY(bool state) { io.writeBit(kRegTapAxes, 1, state); }
	bool getTapDetectionY(void) { return io.readBit(kRegTapAxes, 1); }
	void setTapDetectionZ(bool state) { io.writeBit(kRegTapAxes, 0, state); }
	bool getTapDetectionZ(void) { return io.readBit(kRegTapAxes, 0); }
	void setTapDetectionXYZ(bool state) {
		uint8_t value = io.readByte(kRegTapAxes);
		if (state) {
			value |= 0b00000111;
		} else {
			value &= 0b11111000;
		}
		io.writeByte(kRegTapAxes, value);
	}

	void useInterrupt(int_t interrupt) {
		io.writeByte(kRegIntMap, interrupt == kInt1? 0x00: 0xFF);
		io.writeByte(kRegIntEnable, 0xFF);
	}

	bool isValid;
protected:
	T io;
};


}
