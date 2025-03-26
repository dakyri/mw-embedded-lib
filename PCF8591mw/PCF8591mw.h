#pragma once
/*!
 * PCF8591mw.h - Header file for stripped back access to PCF8591 analog io expander.
 * Interface to any reasonable embedded mcp that we can lay hands on.
 */

#include <stdint.h> // arduino doesn't recognise <cstdint>

#include "../common/common.h"

template <typename T> class PCF8591
{
public:

	static constexpr uint8_t kChannelMask = 0b00000011;
	static constexpr uint8_t kAutoIncRead = 0b00000100;
	static constexpr uint8_t kADCModeSingle = 0b00000000;
	static constexpr uint8_t kADCMode3Diff = 0b00010000;
	static constexpr uint8_t kADCMode2Plus1 = 0b00100000;
	static constexpr uint8_t kADCMode2Diff = 0b00110000;
	static constexpr uint8_t kDACEnable = 0b01000000;
	static constexpr uint8_t kDACDisable = 0b00000000;

	template <typename ... IOArgs> PCF8591(IOArgs ... args): dacEnable(false), io(args ...) {}

	struct ins_t {
		uint8_t v[4];
	};

	/*!
	 * Read all the inputs at once
	 * \param adcMode the mode from one of the flags above controlling whether the read is differential or single
	 * \return a vector of input values
	 */
	struct ins_t adReadAll(uint8_t adcMode=kADCModeSingle){
		uint8_t buf[5];
		ins_t all_ins;
		uint8_t operation = kAutoIncRead | adcMode | (dacEnable? kDACEnable: kDACDisable);
		io.readBuf(operation, 5, buf);
		for (uint8_t i=0; i<4; ++i) all_ins.v[i] = buf[i+1];
		return all_ins;
	};

	/*!
	 * Reads the given adc input channel
	 * \param channel The channel to read
	 * \param adcMode the mode from one of the flags above controlling whether the read is differential or single
	 * \return the value on the given channel
	 */
	uint8_t adRead(uint8_t channel, uint8_t adcMode=kADCModeSingle){
		uint8_t buf[2];
		uint8_t operation = (channel&kChannelMask) | adcMode | (dacEnable? kDACEnable: kDACDisable);
		io.readBuf(operation, 2, buf);
		return buf[1];
	};

	/*!
	 * Turns the adc on and sends the given value
	 * \param value The new state of the device output. value of 0 disables the dac
	 */
	void daWrite(uint8_t value) {
		dacEnable = (value != 0);
		io.writeByte((dacEnable? kDACEnable: kDACDisable), value);
	};

	bool begin() {
		isValid = io.begin();
		return isValid;
	}

protected:
	bool dacEnable;
	bool isValid;
	T io;
};
/*
long PCF8591::readVcc(void) {
	#ifdef __AVR
		// Read 1.1V reference against AVcc
		// set the reference to Vcc and the measurement to the internal 1.1V reference
	#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
		ADMUX = _BV(MUX5) | _BV(MUX0);
	#else
		ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#endif

		delay(2); // Wait for Vref to settle
		ADCSRA |= _BV(ADSC); // Start conversion
		while (bit_is_set(ADCSRA, ADSC))
			; // measuring

		uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
		uint8_t high = ADCH; // unlocks both

		long result = (high << 8) | low;

		  result = 1083630L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
		  // scale_constant = internal1.1Ref * 1023 * 1000
		  // internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
		return result; // Vcc in millivolts
	#else
//		float vdd = readVcc(); //  ESP.getVdd33(); //ESP.getVcc();
		return 3300;
	#endif

 */
