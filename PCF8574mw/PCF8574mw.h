#pragma once
/*!
 * PCF8574mw.h - Header file for stripped back access to PCF8574 dig io expander.
 * Interface to any reasonable embedded mcp that we can lay hands on.
 */

#include <stdint.h> // arduino doesn't recognise <cstdint>

#include "../common/common.h"

/** Comment this define to disable interrupt support */

template <typename T> class PCF8574
{
public:
	template <typename PT> unsigned constexpr pin(PT x) { return 1 << x; }
	template <typename PT, typename ... Args> unsigned constexpr pin(PT x, Args ... args) { return 1 << x | bmap(args...); }

	template <typename ... IOArgs> PCF8574(IOArgs ... args): inputs(0xff), out(0), io(args ...) {}

	/*!
	 * Set the state of a pin (HIGH or LOW)
	 * \param pin The pin number to set
	 * \param value The new state of the pin
	 */
	void writePin(uint8_t pini, uint8_t value) {
		const uint8_t pinb = pin(pini) & ~inputs;
		if (!pinb) return;
		if (value) {
			out |= pinb;
		} else {
			out &= ~pinb;
		}
		io.writeByte(inputs | out);
	}

	/*!
	 * Set the state of all outputs according to the bitmap. Care is taken to preserve designated inputs
	 * \param value The new state of the device outputs
	 */
	void writeOutputs(uint8_t value) {
		out = value & ~inputs;
		io.writeByte(inputs | out);
	}

	/*!
	 * Sets the indicated pins in the given bitmap as inputs. They will be set high. Input on the 8574 is given by
	 * sending the weakly pulled up high outputs to ground.
	 * \param pins a bitmap of input pins. Any non-designated pin will be an output.
	 */
	void setInputs(uint8_t pins) {
		inputs = pins;
		out &= ~inputs;
		io.writeByte(inputs | out); // sets all the inputs high
	}

	/*!
	 * \return a bitmap of designated inputs which are activated (pulled low)
	 */
	uint8_t getInputs() {
		return ~io.readByte() & inputs;
	}

	bool begin() {
		isValid = io.begin();
		if (isValid) {
			setInputs(inputs);
		}
		return isValid;
	}

protected:
	uint8_t inputs;
	uint8_t out;
	bool isValid = false;
	T io;
};
