#pragma once
/*!
 * SC16IS750mw.h - Header file for stripped back access to SC16IS750 uart.
 * Interface to any reasonable embedded mcp that we can lay hands on.
 *
 * To work as directly with 47fx midi lib we need to implement:
 *  - void begin(uint32_t baud);
 *  - int read();
 *  - size_t write(uint8_t val);
 *  - int available();
 *  Thase should also be enough to inherit from the AVR Stream if we are in that world
 *
 */

#include <stdint.h> // arduino doesn't recognise <cstdint>

#include "../mw-common/defs.h"

template <typename T> class SC16IS750
#ifdef Arduino_h
	: public Stream
#endif
{
public:
	//A = Vdd, B = Gnd, C = SCL, D = SDA
	static const uint8_t addrAA = 0x90;
	static const uint8_t addrAB = 0x92;
	static const uint8_t addrAC = 0x94;
	static const uint8_t addrAD = 0x96;
	static const uint8_t addrBA = 0x98;
	static const uint8_t addrBB = 0x9A;
	static const uint8_t addrBC = 0x9C;
	static const uint8_t addrBD = 0x9E;
	static const uint8_t addrCA = 0xA0;
	static const uint8_t addrCB = 0xA2;
	static const uint8_t addrCC = 0xA4;
	static const uint8_t addrCD = 0xA6;
	static const uint8_t addrDA = 0xA8;
	static const uint8_t addrDB = 0xAA;
	static const uint8_t addrDC = 0xAC;
	static const uint8_t addrDD = 0xAE;

	//General Registers
	static const uint8_t regRHR		= 0x00;
	static const uint8_t regTHR		= 0x00;
	static const uint8_t regIER		= 0x01;
	static const uint8_t regFCR		= 0x02;
	static const uint8_t regIIR		= 0x02;
	static const uint8_t regLCR		= 0x03;
	static const uint8_t regMCR		= 0x04;
	static const uint8_t regLSR		= 0x05;
	static const uint8_t regMSR		= 0x06;
	static const uint8_t regSPR		= 0x07;
	static const uint8_t regTCR		= 0x06;
	static const uint8_t regTLR		= 0x07;
	static const uint8_t regTXLVL	= 0x08;
	static const uint8_t regRXLVL	= 0x09;
	static const uint8_t regIODIR	= 0x0A;
	static const uint8_t regIOSTATE	= 0x0B;
	static const uint8_t regIOINTENA = 0x0C;
	static const uint8_t regIOCONTROL = 0x0E;
	static const uint8_t regEFCR	= 0x0F;

	//Special Registers
	static const uint8_t regDLL		= 0x00;
	static const uint8_t regDLH		= 0x01;

	//Enhanced Registers
	static const uint8_t regEFR		= 0x02;
	static const uint8_t regXON1	= 0x04;
	static const uint8_t regXON2	= 0x05;
	static const uint8_t regXOFF1	= 0x06;
	static const uint8_t regXOFF2	= 0x07;

	// enums for interrupts. the values correspond to the enable bitmap, except for GPIO, RXERR, RXTIME which aren't enabled.
	// these values are arbitrary
	static const uint8_t intCTS		= 0x80;
	static const uint8_t intRTS		= 0x40;
	static const uint8_t intXOFF	= 0x20;
	static const uint8_t intSLEEP	= 0x10;
	static const uint8_t intMODEM	= 0x08;
	static const uint8_t intLINE	= 0x04;
	static const uint8_t intTHR		= 0x02;
	static const uint8_t intRHR		= 0x01;
	static const uint8_t intGPIO	= 0x81;
	static const uint8_t intRXERR	= 0x82;
	static const uint8_t intRXTIME	= 0x83;
	static const uint8_t intCRTS	= 0xc0; // CTS or RTS


	template <typename ... IOArgs> SC16IS750(IOArgs ... args): io(args ...), hasPeeked(false), peekBuf(0) {}

	void begin(uint32_t crystalHz, uint32_t baudHz) {
		io.begin();
	    resetDevice();
	    fifoEnable(true);
		setBaudrate(crystalHz, baudHz);
	    setLine(8,0,1);
	}

	/*!
	 * for interop with arduino Stream class, and 47fx Midi
	 */
	virtual int available() {
	    return fifoAvailableData();
	}

	/*!
	 * for interop with arduino Stream class, and 47fx Midi
	 */
	virtual int read() {
		if (hasPeeked) {
			hasPeeked = false;
			return peekBuf;
		}
		if (fifoAvailableData() == 0) {
			return -1;
		}
		return readByte();
	}

	/*!
	 * for interop with arduino Stream class
	 */
	virtual int peek() {
		if (!hasPeeked) {
			if (fifoAvailableData() == 0) {
				return -1;
			}
			peekBuf = readByte();
			hasPeeked = true;
		}
		return peekBuf;
	}

	/*!
	 * for interop with arduino Print class, and 47fx Midi
	 */
	virtual size_t write(uint8_t val) {
	    writeByte(val);
	}

	/*!
	 * for interop with arduino Print class
	 * repeatedly poll line status register until bit 5 goes high (Transmit holding register is empty).
	 */
	virtual void flush() {
		while (!io.readBit(regLSR, 5));
	}

	 /*!
	  * \param crystalFreq crystal frequency in Hz
	  * \param baudrate target baudrate
	  * \return error of baudrate parts per thousand
	  */
	int16_t setBaudrate(uint32_t crystalFreq, uint32_t baudrate) {
	    const uint8_t prescaler = io.readBit(regMCR, 7)? 4 : 1;
	    const uint16_t divisor = (crystalFreq/prescaler)/(baudrate*16);
	    const uint8_t origLCR = io.writeBit(regLCR, 7, true);
	    io.writeByte(regDLL, uint8_t(divisor & 0xff));
	    io.writeByte(regDLH, uint8_t(divisor >> 8));
	    io.writeByte(regLCR,origLCR);
	    const uint32_t actual_baudrate = (crystalFreq/prescaler)/(16*divisor);
	    return int16_t(float(actual_baudrate-baudrate) * 1000 / baudrate);;
	}

	void setLine(uint8_t dataLength, uint8_t paritySelect, uint8_t stopLength )
	{
	    uint8_t tmpLCR = io.readByte(regLCR);
	    tmpLCR &= 0xC0; //Clear LCR[0..5]
	    switch (dataLength) {
	        case 5:
	            break;
	        case 6:
	            tmpLCR |= 0x01;
	            break;
	        case 7:
	            tmpLCR |= 0x02;
	            break;
	        case 8:
	            tmpLCR |= 0x03;
	            break;
	        default:
	            tmpLCR |= 0x03;
	            break;
	    }
	    if (stopLength == 2) {
	        tmpLCR |= 0x04;
	    }
	    switch (paritySelect) {
	        case 1: //odd parity
	            tmpLCR |= 0x08;
	            break;
	        case 2: //even parity
	            tmpLCR |= 0x18;
	            break;
	        case 3: //force '1' parity
	            tmpLCR |= 0x03;
	            break;
	        case 0: //no parity
	        case 4: //force '0' parity
	        default:
	            break;
	    }
	    io.writeByte(regLCR, tmpLCR);
	}

	void resetDevice() {
		io.writeBit(regIOCONTROL, 3, true);
	}

	/*!
	 * \param gpio bool if true, gpio[7:4] are modem pins, else gpio[7:4] are gpios
	 */
	void enableModemPins(bool doModem) {
	    io.writeBit(regIOCONTROL, 1, doModem)
	}

	/*!
	 * enable the given interrupts according to the bitmap
	 * CTS = bit 7, RTS = 6, Xoff = 5,  Sleep mode = 4,  modem status = 3, receive line status = 2, THR empty = 1, RX data avail = 0
	 */
	void interruptEnable(uint8_t interrupts) {
	    io.writeByte(regIER, interrupts);
	}

	bool SC16IS750::interruptIsPending() {
	    return (io.readByte(regIIR) & 0x01) == 0;
	}

	/*!
	 * \return the enum for the current pending interrupt, or 0 if none
	 */
	void SC16IS750::interruptPending(void)
	{
	    uint8_t interrupt = io.readByte(regIIR);
	    if ((interrupt & 0x01) != 0) return 0;
	    interrupt = (interrupt >> 1) & 0x1f;
	    switch (interrupt) {
	        case 0x03: return intRXERR; //Receiver Line Status Error
	        case 0x06: return intRXTIME; //Receiver time-out interrupt
	        case 0x02: return intRHR; //RHR interrupt
	        case 0x01: return intTHR; //THR interrupt
	        case 0x00: return intMODEM; //modem interrupt;
	        case 0x18: return intGPIO; //input pin change of state
	        case 0x08: return intXOFF; //XOFF
	        case 0x10: return intCRTS; //CTS,RTS
	    }
	    return 0;
	}

	void fifoSetTriggerLevel(bool setRxFifo, uint8_t length) {
		io.writeBit(regMCR, 2, true); //SET MCR[2] to '1' to use TLR register or trigger level control in FCR register
		uint8_t origEFR = io.writeBit(regEFR, 4, true);  //set ERF[4] to '1' to use the  enhanced features
		io.writeByte(regTLR, setRxFifo? (length<<4) : (length&0x0f));
	    io.writeByte(regEFR, origEFR);
	    return;
	}

	uint8_t fifoAvailableData() {
	   return io.readByte(regRXLVL);
	}

	uint8_t fifoAvailableSpace() {
	   return io.readByte(regTXLVL);
	}

	void fifoEnable(bool enable) {
		io.writeBit(regFCR, 0, enable);
	}

	void fifoReset(bool rxReset) {
		io.writeBit(regFCR, rxReset? 1: 2, true);
	}

	void writeByte(uint8_t val) {
//		while (fifoAvailableSpace() == 0);
		while (!io.readBit(regLSR, 5));
		io.writeByter(regTHR, val);
	}

	int readByte(void) {
		return io.readBye(regRHR);
	}

	void enableTx(bool tx_enable){
		io.writeBit(regEFCR, 2, !tx_enable);
	}

	void writeScratch(uint8_t val) {
		io.writeByte(regSPR, val);
	}

	uint8_t readScratch() {
		return io.readByte(regSPR);
	}

	void gpioSetPinMode(uint8_t pinNumber, bool isOutput)	{
		io.writeBit(regIODIR, pinNumber, isOutput);
	}

	void gpioSetPinState(uint8_t pinNumber, bool state) {
		io.writeBit(regIOSTATE, pinNumber, state);
	}

	uint8_t gpioGetPinState(uint8_t pinNumber) {
		return io.readBit(regIOSTATE, pinNumber);
	}

	uint8_t gpioGetPortState() {
	    return io.readByte(regIOSTATE);
	}

	void gpioSetPortMode(uint8_t ioMode) {
	    io.writeByte(regIODIR, ioMode);
	}

	void gpioSetPortState(uint8_t ioState) {
	    io.writeByte(regIOSTATE, ioState);
	}

	void gpioLatch(bool latch) {
		io.writeBit(regIOCONTROL, 0, latch);
	}

	void gpioPinInterrupt(uint8_t pinNumber, bool en) {
		io.writeBit(regIOINTENA, pinNumber, en);
	}

protected:
	T io;
	bool hasPeeked;
	uint8_t peekBuf;
}
