#include "Arduino.h"
#include <tuple>
#include <AS5048A.h>

static const uint16_t AS5048A_CLEAR_ERROR_FLAG = 0x0001;
static const uint16_t AS5048A_PROGRAMMING_CONTROL = 0x0003;
static const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH = 0x0016;
static const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW = 0x0017;
static const uint16_t AS5048A_DIAG_AGC = 0x3FFD;
static const uint16_t AS5048A_MAGNITUDE = 0x3FFE;
static const uint16_t AS5048A_ANGLE = 0x3FFF;

static const uint8_t AS5048A_AGC_FLAG = 0xFF;
static const uint8_t AS5048A_ERROR_PARITY_FLAG = 0x04;
static const uint8_t AS5048A_ERROR_COMMAND_INVALID_FLAG = 0x02;
static const uint8_t AS5048A_ERROR_FRAMING_FLAG = 0x01;

static const uint16_t AS5048A_DIAG_COMP_HIGH = 0x2000;
static const uint16_t AS5048A_DIAG_COMP_LOW = 0x1000;
static const uint16_t AS5048A_DIAG_COF = 0x0800;
static const uint16_t AS5048A_DIAG_OCF = 0x0400;

static const double AS5048A_MAX_VALUE = 8191.0;
static const double AS5048A_MAX_VALUE_FULL = 16383.0;


/**
 * Constructor
 */
AS5048A::AS5048A(byte cs, bool debug /*=false*/)
	: _cs(cs), errorFlag(false), ocfFlag(false), position(0), debug(debug), absolutePosition1(0), 
	lastPosition1(0), absolutePosition2(0), lastPosition2(0), absolutePosition3(0), lastPosition3(0),
	offset1(0), offset2(0), offset3(0)
{
}

void AS5048A::zero_three_sensors()
{
	std::tuple<int, int, int> data = AS5048A::getThreeAbsoluteRotation();

	this->offset1 = std::get<0>(data);
	this->offset2 = std::get<1>(data);
	this->offset3 = std::get<2>(data);
}

/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A::begin()
{
	setDelay();

	// 1MHz clock (AMS should be able to accept up to 10MHz)
	this->settings = SPISettings(4000000, MSBFIRST, SPI_MODE1);

	//setup pins
	pinMode(this->_cs, OUTPUT);

	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	SPI.begin();
	delay(1);
	zero_three_sensors();
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each begin()-call the close() function must be called exactly 1 time
 */
void AS5048A::close()
{
	SPI.end();
}

/**
 * Utility function used to calculate even parity of an unigned 16 bit integer
 */
uint8_t AS5048A::spiCalcEvenParity(uint16_t value)
{
	uint8_t cnt = 0;

	for (uint8_t i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

std::tuple<int, int, int> AS5048A::getThreeAbsoluteRotation()
{
	int16_t rotation1 = 0;
	int16_t rotation2 = 0;
	int16_t rotation3 = 0;

	std::tuple<uint16_t, uint16_t, uint16_t> data = AS5048A::read_three(AS5048A_ANGLE);
	rotation1 = std::get<0>(data);
	rotation2 = std::get<1>(data);
	rotation3 = std::get<2>(data);

	int difference1 = rotation1 - lastPosition1;
	int difference2 = rotation2 - lastPosition2;
	int difference3 = rotation3 - lastPosition3;
	
	// wrap for rotation1
	if (difference1 > AS5048A_MAX_VALUE) {
		absolutePosition1 += (difference1 - AS5048A_MAX_VALUE_FULL);
	}
	else if (difference1 < -AS5048A_MAX_VALUE) {
		absolutePosition1 += (difference1 + AS5048A_MAX_VALUE_FULL);
	}
	else {
		absolutePosition1 += difference1;
	}

	// wrap for rotation2
	if (difference2 > AS5048A_MAX_VALUE) {
		absolutePosition2 += (difference2 - AS5048A_MAX_VALUE_FULL);
	}
	else if (difference2 < -AS5048A_MAX_VALUE) {
		absolutePosition2 += (difference2 + AS5048A_MAX_VALUE_FULL);
	}
	else {
		absolutePosition2 += difference2;
	}

	// wrap for rotation3
	if (difference3 > AS5048A_MAX_VALUE) {
		absolutePosition3 += (difference3 - AS5048A_MAX_VALUE_FULL);
	}
	else if (difference3 < -AS5048A_MAX_VALUE) {
		absolutePosition3 += (difference3 + AS5048A_MAX_VALUE_FULL);
	}
	else {
		absolutePosition3 += difference3;
	}

	lastPosition1 = rotation1;
	lastPosition2 = rotation2;
	lastPosition3 = rotation3;

	return {absolutePosition1 - offset1, absolutePosition2 - offset2, absolutePosition3 - offset3};
}

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int16_t} between -2^13 and 2^13
 */
int16_t AS5048A::getRotation()
{
	uint16_t data;
	int16_t rotation;

	data = AS5048A::getRawRotation();
	rotation = static_cast<int16_t>(data) - static_cast<int16_t>(this->position);
	if (rotation > AS5048A_MAX_VALUE)
		rotation = -((0x3FFF) - rotation); //more than -180

	return rotation;
}

/**
 * Returns the raw angle directly from the sensor
 */
int16_t AS5048A::getRawRotation()
{
	return AS5048A::read(AS5048A_ANGLE);
}

/**
  * Get the rotation of the sensor relative to the zero position in degrees.
  *
  * @return {double} between 0 and 360
  */

double AS5048A::getRotationInDegrees()
{
	int16_t rotation = getRotation();
	double degrees = 360.0 * (rotation + AS5048A_MAX_VALUE) / (AS5048A_MAX_VALUE * 2.0);
	return degrees;
}

/**
  * Get the rotation of the sensor relative to the zero position in degrees.
  *
  * @return {double}
  */
std::tuple<double, double, double> AS5048A::getThreeAbsoluteRotationInDegrees()
{
	std::tuple<int, int, int> absoluteRotation = getThreeAbsoluteRotation();

	double degrees1 = 360.0 * (std::get<0>(absoluteRotation)) / (AS5048A_MAX_VALUE_FULL);
	double degrees2 = 360.0 * (std::get<1>(absoluteRotation)) / (AS5048A_MAX_VALUE_FULL);
	double degrees3 = 360.0 * (std::get<2>(absoluteRotation)) / (AS5048A_MAX_VALUE_FULL);
	
	return {degrees1, degrees2, degrees3};
}

/**
  * Get the rotation of the sensor relative to the zero position in radians.
  *
  * @return {double} between 0 and 2 * PI
  */

double AS5048A::getRotationInRadians()
{
	int16_t rotation = getRotation();
	double radians = PI * (rotation + AS5048A_MAX_VALUE) / AS5048A_MAX_VALUE;
	return radians;
}

/**
 * returns the value of the state register
 * @return unsigned 16 bit integer containing flags
 */
uint16_t AS5048A::getState()
{
	return AS5048A::read(AS5048A_DIAG_AGC);
}

/**
 * Print the diagnostic register of the sensor
 */
void AS5048A::printState()
{
	if (this->debug)
	{
		uint16_t data = AS5048A::getState();
		if (AS5048A::error())
		{
			Serial.print("Error bit was set!");
		}
		Serial.println(data, BIN);
	}
}

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5048A::getGain()
{
	uint16_t data = AS5048A::getState();
	return static_cast<uint8_t>(data & AS5048A_AGC_FLAG);
}

/**
 * Get diagnostic
 */
String AS5048A::getDiagnostic()
{
	uint16_t data = AS5048A::getState();
	if (data & AS5048A_DIAG_COMP_HIGH)
	{
		return "COMP high";
	}
	if (data & AS5048A_DIAG_COMP_LOW)
	{
		return "COMP low";
	}
	if (data & AS5048A_DIAG_COF)
	{
		return "CORDIC overflow";
	}
	if (data & AS5048A_DIAG_OCF && ocfFlag == false)
	{
		ocfFlag = true;
		return "Offset compensation finished";
	}
	return "";
}

/*
 * Get and clear the error register by reading it
 */
String AS5048A::getErrors()
{
	uint16_t error = AS5048A::read(AS5048A_CLEAR_ERROR_FLAG);
	if (error & AS5048A_ERROR_PARITY_FLAG)
	{
		return "Parity Error";
	}
	if (error & AS5048A_ERROR_COMMAND_INVALID_FLAG)
	{
		return "Command invalid";
	}
	if (error & AS5048A_ERROR_FRAMING_FLAG)
	{
		return "Framing error";
	}
	return "";
}

/*
 * Set the zero position
 */
void AS5048A::setZeroPosition(uint16_t position)
{
	this->position = position % 0x3FFF;
}

/*
 * Returns the current zero position
 */
uint16_t AS5048A::getZeroPosition()
{
	return this->position;
}

/*
 * Check if an error has been encountered.
 */
bool AS5048A::error()
{
	return this->errorFlag;
}

/*
 * Read 3 daisy chained sensors
 */
std::tuple<uint16_t, uint16_t, uint16_t> AS5048A::read_three(uint16_t registerAddress)
{
	uint16_t command = 0x4000; // PAR=0 R/W=R
	command = command | registerAddress;

	//Add a parity bit on the the MSB
	command |= static_cast<uint16_t>(spiCalcEvenParity(command) << 0xF);

	if (this->debug)
	{
		Serial.print("Read (0x");
		Serial.print(registerAddress, HEX);
		Serial.print(") with command: 0b");
		Serial.println(command, BIN);
	}

	//SPI - begin transaction
	SPI.beginTransaction(this->settings);

	//Send the command
	digitalWrite(this->_cs, LOW);
	SPI.transfer16(command);
	SPI.transfer16(command);
	SPI.transfer16(command);
	digitalWrite(this->_cs, HIGH);

	delay(this->esp32_delay);

	//Now read the response
	digitalWrite(this->_cs, LOW);
	uint16_t response1 = SPI.transfer16(command);
	uint16_t response2 = SPI.transfer16(command);
	uint16_t response3 = SPI.transfer16(command);
	digitalWrite(this->_cs, HIGH);

	//SPI - end transaction
	SPI.endTransaction();

	if (this->debug)
	{
		Serial.print("Read returned: ");
		Serial.println(response1, BIN);
		Serial.println(response2, BIN);
		Serial.println(response3, BIN);
	}

	//Check if the error bit is set
	if (response1 & 0x4000)
	{
		if (this->debug)
		{
			Serial.println("response1 error flag");
		}
		this->errorFlag = true;
		
	}
	else {
		this->errorFlag = false;
	}


	//Return the data, stripping the parity and error bits
	response1 &= ~0xC000;
	response2 &= ~0xC000;
	response3 &= ~0xC000;

	return {response1, response2, response3};
}

/*
 * Read a register from the sensor
 * Takes the address of the register as an unsigned 16 bit
 * Returns the value of the register
 */
uint16_t AS5048A::read(uint16_t registerAddress)
{
	uint16_t command = 0x4000; // PAR=0 R/W=R
	command = command | registerAddress;

	//Add a parity bit on the the MSB
	command |= static_cast<uint16_t>(spiCalcEvenParity(command) << 0xF);

	if (this->debug)
	{
		Serial.print("Read (0x");
		Serial.print(registerAddress, HEX);
		Serial.print(") with command: 0b");
		Serial.println(command, BIN);
	}

	//SPI - begin transaction
	SPI.beginTransaction(this->settings);

	//Send the command
	digitalWrite(this->_cs, LOW);
	SPI.transfer16(command);
	digitalWrite(this->_cs, HIGH);

	delay(this->esp32_delay);

	//Now read the response
	digitalWrite(this->_cs, LOW);
	uint16_t response = SPI.transfer16(0x00);
	digitalWrite(this->_cs, HIGH);

	//SPI - end transaction
	SPI.endTransaction();

	if (this->debug)
	{
		Serial.print("Read returned: ");
		Serial.println(response, BIN);
	}

	//Check if the error bit is set
	if (response & 0x4000)
	{
		if (this->debug)
		{
			Serial.println("Setting error bit");
		}
		this->errorFlag = true;
	}
	else
	{
		this->errorFlag = false;
	}

	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}

/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the unsigned 16 bit of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t AS5048A::write(uint16_t registerAddress, uint16_t data)
{

	uint16_t command = 0x0000; // PAR=0 R/W=W
	command |= registerAddress;

	//Add a parity bit on the the MSB
	command |= static_cast<uint16_t>(spiCalcEvenParity(command) << 0xF);

	if (this->debug)
	{
		Serial.print("Write (0x");
		Serial.print(registerAddress, HEX);
		Serial.print(") with command: 0b");
		Serial.println(command, BIN);
	}

	//SPI - begin transaction
	SPI.beginTransaction(this->settings);

	//Start the write command with the target address
	digitalWrite(this->_cs, LOW);
	SPI.transfer16(command);
	digitalWrite(this->_cs, HIGH);

	uint16_t dataToSend = 0x0000;
	dataToSend |= data;

	//Craft another packet including the data and parity
	dataToSend |= static_cast<uint16_t>(spiCalcEvenParity(dataToSend) << 0xF);

	if (this->debug)
	{
		Serial.print("Sending data to write: ");
		Serial.println(dataToSend, BIN);
	}

	//Now send the data packet
	digitalWrite(this->_cs, LOW);
	SPI.transfer16(dataToSend);
	digitalWrite(this->_cs, HIGH);

	delay(this->esp32_delay);

	digitalWrite(this->_cs, LOW);
	uint16_t response = SPI.transfer16(0x0000);
	digitalWrite(this->_cs, HIGH);

	//SPI - end transaction
	SPI.endTransaction();

	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}

/**
 * Set the delay acording to the microcontroller architecture
 */
void AS5048A::setDelay()
{
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
	this->esp32_delay = 10;
	if (this->debug)
	{
		Serial.println("AS5048A working with ESP32");
	}
#elif __AVR__
	this->esp32_delay = 0;
	if (this->debug)
	{
		Serial.println("AS5048A working with AVR");
	}
#else
	this->esp32_delay = 0;
	if (this->debug)
	{
		Serial.println("Device not detected");
	}
#endif
}
