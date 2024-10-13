#include <Arduino.h>
#include <AS5048A.h>

using namespace std;

static const uint16_t AS5048A_ANGLE = 0x3FFF;

AS5048A angleSensor(16, false);

void setup()
{
	Serial.begin(115200);
	angleSensor.begin();
}

void loop()
{
	delay(100);

	//tuple<int, int, int> measurement = angleSensor.getThreeAbsoluteRotation();
	tuple<double, double, double> measurement = angleSensor.getThreeAbsoluteRotation();
  
	Serial.print(get<0>(measurement));
	Serial.print("  ");
	Serial.print(get<1>(measurement));
	Serial.print("  ");
	Serial.println(get<2>(measurement));
	//Serial.print("State: ");
	//angleSensor.printState();
	//Serial.print("Errors: ");
	//Serial.println(angleSensor.getErrors());
}