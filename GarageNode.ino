/*
- Garage node

read each sensor, prepend with an identifyer and send
controller contains the business logic

- The circuit

IR Sensor (no longer used)
A0	No resistor needed. Analog output that varies from 3.1V at 4cm to 0.3V at 30cm with a supply voltage between 4.5 and 5.5VDC
http://bildr.org/2011/03/various-proximity-sensors-arduino/
https://www.sparkfun.com/products/12728

Sonar Sensors for car ports analog breadout board connects directly to analog pin
A0 = Sonar Sensor - car port A
A1 = Sonar Sensor - car port B

Light Sensors (A1 and A2)
A2 to Sensor to Gnd
A2 to 10K to 5V

Temp Sensor
DHT22 (also RHT03)
Data(pin2) D4 via 10K pullup resistor from data pin to power of the sensor
5V and Gnd

I2C Temp Sensor - Sparkfun Breakout board is 3.3V
A5	SCL
A4	SDA
3.3V Power
Ground

TODO: re-evaluate these LED needs
D2 Disovered LED
D4 Send LED
Dn -> 10K -> Led -> Gnd

D3 = garage door
D3 -> 10K -> Gnd, switch between D3 and Gnd
Gnd -> .01uF -> D3

Switches with hardware debounce
D5 = car garage door lower pin
D6 = car garage door upper pin
Dn -> 10K -> Gnd, switch leads between Dn and Gnd
Gnd -> .01uF -> Dn
*/

#include "DHT.h"
#include "wire.h"

// LED
const uint8_t discoveredLED = 7;	// connected and sending between controller and node
const uint8_t activityLED = 4;

// sonar sensor - car port A
const uint8_t sonarPinA = A0;
int sonarReadingA = 0;
char *sonarIdA = "sa";

// sonar sensor - car port B
const uint8_t sonarPinB = A1;
int sonarReadingB = 0;
char *sonarIdB = "sb";

// light sensors
const uint8_t lightPin = A2;
int lightReading = 0;
char *lightIdentifier = "la";	// (L)ights, not garage door opener light

const uint8_t garageDoorLightPin = A1;
int garageDoorLightReading = 0;
char *garageDoorLightIdentifier = "lb";

// temp sensor DHT22 (outside)
#define DHTPIN 2
DHT dht(DHTPIN, DHT22);
char *tmpIdC = "tc";	// temp in C
char *tmpIdF = "tf";	// temp in F
char *tmpIdH = "th";	// humidity
char *tmpIdHiC = "ta";	// heat index in C
char *tmpIdHiF = "tb";	// heat index in F

// I2C temp sensor (inside) - Note: breakout board is 3.3v
const int tmpAddress = 0x48;
char *tmpIntIdC = "ti";	// internal temp in C
float tmpIntReadingC = 0;

float lastTmpIntReadingC = 0;

// kitchen door switch from house
const uint8_t kitchenDoorPin = 3;
uint8_t kitchenDoorReading = 0;
char *kitchenDoorId = "kd";

// car (G)arage door reed switchs
// two sensors GarageLower and GarageUpper
const uint8_t lowerSwitchPin = 5;
const uint8_t upperSwitchPin = 6;
char *lGarageId = "gl";
char *uGarageId = "gu";

uint8_t lowerSwitchReading = 0;
uint8_t upperSwitchReading = 0;

uint8_t lastLowerSwitchReading = -1;
uint8_t lastUpperSwitchReading = -1;
uint8_t lastKitchenDoorReading = -1;
int lastLightReading = 0;
int lastGarageDoorLightReading = 0;
int lastSonarReadingA = 0;
int lastSonarReadingB = 0;
struct LastTempReadings
{
	float lastTempF;
	float lastTempC;
	float lastHumidity;
	float lastHeatIndexF;
	float lastHeatIndexC;
};

LastTempReadings *lastTempData = new LastTempReadings();

// struct for DHT22
struct TempReadings
{
	float tempF;
	float tempC;
	float humidity;
	float heatIndexF;
	float heatindexC;
};

TempReadings *tempData = new TempReadings();

// poll garaage bay timer
long lastBayMills = 0;
int bayTimer = 3000;

// poll temp data
long lastTempMills = 0;
int tempTimer = 5000;

// poll light sensor every second
long lastLightMills = 0;
int lightTimer = 1000;

// poll all door timer (unsure if this needs a polling threshold)
long lastDoorMills = 0;
int doorTimer = 100;

// serial data
#define SERIAL_BUFFER 2
uint8_t inboundSerialRead[SERIAL_BUFFER];

// is the USB Serial cable connected
bool isConnected = false;

// used to discover node, host will have to search the ports for the node
//bool isDiscovered = false;
// enabled by default for debugging
bool isDiscovered = true;

// wait for host signal to start/pause
//bool startSession = false;
// enabled by default for debugging
bool startSession = true;

void setup()
{
	// LED
	pinMode(discoveredLED, OUTPUT);
	pinMode(activityLED, OUTPUT);

	// house garage door
	pinMode(kitchenDoorPin, INPUT);

	// car garage door
	pinMode(lowerSwitchPin, INPUT);
	pinMode(upperSwitchPin, INPUT);

	digitalWrite(discoveredLED, LOW);
	digitalWrite(activityLED, LOW);

	Serial.begin(9600);
	Wire.begin();

	// TODO: need this? Flash until the USB serial cable is connected
	while (!Serial)
	{
		flashLed(isDiscovered);
	}

	// now connected to Remote
	isConnected = true;
}

void loop()
{
	// host not discovered node (via port) yet
	// TODO: is this really necessary with a UI controller?
	if (!isDiscovered)
	{
		nodeDiscovery();
	}

	// wait for host to signal to start
	if (!startSession)
	{
		while (!startSession)
		{
			readData();
		}
	}

	// up and running, enable discoveredLED
	digitalWrite(discoveredLED, HIGH);

	// reciver to filter noise logic
	while (startSession)
	{
		// poll doors
		if (millis() - lastDoorMills > doorTimer)
		{
			upperSwitchReading = digitalRead(upperSwitchPin);
			lowerSwitchReading = digitalRead(lowerSwitchPin);
			
			if (didValueChange(lastUpperSwitchReading, upperSwitchReading) || didValueChange(lastLowerSwitchReading, lowerSwitchReading))
			{
				sendData(uGarageId, upperSwitchReading);
				sendData(lGarageId, lowerSwitchReading);

				lastUpperSwitchReading = upperSwitchReading;
				lastLowerSwitchReading = lowerSwitchReading;
				flashLed(activityLED);
			}

			/*
			// read garage car door positions
			upperSwitchReading = digitalRead(upperSwitchPin);
			sendData(uGarageId, upperSwitchReading);

			if (didValueChange(lastUpperSwitchReading, upperSwitchReading))
			{
				lastUpperSwitchReading = upperSwitchReading;
				sendData(uGarageId, upperSwitchReading);
			}


			lowerSwitchReading = digitalRead(lowerSwitchPin);
			sendData(lGarageId, lowerSwitchReading);

			if (didValueChange(lastLowerSwitchReading, lowerSwitchReading))
			{
				lastLowerSwitchReading = lowerSwitchReading;
				sendData(lGarageId, lowerSwitchReading);
			}
			*/

			// read kitchen (to/from garage) door
			kitchenDoorReading = digitalRead(kitchenDoorPin);
			if (didValueChange(lastKitchenDoorReading, kitchenDoorReading))
			{
				lastKitchenDoorReading = kitchenDoorReading;
				sendData(kitchenDoorId, kitchenDoorReading);
				flashLed(activityLED);
			}

			lastDoorMills = millis();
		}

		// poll for temp
		if (millis() - lastTempMills > tempTimer)
		{
			// temp readings
			takeTempReading();

			// outside sensor
			if (didFloatValueChange(lastTempData->lastTempC, tempData->tempC))
			{
				lastTempData->lastTempC = tempData->tempC;
				sendTempData(tmpIdC, tempData->tempC);
				flashLed(activityLED);
			}

			if (didFloatValueChange(lastTempData->lastHumidity, tempData->humidity))
			{
				lastTempData->lastHumidity = tempData->humidity;
				sendTempData(tmpIdH, tempData->humidity);
				flashLed(activityLED);
			}

			if (didFloatValueChange(lastTempData->lastHeatIndexC, tempData->heatindexC))
			{
				lastTempData->lastHeatIndexC = tempData->heatindexC;
				sendTempData(tmpIdHiC, tempData->heatindexC);
				flashLed(activityLED);
			}

			// inside sensor
			if (didFloatValueChange(lastTmpIntReadingC, tmpIntReadingC))
			{
				lastTmpIntReadingC = tmpIntReadingC;
				sendTempData(tmpIntIdC, tmpIntReadingC);
				flashLed(activityLED);
			}

			lastTempMills = millis();
		}

		// poll for lights
		if (millis() - lastLightMills > lightTimer)
		{
			// main garage lights
			lightReading = analogRead(lightPin);
			if (didValueChange(lastLightReading, lightReading))
			{
				lastLightReading = lightReading;
				sendData(lightIdentifier, lightReading);
				flashLed(activityLED);
			}

			// garage door opener lights
			garageDoorLightReading = analogRead(garageDoorLightPin);
			if (didValueChange(lastGarageDoorLightReading, garageDoorLightReading))
			{
				lastGarageDoorLightReading = garageDoorLightReading;
				sendData(garageDoorLightIdentifier, garageDoorLightReading);
				flashLed(activityLED);
			}

			lastLightMills = millis();
		}

		// poll garage bays
		if (millis() - lastBayMills > bayTimer)
		{
			// read sonar data, ignore first reading to allow ADC level to settle
			analogRead(sonarPinA);
			delay(50);
			for (uint8_t i = 0; i < 4; i++)
			{
				sonarReadingA += analogRead(sonarPinA);
				delay(50);
			}
			sonarReadingA /= 4;

			if (didValueChange(lastSonarReadingA, sonarReadingA))
			{
				sendData(sonarIdA, sonarReadingA);
				sonarReadingA = 0;
				flashLed(activityLED);
			}

			/*
			// read sonar data, ignore first reading to allow ADC level to settle
			analogRead(sonarPinB);
			delay(50);
			for (uint8_t i = 0; i < 4; i++)
			{
				sonarReadingB += analogRead(sonarPinB);
				delay(50);
			}
			sonarReadingB /= 4;

			if (didValueChange(lastSonarReadingB, sonarReadingB))
			{
				sendData(sonarIdB, sonarReadingB);
				sonarReadingB = 0;
				flashLed(activityLED);
			}
			*/

			lastBayMills = millis();
		}

		// check for new requests from controller
		readData();
	}
}

void sendData(char *identifier, int value)
{
	Serial.print(identifier);
	Serial.println(value);
}

bool didValueChange(int lastValue, int currentValue)
{
	if (lastValue != currentValue)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool didFloatValueChange(float lastValue, float currentValue)
{
	if (lastValue != currentValue)
	{
		return true;
	}
	else
	{
		return false;
	}
}

// data type is a float used to send tempature data
void sendTempData(char *identifier, float value)
{
	Serial.print(identifier);
	Serial.println(value);
}

void readData()
{
	while (Serial.available())
	{
		// read the expected bytes
		for (uint8_t i = 0; i < SERIAL_BUFFER; i++)
		{
			inboundSerialRead[i] = Serial.read();
		}
	}

	// control logic
	switch (inboundSerialRead[0])
	{
	case 's':
		Serial.println("Starting.");
		startSession = true;
		digitalWrite(activityLED, HIGH);
		break;

	case 'p':
		Serial.println("Paused.");
		startSession = false;
		digitalWrite(activityLED, LOW);
		break;

	default:
		break;
	}

	inboundSerialRead[0] = 0;
}

void takeTempReading()
{
	// external temp reading (DHT22)
	tempData->humidity = dht.readHumidity();
	tempData->tempC = dht.readTemperature();
	tempData->tempF = dht.readTemperature(true);

	// calculate the heat index
	tempData->heatindexC = dht.computeHeatIndex(tempData->tempC, tempData->humidity, false);
	tempData->heatIndexF = dht.computeHeatIndex(tempData->tempF, tempData->humidity);

	// internal temp reading (sparkfun breakout board)
	Wire.requestFrom(tmpAddress, 2);

	byte MSB = Wire.read();
	byte LSB = Wire.read();

	int tempSum = (MSB << 8 | LSB) >> 4;
	tmpIntReadingC = tempSum*0.0625;
}

void nodeDiscovery()
{
	while (!isDiscovered)
	{
		// wait for a signal from the host for node discovery
		while (Serial.available())
		{
			// read the expected bytes
			for (uint8_t i = 0; i < SERIAL_BUFFER; i++)
			{
				inboundSerialRead[i] = Serial.read();
			}

			if (inboundSerialRead[0] == 'z')
			{
				// notify host the node is available
				//Serial.println("ack");
				isDiscovered = true;
			}
		}
	}

	// finally connected
	inboundSerialRead[0] = 0;
	digitalWrite(discoveredLED, HIGH);
}

void flashLed(uint8_t led)
{
	digitalWrite(led, HIGH);
	delay(50);
	digitalWrite(led, LOW);
}
