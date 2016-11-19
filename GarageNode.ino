/*
- Garage node

read each sensor, prepend with an identifyer and send
controller contains the business logic

- The circuit

IR Sensor
A0	No resistor needed. Analog output that varies from 3.1V at 4cm to 0.3V at 30cm with a supply voltage between 4.5 and 5.5VDC
http://bildr.org/2011/03/various-proximity-sensors-arduino/
https://www.sparkfun.com/products/12728

Light Sensor
A2 to Sensor to Gnd
A2 to 10K to 5V

Temp Sensor
DHT22 (also RHT03)
Data(pin2) D4 via 10K pullup resistor from data pin to power of the sensor
5V and Gnd

A0 = Sonar Sensor, breadout board connects directly

TODO: re-evaluate these LED needs
D6 Disovered LED
D7 Send LED
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

// LED			TODO: add LED and correct pin
const uint8_t discoveredLED = 13;
const uint8_t activityLED = 4;

// light sensor
const uint8_t lightPin = A1;
int lightReading = 0;
int lightLastReading = 0;
char *lightIdentifier = "l";	// (L)ights, not garage door opener light

// sonar sensor
const uint8_t sonarPin = A0;
int sonarReading = 0;
int lastSonarReading = 0;
char *sonarId = "sa";		// accomodating multiple sonar sensors

// temp sensor
#define DHTPIN 2
DHT dht(DHTPIN, DHT22);
char *tmpIdC = "tc";	// temp in C
char *tmpIdF = "tf";	// temp in F
char *tmpIdH = "th";	// humidity
char *tmpIdHiC = "ta";	// heat index in C
char *tmpIdHiF = "tb";	// heat index in F

// garage door switch from house
const uint8_t garageDoorPin = 3;
uint8_t garageDoorReading = 0;
char *doorId = "de";

// car (G)arage door reed switchs
// two sensors GarageLower and GarageUpper
const uint8_t lowerSwitchPin = 5;
const uint8_t upperSwitchPin = 6;
char *lGarageId = "gl";
char *uGarageId = "gu";

uint8_t lowerSwitchReading = 0;
uint8_t upperSwitchReading = 0;

struct TempReadings
{
	float tempF;
	float tempC;
	float humidity;
	float heatIndexF;
	float heatindexC;
};

TempReadings *tempData = new TempReadings();

// timer
long lastMills = 0;
int timerThreshold = 2000;

// serial data
#define SERIAL_BUFFER 3
uint8_t inboundSerialRead[SERIAL_BUFFER];

// is the USB Serial cable connected
bool isConnected = false;

// used to discover node, host will have to search the ports for the node
bool isDiscovered = false;

// wait for host signal to start/pause
bool startSession = false;

void setup()
{
	pinMode(discoveredLED, OUTPUT);
	pinMode(activityLED, OUTPUT);

	// house garage door
	pinMode(garageDoorPin, INPUT);

	// car garage door
	pinMode(lowerSwitchPin, INPUT);
	pinMode(upperSwitchPin, INPUT);

	//pinMode(sonarPin, INPUT);
	//pinMode(lightPin, INPUT);

	digitalWrite(discoveredLED, LOW);
	digitalWrite(activityLED, LOW);

	Serial.begin(9600);

	// TODO: need this? Flash until the USB serial cable is connected
	while (!Serial)
	{
		flashLed(isDiscovered);
	}

	// now connected to PC
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

	// reciver to filter noise logic
	while (startSession)
	{
		// TODO: space these apart
		if (millis() - lastMills > timerThreshold)
		{
			// read garage car door positions
			upperSwitchReading = digitalRead(upperSwitchPin);
			sendData(uGarageId, upperSwitchReading);

			lowerSwitchReading = digitalRead(lowerSwitchPin);
			sendData(lGarageId, lowerSwitchReading);

			// read house garage door
			garageDoorReading = digitalRead(garageDoorPin);
			sendData(doorId, garageDoorReading);

			// read light data	TODO: read 10 times and take average to reduce noise?
			lightReading = analogRead(lightPin);
			sendData(lightIdentifier, lightReading);

			// read sonar data
			delay(50);	// allow ADC level to settle	TODO: read 10 times and take average to remove noise
			sonarReading = analogRead(sonarPin);
			sendData(sonarId, sonarReading);

			// temp 
			takeTempReading();
			sendTempData(tmpIdC, tempData->tempC);
			sendTempData(tmpIdH, tempData->humidity);
			sendTempData(tmpIdHiC, tempData->heatindexC);

			// read door entrance

			lastMills = millis();
			flashLed(activityLED);

			// check for new requests from controller
			readData();
		}
	}
}

void sendData(char *identifier, int value)
{
	Serial.print(identifier);
	Serial.println(value);
}

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
	tempData->humidity = dht.readHumidity();
	tempData->tempC = dht.readTemperature();
	tempData->tempF = dht.readTemperature(true);

	// calculate the heat index
	tempData->heatindexC = dht.computeHeatIndex(tempData->tempC, tempData->humidity, false);
	tempData->heatIndexF = dht.computeHeatIndex(tempData->tempF, tempData->humidity);
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
				Serial.println("ack");
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
	delay(100);
	digitalWrite(led, LOW);
}
