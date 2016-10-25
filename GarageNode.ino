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

DHT22 (also RHT03)
Data(pin2) D4 via 10K pullup resistor from data pin to power of the sensor

TODO: re-evaluate these LED needs
D6 Disovered LED
D7 Send LED

switches have hardware debounce
D5 = car garage door lower pin
D6 = car garage door upper pin

*/

#include "DHT.h"

// LED			TODO: add LED and correct pin
const uint8_t discoveredLED = 13;
const uint8_t activityLED = 4;

/*
// IR
const uint8_t irPin = A0;
int irReading = 0;
int irLastReading = 0;
char *irIdentifier = "d";		// Car Garage (D)oor
*/

// light sensor
const uint8_t lightPin = A1;
int lightReading = 0;
int lightLastReading = 0;
char *lightIdentifier = "l";	// (L)ights, not garage door opener light

// temp sensor
#define DHTPIN 2
DHT dht(DHTPIN, DHT22);
char *tmpIdC = "tc";	// temp in C
char *tmpIdF = "tf";	// temp in F
char *tmpIdH = "th";	// humidity
char *tmpIdHiC = "ta";	// heat index in C
char *tmpIdHiF = "tb";	// heat index in F

// car (G)arage door reed switchs
// two sensors GarageLower and GarageUpper
const uint8_t lowerSwitchPin = 5;
const uint8_t upperSwitchPin = 6;
char *lGarageId = "gl";
char *uGarageId = "gu";

uint8_t lowerSwitchReading = 0;
uint8_t upperSwitchReading = 0;
/*
uint8_t lastLowerSwitchState = 0;
uint8_t lastUpperSwitchState = 0;
*/

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
int timerThreshold = 5000;

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

	pinMode(lowerSwitchPin, INPUT);
	pinMode(upperSwitchPin, INPUT);

	digitalWrite(discoveredLED, LOW);
	digitalWrite(activityLED, LOW);

	Serial.begin(9600);

	// flash until the USB serial cable is connected
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
			lowerSwitchReading = digitalRead(lowerSwitchPin);
			sendData(lGarageId, lowerSwitchReading);

			/*
			// IR reading
			irReading = analogRead(irPin);
			sendData(irIdentifier, irReading);
			*/

			// allow ADC level to settle
			delay(50);
			lightReading = analogRead(lightPin);
			sendData(lightIdentifier, lightReading);

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
