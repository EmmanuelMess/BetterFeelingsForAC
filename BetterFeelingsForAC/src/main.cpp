#include <Arduino.h>
#include <math.h>
#include <DHT.h>
#include <arduino-heatpumpir/HeatpumpIR.h>
#include <arduino-heatpumpir/PhilcoHeatpumpIR.h>

const uint8_t LED_PIN = 5;
const float LED_ACTIVE_TIME = 1; // in seconds

const float TEMPERATURE_MESSAGE_INTERVAL = 2;//5*60; // in seconds

const uint8_t IR_PIN = 8;

const uint8_t SENSOR_PIN = 2;
const int SENSOR_TYPE = DHT11;

static double secs() {
	return static_cast<double>(millis()) / 1000.0;
}

struct state {
	bool ledActive;
	double ledActiveSince;
	DHT sensor;
	double timeOfLastTemperatureMessage;
	IRSenderBitBang irSender;
	PhilcoHeatpumpIR heatpumpIr;
	double currentTemperature;
};

state currentState = {
	.ledActive = false,
	.ledActiveSince = NAN,
	.sensor = DHT(SENSOR_PIN, SENSOR_TYPE),
	.timeOfLastTemperatureMessage = 0,
	.irSender = IRSenderBitBang(IR_PIN),
	.heatpumpIr = PhilcoHeatpumpIR(),
	.currentTemperature = NAN
};

void setup() {
	Serial.begin(9600);
	currentState.sensor.begin();
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
}

void activateLed() {
	digitalWrite(LED_PIN, HIGH);
	currentState.ledActive = true;
	currentState.ledActiveSince = secs();
}

void deactivateLed() {
	digitalWrite(LED_PIN, LOW);
	currentState.ledActive = false;
	currentState.ledActiveSince = NAN;//NaN
}

struct sensorInfo {
	float temperature;
	float humidity;
};

sensorInfo getSensorInfo() {
	const float temperature = currentState.sensor.readTemperature();
	const float humidity = currentState.sensor.readHumidity();

	if(isnan(temperature) || isnan(humidity)) {
		Serial.println("Error reading temperature and humidity");
	}

	return sensorInfo {
		.temperature = temperature,
		.humidity = humidity
	};
}

double heatIndex(double t, double h) {
	const double c1 = -8.78469475556;
	const double c2 = 1.61139411;
	const double c3 = 2.33854883889;
	const double c4 = -0.14611605;
	const double c5 = -0.012308094;
	const double c6 = -0.0164248277778;
	const double c7 = 2.211732e-3;
	const double c8 = 7.2546e-4;
	const double c9 = -3.582e-6;
	return c1 + c2 * t + c3 * h + c4 * t * h + c5 * t * t + c6 * h * h + c7 * t * t * h + c8 * t * h * h + c9 * t * t * h * h;
}

void sendMessage() {
	Serial.println("Sending message...");
	const long r = lround(currentState.currentTemperature);
	currentState.heatpumpIr.send(currentState.irSender, static_cast<uint8_t>(r));

	activateLed();
	currentState.timeOfLastTemperatureMessage = secs();
}

// the loop function runs over and over again forever
void loop() {
	const double now = secs();

	if (currentState.ledActive && now - currentState.ledActiveSince > LED_ACTIVE_TIME) {
		deactivateLed();
	}

	if(now - currentState.timeOfLastTemperatureMessage > TEMPERATURE_MESSAGE_INTERVAL) {
		const sensorInfo info = getSensorInfo();
		currentState.currentTemperature = heatIndex(info.temperature, info.humidity);
		Serial.print("Heat index ");
		Serial.print(currentState.currentTemperature);
		Serial.print("°C. ");

		sendMessage();
	}
}