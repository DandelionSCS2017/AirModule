#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "AirQuality2.h"
#include <ATT_IOT_LoRaWAN.h>
#include "keys.h"
#include <MicrochipLoRaModem.h>
#include <PayloadBuilder.h>

#define SERIAL_BAUD 57600

#define debugSerial Serial
#define loraSerial Serial1

#define AirQualityPin A0
#define LightSensorPin A2
#define SoundSensorPin A4

#define SEND_EVERY 10000

MicrochipLoRaModem modem(&loraSerial, &debugSerial);
ATTDevice device(&modem, &debugSerial, false, 7000);  // minimum time between 2 messages set at 7000 milliseconds

PayloadBuilder payload(device);

AirQuality2 airqualitysensor;
Adafruit_BME280 tph; // I2C

float soundValue;

float temp;
float hum;
float pres;


void setup() 
{
  pinMode(GROVEPWR, OUTPUT);  // turn on the power for the secondary row of grove connectors
  digitalWrite(GROVEPWR, HIGH);

  debugSerial.begin(SERIAL_BAUD);
  debugSerial.begin(SERIAL_BAUD);
  while((!debugSerial) && (millis()) < 10000){}  // wait until the serial bus is available

  loraSerial.begin(modem.getDefaultBaudRate());  // set baud rate of the serial connection to match the modem
  while((!loraSerial) && (millis()) < 10000){}   // wait until the serial bus is available

  while(!device.initABP(DEV_ADDR, APPSKEY, NWKSKEY))
  debugSerial.println("Ready to send data");
  
  debugSerial.println();
  debugSerial.println("-- Environmental Sensing LoRa experiment --");
  debugSerial.println();

  initSensors();
}

void loop() 
{
  readSensors();
  displaySensorValues();
  sendSensorValues();
  
  debugSerial.print("Delay for: ");
  debugSerial.println(SEND_EVERY);
  debugSerial.println();
  delay(SEND_EVERY);
}

void initSensors()
{
  debugSerial.println("Initializing sensors, this can take a few seconds...");
  
  pinMode(SoundSensorPin, INPUT);
  tph.begin();
  debugSerial.println("Done");
}

void readSensors()
{
    debugSerial.println("Start reading sensors");
    debugSerial.println("---------------------");
    
    soundValue = analogRead(SoundSensorPin);
   /* lightValue = analogRead(LightSensorPin);
    lightValue = lightValue * 3.3 / 1023;  // convert to lux based on the voltage that the sensor receives
    lightValue = pow(10, lightValue);
    */
    temp = tph.readTemperature();
    hum = tph.readHumidity();
    pres = tph.readPressure()/100.0;
}

void process()
{
  while(device.processQueue() > 0)
  {
    debugSerial.print("QueueCount: ");
    debugSerial.println(device.queueCount());
    delay(10000);
  }
}

void sendSensorValues()
{
  payload.reset();

  payload.addNumber(temp);
  payload.addNumber(pres);
  payload.addNumber(hum);
  payload.addNumber(soundValue);
  
  payload.addToQueue(false);  
  process();
}

void displaySensorValues()
{
  debugSerial.print("Sound level: ");
  debugSerial.print(soundValue);
  debugSerial.println(" Analog (0-1023)");
      
  debugSerial.print("Temperature: ");
  debugSerial.print(temp);
  debugSerial.println(" °C");
      
  debugSerial.print("Humidity: ");
  debugSerial.print(hum);
	debugSerial.println(" %");
      
  debugSerial.print("Pressure: ");
  debugSerial.print(pres);
	debugSerial.println(" hPa");
  
}


