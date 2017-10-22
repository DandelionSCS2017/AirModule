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

Adafruit_BME280 tph; // I2C

float soundValue;

float temp;
float hum;
float pres;

// -------------------------- CO2 Definitions --------------------------------//
#include <SoftwareSerial.h>
SoftwareSerial s_serial(2, 3);      // TX, RX

#define sensor s_serial

const unsigned char cmd_get_sensor[] =
{
    0xff, 0x01, 0x86, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x79
};

float CO2Value = 1900;

//---------------------------------------------------------------------------//


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
    CO2Value = co2Read();
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
  payload.addNumber(CO2Value);
  
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

  debugSerial.print("CO2: ");
  debugSerial.print(CO2Value);
  debugSerial.println(" ppm");
  
}



//----------------- Data Receive Method for CO2 Sensor ---------------------------------//
int co2Read(void)
{
    int CO2PPM;
    
    byte data[9];
    int i = 0;

    //transmit command data
    for(i=0; i<sizeof(cmd_get_sensor); i++)
    {
        sensor.write(cmd_get_sensor[i]);
    }
    delay(10);
    //begin reveiceing data
    if(sensor.available())
    {
        while(sensor.available())
        {
            for(int i=0;i<9; i++)
            {
                data[i] = sensor.read();
            }
        }
    }
    
    Serial.println("");

    if((i != 9) || (1 + (0xFF ^ (byte)(data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7]))) != data[8])
    {
        return 0;
    }

    CO2PPM = (int)data[2] * 256 + (int)data[3];

    return CO2PPM;
}

