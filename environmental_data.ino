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
// TPH ---------//
float temp;
float hum;
float pres;
//----------------//

// Battery and Solar //

#define batPin A6
#define solarPin A7
float vBat, vRead;
float vSolar, vReadS;

bool readBat = true;
bool readSolar = true;

//---------------------//

//----------- MQ9 Gas Sensors Definitions -----------------------------------//
#define R0 0.15    // R0 as set by calibration testing
#define MQ9_Pin A0

int ch4Value = 0;
int coValue = 0; 
int lpgValue = 0;
//---------------------------------------------------------------------------///

// -------------------------- CO2 Definitions --------------------------------//
#include <SoftwareSerial.h>
SoftwareSerial s_serial(4, 5);      // TX, RX

#define sensor s_serial

const unsigned char cmd_get_sensor[] =
{
    0xff, 0x01, 0x86, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x79
};

float CO2PPM =420;

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
  sensor.begin(9600);
  debugSerial.println("Done");

  //----------------  MQ9 Setup ---------//
    pinMode(MQ9_Pin, INPUT);
   //------------------------------------//

   //------------------ Batt and Solar ---//
    pinMode(batPin, INPUT);
    pinMode(solarPin, INPUT);
   //-------------------------------------//
  
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
    dataReceive();
    gasReadings(ch4Value, coValue, lpgValue);

    // ------------- Batt and Solar ------------------//
        if(readBat)
      {
        vRead = (analogRead(batPin)/1024.0f)*3.3f;
        vBat = vRead/(10.0/14.7);
      }
      if(readSolar)
      {
        vReadS = (analogRead(solarPin)/1024.0f)*3.3f;
        vSolar = vReadS * 2.0f;
      }    
    //-------------------------------------------------//
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
  payload.addNumber(CO2PPM);
  payload.addNumber(coValue);
  payload.addNumber(ch4Value);
  payload.addNumber(lpgValue);
  payload.addNumber(vBat);
  payload.addNumber(vSolar);
  
  
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
  debugSerial.print(CO2PPM);
  debugSerial.println(" ppm");
  
  debugSerial.print("CH4: ");
  debugSerial.print(ch4Value);
  debugSerial.println(" ppm");
  
  debugSerial.print("CO: ");
  debugSerial.print(coValue);
  debugSerial.println(" ppm");
  
  debugSerial.print("LPG: ");
  debugSerial.print(lpgValue);
  debugSerial.println(" ppm");

  debugSerial.print("BAT: ");
  debugSerial.print(vBat);
  debugSerial.println(" V");

  debugSerial.print("SOLAR: ");
  debugSerial.print(vSolar);
  debugSerial.println(" V");
  
  debugSerial.println();

}


bool dataReceive(void)
{
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

    /*for(int j=0; j<9; j++)
    {
        Serial.print(data[j]);
        Serial.print(" ");
    }*/
    

    if((i != 9) || (1 + (0xFF ^ (byte)(data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7]))) != data[8])
    {
        return false;
    }

    CO2PPM = (int)data[2] * 256 + (int)data[3];

    return true;
}

void gasReadings(int & ch4Value, int & coValue, int & lpgValue)
{
  float sensor_volt;     
  float RS_gas; // Get value of RS in a GAS
  float ratio; // Get ratio RS_GAS/RS_air
  int sensorValue = analogRead(MQ9_Pin);
  sensor_volt=(float)sensorValue/1024*5.0;
  RS_gas = (5.0-sensor_volt)/sensor_volt; // omit *RL
  
  ratio = RS_gas/R0;  // ratio = RS/R0

  ch4Value = pow((19.821/ratio),2.739);
  coValue = pow((14.882/ratio),2.315);
  lpgValue = pow((21.161/ratio),2.222);
}
