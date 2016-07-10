/*
    This sketch sends ads1115 current sensor data via HTTP POST request to thingspeak server.
    It needs the following libraries to work (besides the esp8266 standard libraries supplied with the IDE):

    - https://github.com/adafruit/Adafruit_ADS1X15

    designed to run directly on esp8266-01 module, to where it can be uploaded using this marvelous piece of software:

    https://github.com/esp8266/Arduino

    2015 Tisham Dhar
    licensed under GNU GPL
*/
#include <FS.h> //this needs to be first, or it all crashes and burns..

#if PLATFORM_ID == 88
#include "application.h"
#else
#include <Wire.h>
#include <SPI.h>
#endif

#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
//#define Serial SerialUSB
//#define Serial Serial1
/*
  #if defined(ARDUINO)
  SYSTEM_MODE(MANUAL);//do not connect to cloud
  #else
  SYSTEM_MODE(AUTOMATIC);//connect to cloud
  #endif
*/
#include <Adafruit_ADS1015.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_NeoPixel.h>

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[36] = "YOUR_BLYNK_TOKEN";

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


bool notifyFlag = false;
#define VIRTUAL_EN_PUSH V0
#define VIRTUAL_POWER   V1
#define VIRTUAL_VOLTAGE V2
#define VIRTUAL_CURRENT V3
#define VIRTUAL_LCD     V4
#define VIRTUAL_PF      V5
WidgetLCD lcd(VIRTUAL_LCD);
bool pushEnabled = false;

//////////////////////////
// Hardware Definitions //
//////////////////////////
const int LED_PIN = 5;
const int RGB_PIN = 4;
Adafruit_NeoPixel rgb = Adafruit_NeoPixel(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
void setLED(uint8_t red, uint8_t green, uint8_t blue);

#ifdef ESP8266
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif


Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Maximum value of ADS
#define ADC_COUNTS 32768
#define PHASECAL 1.7
#define VCAL 0.8
#define ICAL 0.04

double filteredI;
double lastFilteredV, filteredV; //Filtered_ is the raw analog value minus the DC offset
int sampleV;                 //sample_ holds the raw analog read value
int sampleI;

double offsetV;                          //Low-pass filter output
double offsetI;                          //Low-pass filter output

//Scaling for final calibration
double vmult=1.0,imult=1.0,phmult=1.0;

double realPower,
       apparentPower,
       powerFactor,
       Vrms,
       Irms;
double phaseShiftedV; //Holds the calibrated phase shifted voltage.
int startV; //Instantaneous voltage at start of sample window.
double sqV, sumV, sqI, sumI, instP, sumP; //sq = squared, sum = Sum, inst = instantaneous
boolean lastVCross, checkVCross; //Used to measure number of times threshold is crossed.

double squareRoot(double fg)
{
  double n = fg / 2.0;
  double lstX = 0.0;
  while (n != lstX)
  {
    lstX = n;
    n = (n + fg / n) / 2.0;
  }
  return n;
}

void calcVI(unsigned int crossings, unsigned int timeout)
{

  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  boolean st = false;                                //an indicator to exit the while loop

  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  /*
    while(st==false)                                   //the while loop...
    {
     startV = ads.readADC_Differential_2_3();                    //using the voltage waveform
     if ((startV < (ADC_COUNTS*0.6)) && (startV > (-ADC_COUNTS*0.6))) st=true;  //check its within range
     if ((millis()-start)>timeout) st = true;
    }
  */
  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millis();

  while ((crossCount < crossings) && ((millis() - start) < timeout))
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = ads.readADC_Differential_0_1();                 //Read in raw voltage signal
    sampleI = ads.readADC_Differential_2_3();                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV - offsetV) / 1024);
    filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI - offsetI) / 1024);
    filteredI = sampleI - offsetI;

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV = filteredV * filteredV;                //1) square voltage values
    sumV += sqV;                                //2) sum

    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
    instP = phaseShiftedV * filteredI;          //Instantaneous Power
    sumP += instP;                              //Sum

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
    else checkVCross = false;
    if (numberOfSamples == 1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.
  float multiplier = 0.125F; /* ADS1115 @ +/- 4.096V gain (16-bit results) */
  double V_RATIO = VCAL * multiplier;
  Vrms = V_RATIO * squareRoot(sumV / numberOfSamples);

  double I_RATIO = ICAL * multiplier;
  Irms = I_RATIO * squareRoot(sumI / numberOfSamples);

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor = realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
  //--------------------------------------------------------------------------------------
}

void readBlynkConfig()
{
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  //Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    //Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      //Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        //Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          //Serial.println("\nparsed json");
          strcpy(auth, json["auth"]);

        } else {
          //Serial.println("failed to load json config");
        }
      }
    }
  } else {
    //Serial.println("failed to mount FS");
  }
  //end read
}

void saveBlynkConfig()
{
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    //Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["vmult"] = vmult;
    json["imult"] = vmult;
    json["phmult"] = vmult;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      //Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
}

void readMultConfig()
{
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  //Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    //Serial.println("mounted file system");
    if (SPIFFS.exists("/mult.json")) {
      //file exists, reading and loading
      //Serial.println("reading config file");
      File configFile = SPIFFS.open("/mult.json", "r");
      if (configFile) {
        //Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          //Serial.println("\nparsed json");
          vmult = json["vmult"];
          imult = json["imult"];
          phmult = json["phmult"];
        } else {
          //Serial.println("failed to load json config");
        }
      }
    }
  } else {
    //Serial.println("failed to mount FS");
  }
  //end read
}

void saveMultConfig()
{
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    //Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["vmult"] = vmult;
    json["imult"] = imult;
    json["phmult"] = phmult;

    File configFile = SPIFFS.open("/mult.json", "w");
    if (!configFile) {
      //Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Turn off blue LED
  rgb.begin(); // Set up WS2812
  setLED(0, 0, 32); // LED blue

  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  ads.begin();

  Wire.begin();

  //Read previous config
  readBlynkConfig();
  readMultConfig();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", auth, 33);

  //Use wifi manager to get config
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_blynk_token);

  //first parameter is name of access point, second is the password
  wifiManager.autoConnect("Blynkwhatnick", "EnergyMonitor");

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(auth, custom_blynk_token.getValue());

  saveBlynkConfig();

  // Initialize Blynk, and wait for a connection before doing anything else
  Serial.println("Connecting to Blynk Server");
  Blynk.config(auth);
  while (!Blynk.connected())
    Blynk.run();
  Serial.println("Blynk connected! Energy monitor starting.");

  setLED(0, 32, 0); // LED green
  // init done
}

int lastTick = 0;
void loop() {
  
  Blynk.run(); // Initiates Blynk
  if ((millis() - lastTick > 10000)) { //60000 - 1 minute
    measureVI();
    lastTick = millis();
    Serial.print("Vrms:");
    Serial.println(Vrms*vmult);
    Serial.print("Irms:");
    Serial.println(Irms*imult);
    Serial.print("Power:");
    Serial.println(realPower * vmult * imult);
    Serial.print("p.f.:");
    Serial.println(powerFactor * phmult);
  }
  yield();
}

bool firstConnect = true;
BLYNK_CONNECTED()
{
  if (firstConnect) // When we first connect to Blynk
  {
    // Two options here. Either sync values from phone to Blynk Board:
    //Blynk.syncAll(); // Uncomment to enable.
    // Or set phone variables to default values of the globals:
    Blynk.virtualWrite(VIRTUAL_POWER, realPower * vmult * imult);
    Blynk.virtualWrite(VIRTUAL_CURRENT, Irms * imult);
    Blynk.virtualWrite(VIRTUAL_VOLTAGE, Vrms * vmult);
    Blynk.virtualWrite(VIRTUAL_PF, powerFactor * phmult);

    // Print a splash screen:
    lcd.clear();
    lcd.print(0, 0, "Energy Monitor ");
    lcd.print(0, 1, "     Ready      ");
  }
}

void measureVI()
{
  calcVI(20, 2000);

  // Write the values to Blynk:
  Blynk.virtualWrite(VIRTUAL_POWER, realPower * vmult * imult);
  Blynk.virtualWrite(VIRTUAL_CURRENT, Irms * imult);
  Blynk.virtualWrite(VIRTUAL_VOLTAGE, Vrms * vmult);
  Blynk.virtualWrite(VIRTUAL_PF, powerFactor * phmult);

}

BLYNK_WRITE(V6) //Button Widget is writing to pin V6
{
  int pinData = param.asInt();
  vmult = pinData / 100.0f;
  Serial.print("V Mult: ");
  Serial.println(vmult);
  saveMultConfig();
}

BLYNK_WRITE(V7) //Button Widget is writing to pin V7
{
  int pinData = param.asInt();
  imult = pinData / 100.0f;
  Serial.print("I Mult: ");
  Serial.println(imult);
  saveMultConfig();
}

BLYNK_WRITE(V8) //Button Widget is writing to pin V8
{
  int pinData = param.asInt();
  phmult = pinData / 100.0f;
  Serial.print("Phase Mult: ");
  Serial.println(phmult);
  saveMultConfig();
}

void setLED(uint8_t red, uint8_t green, uint8_t blue)
{
  rgb.setPixelColor(0, rgb.Color(red, green, blue));
  rgb.show();
}

