/*
 *  This sketch sends ads1115 current sensor data via HTTP POST request to thingspeak server.
 *  It needs the following libraries to work (besides the esp8266 standard libraries supplied with the IDE):
 *
 *  - https://github.com/adafruit/Adafruit_ADS1X15
 *
 *  designed to run directly on esp8266-01 module, to where it can be uploaded using this marvelous piece of software:
 *
 *  https://github.com/esp8266/Arduino
 *
 *  2015 Tisham Dhar
 *  licensed under GNU GPL
 */
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

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
#define WSSID "xxxxx"
#define PASS "xxxx"

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
double lastFilteredV,filteredV; //Filtered_ is the raw analog value minus the DC offset
int sampleV;                 //sample_ holds the raw analog read value
int sampleI; 

double offsetV;                          //Low-pass filter output
double offsetI;                          //Low-pass filter output

double realPower,
       apparentPower,
       powerFactor,
       Vrms,
       Irms;
double phaseShiftedV; //Holds the calibrated phase shifted voltage.
int startV; //Instantaneous voltage at start of sample window.
double sqV,sumV,sqI,sumI,instP,sumP; //sq = squared, sum = Sum, inst = instantaneous
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
  boolean st=false;                                  //an indicator to exit the while loop

  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(st==false)                                   //the while loop...
  {
     startV = ads.readADC_Differential_2_3();                    //using the voltage waveform
     if ((abs(startV) < (ADC_COUNTS*0.55)) && (abs(startV) > (ADC_COUNTS*0.45))) st=true;  //check its within range
     if ((millis()-start)>timeout) st = true;
  }
  
  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //------------------------------------------------------------------------------------------------------------------------- 
  start = millis(); 

  while ((crossCount < crossings) && ((millis()-start)<timeout)) 
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
    offsetV = offsetV + ((sampleV-offsetV)/1024);
  filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI-offsetI)/1024);
  filteredI = sampleI - offsetI;
   
    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------  
    sqV= filteredV * filteredV;                 //1) square voltage values
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
    sumP +=instP;                               //Sum  
    
    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength 
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------       
    lastVCross = checkVCross;                     
    if (sampleV > startV) checkVCross = true; 
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;                  
                     
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
  powerFactor=realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
//--------------------------------------------------------------------------------------       
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

  // Initialize Blynk, and wait for a connection before doing anything else
  Serial.println("Connecting to WiFi and Blynk");
  Blynk.begin(auth, WSSID, PASS);
  while (!Blynk.connected())
    Blynk.run();
  Serial.println("Blynk connected! Energy monitor starting.");

  setLED(0, 32, 0); // LED green  
  // init done
}

void loop() {
  measureVI();
  Blynk.run(); // Initiates Blynk
}

bool firstConnect = true;
BLYNK_CONNECTED()
{
  if (firstConnect) // When we first connect to Blynk
  {
    // Two options here. Either sync values from phone to Blynk Board:
    //Blynk.syncAll(); // Uncomment to enable.
    // Or set phone variables to default values of the globals:
    Blynk.virtualWrite(VIRTUAL_POWER, realPower);
    Blynk.virtualWrite(VIRTUAL_CURRENT, Irms);
    Blynk.virtualWrite(VIRTUAL_VOLTAGE, Vrms);
    Blynk.virtualWrite(VIRTUAL_PF, powerFactor);

    // Print a splash screen:
    lcd.clear();
    lcd.print(0, 0, "Energy Monitor ");
    lcd.print(0, 1, "     Ready      ");
  }
}

void measureVI()
{
  calcVI(20,2000); 

  // Write the values to Blynk:
  Blynk.virtualWrite(VIRTUAL_POWER, realPower);
  Blynk.virtualWrite(VIRTUAL_CURRENT, Irms);
  Blynk.virtualWrite(VIRTUAL_VOLTAGE, Vrms);
  Blynk.virtualWrite(VIRTUAL_PF, powerFactor);

}

void setLED(uint8_t red, uint8_t green, uint8_t blue)
{
  rgb.setPixelColor(0, rgb.Color(red, green, blue));
  rgb.show();
}

