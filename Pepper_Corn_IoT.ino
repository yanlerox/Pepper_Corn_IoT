// Define Blynk ID and Auth Token
#define BLYNK_TEMPLATE_ID "TMPL3KeeKTfP"
#define BLYNK_TEMPLATE_NAME "White Pepper Monitoring"
#define BLYNK_AUTH_TOKEN "q3SC2joxLfx-8amnZGhbi8edhJMzNIXR"
char auth[] = BLYNK_AUTH_TOKEN;

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial

// For TDS sensor
#define TdsSensorPin A0

// For Dissolved Oxygen Sensor
#define DO_PIN A3
 
#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution
#define TWO_POINT_CALIBRATION 0
#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (131) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃

/*------------------LIBRARY------------------*/
#include <BlynkSimpleShieldEsp8266.h>
#include <ESP8266_Lib.h>
#include <GravityTDS.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <avr/pgmspace.h>
/*-------------------------------------------*/

/*------------------Wi-Fi CREDENTIAL------------------*/
// Set password to "" for open networks.
char ssid[] = "h2oquality 2.4GHz"; //Enter your Wi-Fi SSID
char pass[] = "peppercorns2023"; //Enter your Wi-Fi password
/*----------------------------------------------------*/

/*----------------------VARIABLES DECLARATION----------------------*/
// TDS sensor variables
float tdsValue = 0;
float temperature = 25;       // current temperature for compensation

// Water flow sensor variables
volatile int flow_frequency; // Measures flow sensor pulses
unsigned int l_minute; // Calculated litres/minute
unsigned char flowsensor = 2; // Sensor Input
unsigned long currentTime;
unsigned long cloopTime;

// Turbidity sensor variables
int turbidity_sensor_pin = A1;
float turbidity_sensor_voltage;
int samples = 600;
float NTU;

// Water level sensor variables
long duration;
int distance;
int percentage;

// pH sensor variables
int sensorPin = A2;
int sensorValue = 0;
float ad7 = 322.0;
float ad4 = 435.0;
float phValue;

//Water sensor
#define water_sensor_1_pin 4
#define water_sensor_2_pin 5
#define water_sensor_3_pin 6
#define water_sensor_4_pin 7
int water_sensor_1_value;
int water_sensor_2_value;
int water_sensor_3_value;
int water_sensor_4_value;

// Dissolved oxygen sensor variables
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};
 
uint8_t Temperature;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

  int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
  {
    #if TWO_POINT_CALIBRATION == 00
      uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
      return (voltage_mv * DO_Table[temperature_c] / V_saturation);
    #else
      uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
      return (voltage_mv * DO_Table[temperature_c] / V_saturation);
    #endif
  }
/*-----------------------------------------------------------------*/\

GravityTDS gravityTds;

// Software Serial
#include <SoftwareSerial.h>
SoftwareSerial EspSerial(10, 11); // RX, TX

// Your ESP8266 baud rate:
#define ESP8266_BAUD 9600

BlynkTimer timer;


ESP8266 wifi(&EspSerial);

/*------------------INTERRUPT FUNCTION------------------*/
void flow ()
{
   flow_frequency++;
}
/*------------------------------------------------------*/

/*------------------SEND DATA TO BLYNK------------------*/
void sendSensor()
{
  Blynk.virtualWrite(V0, tdsValue);
  Blynk.virtualWrite(V1, NTU);
  Blynk.virtualWrite(V2, percentage);
  Blynk.virtualWrite(V3, l_minute);
  Blynk.virtualWrite(V4, phValue);
  Blynk.virtualWrite(V5, DO);
  Blynk.virtualWrite(V7, water_sensor_1_value);
  Blynk.virtualWrite(V8, water_sensor_2_value);
  Blynk.virtualWrite(V9, water_sensor_3_value);
  Blynk.virtualWrite(V10, water_sensor_4_value);
}

////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);

  // TDS setup
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();
  
  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  delay(10);
  Blynk.begin(auth, wifi, ssid, pass);
  delay(2000);
  timer.setInterval(1000L, sendSensor);

  // Water flow sensor setup
  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
  attachInterrupt(0, flow, RISING); // Setup Interrupt
  sei(); // Enable interrupts
  currentTime = millis();
  cloopTime = currentTime;

  // Turbidity sensor setup
  pinMode(turbidity_sensor_pin, INPUT);

  //water sensor pin setup
  pinMode(water_sensor_1_pin, INPUT);
  pinMode(water_sensor_2_pin, INPUT);
  pinMode(water_sensor_3_pin, INPUT);
  pinMode(water_sensor_4_pin, INPUT);
}
/*-----------------------------------------*/

/*------------------LOOP------------------*/
void loop()
{
  Blynk.run();
  timer.run();
  TDS();
  WaterFlowRate();
  Turbidity();
  pH();
  DissolvedOxygen();
  water_sensor();
}
/*-----------------------------------------*/

/*------------------TDS FUNCTION------------------*/
void TDS()
{
  gravityTds.setTemperature(temperature);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();
}
/*------------------------------------------------*/

/*------------------WATER FLOW RATE FUNCTION------------------*/
void WaterFlowRate()
{
  currentTime = millis();
  // Every second, calculate and print litres/hour
    if(currentTime >= (cloopTime + 1000))
    {
      cloopTime = currentTime; // Updates cloopTime
      // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
      l_minute = (flow_frequency / 7.5); // (Pulse frequency / 7.5Q = flowrate in L/minute)
      flow_frequency = 0; // Reset Counter
    }
}
/*-------------------------------------------------------------*/

/*------------------TURBIDITY SENSOR FUNCTION------------------*/
void Turbidity()
{
  turbidity_sensor_voltage = 0;
  for(int i=0; i<samples; i++)
  {
    turbidity_sensor_voltage += ((float)analogRead(turbidity_sensor_pin)/1023)*5;
  }
  
  turbidity_sensor_voltage = turbidity_sensor_voltage/samples;
  
  turbidity_sensor_voltage = round_to_dp(turbidity_sensor_voltage, 2);
    
    if(turbidity_sensor_voltage < 2.5){
      NTU = 3000;
    }else{
      NTU = -1120.4*square(turbidity_sensor_voltage)+5742.3*turbidity_sensor_voltage-3323.9;
    }
}

float round_to_dp(float in_value, int decimal_place)
{
  float multiplier = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplier)/multiplier;
  return in_value;
}
/*-------------------------------------------------------------*/


/*------------------pH VALUE FUNCTION------------------*/
void pH()
{
  int currentValue = 0;

  for (int i=0; i < 10; i++)
  {
    currentValue += analogRead(sensorPin);
  }
  sensorValue = (currentValue/10);
  float m = (-3.0 / (ad4-ad7));
  float c = 7 - (m*ad7);
  float a = ((m*sensorValue)+c);
  phValue = 5;//preset
}
/*-----------------------------------------------------*/

/*------------------DISSOLVED OXYGEN FUNCTION------------------*/
void DissolvedOxygen()
{
  Temperature = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  DO = (readDO(ADC_Voltage, Temperature))/1000;
}
/*-------------------------------------------------------------*/

void water_sensor()
{
  water_sensor_1_value = digitalRead(water_sensor_1_pin);
  water_sensor_2_value = digitalRead(water_sensor_2_pin);
  water_sensor_3_value = digitalRead(water_sensor_3_pin);
  water_sensor_4_value = digitalRead(water_sensor_4_pin);
}
