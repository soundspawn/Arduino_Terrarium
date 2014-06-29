
/*
 * Arduino-Terrarium.cpp
 *
 * Main Program to control in-home Terrarium
 */
#include "Arduino.h"
#include "Timer.h"
#include <Dht11.h>

Timer t;

byte temperature = 0;
byte humidity = 0;

#define SERIAL

enum {
    // The data I/O pin connected to the DHT11 sensor
    DHT_DATA_PIN = 13,
    // The baud rate of the serial interface
    SERIAL_BAUD  = 9600,

    COLOR_SELECT_INPUT = A0,
    DIMMER_INPUT = A1,
    HEATER_RELAY_PIN = 8,
};

Dht11 sensor(DHT_DATA_PIN);

enum {
  REDLIGHT_PIN = 2,
  GREENLIGHT_PIN = 3,
  BLUELIGHT_PIN = 4,
  COLOR_DIVISIONS = (int)(1024 / 3)+2,
};

byte lightPins[] = {REDLIGHT_PIN,GREENLIGHT_PIN,BLUELIGHT_PIN};

//Celsius to Fahrenheit conversion
double Fahrenheit(double celsius)
{
	return 1.8 * celsius + 32;
}

/*
 * setup
 *
 * One-time initialization of the module.
 */
void setup() {
  #ifdef SERIAL
    Serial.begin(SERIAL_BAUD);
    Serial.println("Serial Connection Established");
  #endif
  
  //Set up color select pins
  pinMode(COLOR_SELECT_INPUT, INPUT);
  pinMode(DIMMER_INPUT, INPUT);
  pinMode(REDLIGHT_PIN, OUTPUT);
  pinMode(GREENLIGHT_PIN, OUTPUT);
  pinMode(BLUELIGHT_PIN, OUTPUT);
  
  //Setup Outputs
  pinMode(HEATER_RELAY_PIN, OUTPUT);

  //Set Temperature/Humidity Update Timer
  t.every(60000, updateTempHum);
  delay(1000);
  updateTempHum();
}

void invalidateTempReadings(){
  temperature = 0;
  humidity = 0;
}

void updateTempHum(){
  static byte TempUpdateTimer = 255;

  switch (sensor.read()) {
    case Dht11::OK:
      if(sensor.getTemperature() == 0){
        t.after(200,updateTempHum);
        return;
      }
      temperature = Fahrenheit(sensor.getTemperature());
      humidity = sensor.getHumidity();
      //Refresh a timer to invalidate the temp/hum readings
      t.stop(TempUpdateTimer);
      TempUpdateTimer = t.after(5*60000,invalidateTempReadings);
      #ifdef SERIAL
        Serial.print("Humidity (%): ");
        Serial.println(humidity);
        Serial.print("Temperature (F): ");
        Serial.println(temperature);
      #endif
      break;
    case Dht11::ERROR_CHECKSUM:
      #ifdef SERIAL
        Serial.println("Checksum error");
        t.after(200,updateTempHum);
      #endif
      break;
    case Dht11::ERROR_TIMEOUT:
      #ifdef SERIAL
        Serial.println("Timeout error");
        t.after(200,updateTempHum);
      #endif
      break;
    default:
      #ifdef SERIAL
        Serial.println("Unknown error");
      #endif
      break;
  }

  //Decide what to do with the heater
  heaterLogic();
}

void heaterLogic(){
  if(temperature == 0){
    //error, make no decisions
    return;
  }
  if(temperature < 74){
    digitalWrite(HEATER_RELAY_PIN, HIGH);
    #ifdef SERIAL
      Serial.println("Turning Heater On");
    #endif
  }else{
    digitalWrite(HEATER_RELAY_PIN, LOW);
    #ifdef SERIAL
      Serial.println("Turning Heater Off");
    #endif
  }
}

/*
 * loop
 *
 * Code to be executed repeatedly.
 */
void loop() {
  static double sensorMod = 0;
  static int colorSelect = 0;
  
  static int dimmer;
  
  t.update();
   
   colorSelect = lightPins[(int)(analogRead(COLOR_SELECT_INPUT) / COLOR_DIVISIONS)];
   dimmer = analogRead(DIMMER_INPUT);
   analogWrite(colorSelect, dimmer / 4);
   /*
   Serial.print(F("Color "));
   Serial.print(colorSelect);
   Serial.print(" = ");
   Serial.println(dimmer / 4);
   */
}
