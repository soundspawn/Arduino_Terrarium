/*
 * Arduino-Terrarium.cpp
 *
 * Main Program to control in-home Terrarium
 */
#include "Arduino.h"
#include "Timer.h"
#include <Dht11.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Ethernet.h>
#include <avr/wdt.h>
#include <JsonArray.h>
#include <JsonHashTable.h>
#include <JsonObjectBase.h>
#include <JsonParser.h>

Timer t;

byte desired_temperature = 76;
byte temperature_allowance = 3;
byte temperature = 0;
byte humidity = 0;
byte heater_on = 0;

//Color Intensities
byte colors[] = {0,0,0};
float gradient[] = {0,0,0};
byte color_gradient_timer = 255;

const char genericAjaxSuccess[] PROGMEM = "{\"result\":true";
const char genericAjaxFailure[] PROGMEM = "{\"result\":false";
const char genericAjaxClose[] PROGMEM = "}";

#define SERIALCOM

enum {
    // The data I/O pin connected to the DHT11 sensor
    DHT_DATA_PIN = 9,
    // The baud rate of the serial interface
    SERIAL_BAUD  = 9600,

    COLOR_SELECT_INPUT = A0,
    DIMMER_INPUT = A1,
    HEATER_RELAY_PIN = 8,
};

Dht11 sensor(DHT_DATA_PIN);

#define LIGHT_COLORS 3
enum {
  REDLIGHT_PIN = 2,
  GREENLIGHT_PIN = 3,
  BLUELIGHT_PIN = 4,
  COLOR_DIVISIONS = (int)(1024 / (LIGHT_COLORS+1))+2,
};
byte lightPins[] = {REDLIGHT_PIN,GREENLIGHT_PIN,BLUELIGHT_PIN};

enum {
  I2C_ADDR = 0x3F,
  BACKLIGHT_PIN = 3,
  En_pin = 2,
  Rw_pin = 1,
  Rs_pin = 0,
  D4_pin = 4,
  D5_pin = 5,
  D6_pin = 6,
  D7_pin = 7,
};
LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin,3, POSITIVE);

byte mac[] = { 0xDE, 0xAD, 0xB0, 0xEF, 0xFE, 0xAF };
IPAddress ip(192,168,1, 16);
EthernetServer server(80);
EthernetClient client;
EthernetClient serverajax;

boolean ServerConnected = false;
int ServerTimeout;

JsonParser<32> parser;
JsonHashTable hashTable;

//Celsius to Fahrenheit conversion
double Fahrenheit(double celsius){
  return 1.8 * celsius + 32;
}

/*
 * setup
 *
 * One-time initialization of the module.
 */
void setup() {
  #ifdef SERIALCOM
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("Serial Connection Established"));
  #endif
  
  lcd.begin (20,4);
  // Switch on the backlight
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);

  //Set up color select pins
  pinMode(COLOR_SELECT_INPUT, INPUT);
  pinMode(DIMMER_INPUT, INPUT);
  pinMode(REDLIGHT_PIN, OUTPUT);
  pinMode(GREENLIGHT_PIN, OUTPUT);
  pinMode(BLUELIGHT_PIN, OUTPUT);
  
  //Setup Outputs
  pinMode(HEATER_RELAY_PIN, OUTPUT);

  //Timer-based Function
  //These will set a timer upon their completion
  updateTempHum();
  ServerTime();

  Ethernet.begin(mac, ip);
  server.begin();
}

int memoryTest() {
  int byteCounter = 0; // initialize a counter
  byte *byteArray; // create a pointer to a byte array
  // More on pointers here: http://en.wikipedia.org/wiki/Pointer#C_pointers

  // use the malloc function to repeatedly attempt allocating a certain number of bytes to memory
  // More on malloc here: http://en.wikipedia.org/wiki/Malloc
  while ( (byteArray = (byte*) malloc (byteCounter * sizeof(byte))) != NULL ) {
    byteCounter++; // if allocation was successful, then up the count for the next try
    free(byteArray); // free memory after allocating it
  }

  free(byteArray); // also free memory after the function finishes
  return byteCounter; // send back the highest number of bytes successfully allocated
}

void invalidateTempReadings(){
  #ifdef SERIALCOM
    Serial.println(F("Invalidating Temperature Readings"));
  #endif
  temperature = 0;
  humidity = 0;
  digitalWrite(HEATER_RELAY_PIN, LOW);
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
      lcd.setCursor(0,1);
      lcd.print(F("Temperature: "));
      lcd.print(temperature);
      lcd.print(F(" F"));
      lcd.setCursor(0,2);
      lcd.print(F("Humidity: "));
      lcd.print(humidity);
      lcd.print(F(" %"));
      //Refresh a timer to invalidate the temp/hum readings
      t.stop(TempUpdateTimer);
      TempUpdateTimer = t.after(300000,invalidateTempReadings);
      #ifdef SERIALCOM
        Serial.print(F("Humidity (%): "));
        Serial.println(humidity);
        Serial.print(F("Temperature (F): "));
        Serial.println(temperature);
      #endif
      break;
    case Dht11::ERROR_CHECKSUM:
      #ifdef SERIALCOM
        Serial.println(F("Checksum error"));
      #endif
      t.after(200,updateTempHum);
      return;
    case Dht11::ERROR_TIMEOUT:
      #ifdef SERIALCOM
        Serial.println(F("Timeout error"));
      #endif
      t.after(200,updateTempHum);
      return;
    default:
      #ifdef SERIALCOM
        Serial.println(F("Unknown error"));
      #endif
      t.after(200,updateTempHum);
      return;
  }

  //Decide what to do with the heater
  heaterLogic();
  t.after(60000,updateTempHum);
}

void heaterLogic(){
  if(temperature == 0){
    //error, make no decisions
    return;
  }
  if((heater_on == 0 && temperature < (desired_temperature-temperature_allowance)) || 
     (heater_on == 1 && temperature < desired_temperature)
  ){
    digitalWrite(HEATER_RELAY_PIN, HIGH);
    heater_on = 1;
    #ifdef SERIALCOM
      Serial.println(F("Turning Heater On"));
    #endif
  }else{
    digitalWrite(HEATER_RELAY_PIN, LOW);
    heater_on = 0;
    #ifdef SERIALCOM
      Serial.println(F("Turning Heater Off"));
    #endif
  }
}

void set_color_intensity(byte color, byte intensity){
  analogWrite(lightPins[color], intensity);
  colors[color] = intensity;
}

void set_color_gradient(byte red, byte green, byte blue, unsigned long transition_time){
  signed int gradients[3];
  signed long seconds = transition_time/1000;
  signed long secondMod = 1;
  byte i = 0;
  byte lowest = 0;
  gradients[0] = (colors[0] - red) / seconds;
  gradients[1] = (colors[1] - green) / seconds;
  gradients[2] = (colors[2] - blue) / seconds;
  //increase the secondMod until at least one color is incrementing by a whole number
  for(i=1;i<3;i++){
    if(gradients[i]>0 && gradients[i] < gradients[lowest]){
      lowest = i;
    }
  }
  while(gradients[lowest]>0 && gradients[lowest]*secondMod < 1){
    secondMod++;
  }
  //Stop any existing gradient timer
  t.stop(color_gradient_timer);
  if(gradients[lowest] > 0 && seconds > 0){
    gradient[0] = gradients[0]*secondMod;
    gradient[1] = gradients[1]*secondMod;
    gradient[2] = gradients[2]*secondMod;
    color_gradient_timer = t.every(1000*secondMod,color_gradient,floor(seconds/secondMod));
  }
}

void color_gradient(){
  for(int i = 0; i < 3; i++){
    set_color_intensity(i,colors[i] + gradient[i]);
  }
}

void set_rgb(byte red, byte green, byte blue){
  set_color_intensity(0, red);
  set_color_intensity(1, green);
  set_color_intensity(2, blue);
}

void httpServer(){
  //Web Server
  client = server.available();
  if (client) {
    String HTTP_req = "";
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    char buffer[60] = {};
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        HTTP_req += c;

        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: application/json"));
          client.println(F("Connection: close"));  // the connection will be closed after completion of the response
          client.println();

          if(HTTP_req.indexOf("/get_temperature") == 4){
            strcpy_P(buffer,genericAjaxSuccess);
            client.print(buffer);
            client.print(F(",\"temperature\":"));
            client.print(temperature);
            client.print(F(",\"humidity\":"));
            client.print(humidity);
            client.print(F(",\"thermostat\":"));
            client.print(desired_temperature);
            client.print(F(",\"temperature_allowance\":"));
            client.print(temperature_allowance);
            client.print(F(",\"heater_status\":"));
            client.print(heater_on);
            strcpy_P(buffer,genericAjaxClose);
            client.print(buffer);
          }

          if(HTTP_req.indexOf("/set_color/") == 4){
            String sub = "";
            sub = HTTP_req.substring(15,100);
            sub = sub.substring(0,sub.indexOf("/"));
            byte color = sub.toInt();
            sub = HTTP_req.substring(15+sub.length()+1,100);
            byte intensity = sub.toInt();
            set_color_intensity(color, intensity);
            strcpy_P(buffer,genericAjaxSuccess);
            client.print(buffer);
            strcpy_P(buffer,genericAjaxClose);
            client.print(buffer);
          }

          if(HTTP_req.indexOf("/set_rgb/") ==  4){
            String sub = "";
            String sub2 = "";
            sub = HTTP_req.substring(13,100);
            sub = sub.substring(0,sub.indexOf("/"));
            byte red = sub.toInt();
            sub2 = HTTP_req.substring(13+sub.length()+1,100);
            sub2 = sub2.substring(0,sub2.indexOf("/"));
            byte green = sub2.toInt();
            sub2 = HTTP_req.substring(13+sub.length()+sub2.length()+2,100);
            byte blue = sub2.toInt();

            #ifdef SERIALCOM
              Serial.println(F("Setting RGB: "));
              Serial.println(red);
              Serial.println(green);
              Serial.println(blue);
            #endif
            set_rgb(red,green,blue);

            strcpy_P(buffer,genericAjaxSuccess);
            client.print(buffer);
            strcpy_P(buffer,genericAjaxClose);
            client.print(buffer);
          }

          if(HTTP_req.indexOf("/set_thermostat/") == 4){
            String sub = "";
            sub = HTTP_req.substring(20,100);
            sub = sub.substring(0,sub.indexOf("/"));
            desired_temperature = sub.toInt();
            sub = HTTP_req.substring(20+sub.length()+1,100);
            temperature_allowance = sub.toInt();
            strcpy_P(buffer,genericAjaxSuccess);
            client.print(buffer);
            strcpy_P(buffer,genericAjaxClose);
            client.print(buffer);
            heaterLogic();
          }

          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
      // close the connection:
      client.stop();
      // give the web browser time to receive the data
      delay(1);
  }
  // close the connection:
  client.stop();
}

char* Ajax(char *url){
  String message;
  String submessage;
  int connectLoop = 0;
  char c;
  message = "";
  if(serverajax.connect("soundspawn.com",80)){
    serverajax.print(F("GET /proxy/proxy.php?url=https%3A//jarvis.soundspawn.com/"));
    serverajax.print(url);
    serverajax.println(F(" HTTP/1.0"));
    serverajax.println(F("Host: soundspawn.com"));
    serverajax.println(F("Connection: close"));
    serverajax.println();
    while(serverajax.connected()){
      connectLoop++;
      if(serverajax.available()){
        c = serverajax.read();
        connectLoop = 0;
        message += c;
      }
      if(connectLoop > 10000){
        return (char*)F("{\"result\":false,\"message\":\"AJAX Timeout\"}");
        serverajax.stop();
      }
    }
    submessage = message.substring(message.indexOf("{"),message.length());
    submessage += "}";
    char d2[message.length()];
    submessage.toCharArray(d2,submessage.length());
    serverajax.stop();
    if(submessage.length() == 1){
      return (char*)F("{\"result\":false,\"message\":\"Arduino Ajax Abandoned Connection before response\"}");
    }
    return d2;
  }
  return (char*)F("{\"result\":false,\"message\":\"Arduino Ajax Connection Error\"}");
}

void ServerTime(){
  char* ajax = Ajax("terrarium/get_time");
  hashTable = parser.parseHashTable(ajax);
  if(!hashTable.success()){
    t.after(1000,ServerTime);
    return;
  }
  unsigned long time = hashTable.getLong("now");
  lcd.setCursor(0,3);
  lcd.print(time);
  lcd.print(F("    "));
  lcd.print(memoryTest());

  t.after(60000,ServerTime);
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

  //Receive http request
  httpServer();
   
  colorSelect = (int)(analogRead(COLOR_SELECT_INPUT) / COLOR_DIVISIONS);
  dimmer = analogRead(DIMMER_INPUT);
  set_color_intensity(colorSelect, dimmer / 4);
  if((int)analogRead(COLOR_SELECT_INPUT) / COLOR_DIVISIONS < 3){
    delay(200);
    lcd.setCursor(0,0);
    lcd.print(F("Color "));
    lcd.print(colorSelect);
    lcd.print(F(" = "));
    lcd.print(dimmer / 4);
  }
}
