/*
 * Arduino-Terrarium.cpp
 *
 * Main Program to control in-home Terrarium
 */
#include "settings.h"
#include "Arduino.h"
#include "Timer.h"
#include <Dht11.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <Ethernet.h>
#include <avr/wdt.h>
#include <JsonArray.h>
#include <JsonHashTable.h>
#include <JsonObjectBase.h>
#include <JsonParser.h>
#include <Bounce2.h>

Timer t = Timer(8);

byte desired_temperature = 76;
byte temperature_allowance = 3;
byte desired_humidity = 55;
byte humidity_allowance = 5;
byte temperature = 0;
byte humidity = 0;
byte heater_on = 0;
byte humidifier_on = 0;

//Color Intensities
float colors[] = {0,0,0};
float gradient[] = {0,0,0};

//Timer IDs
byte TempHumTimer = 255;
byte ServerTimeTimer = 255;
byte color_gradient_timer = 255;
byte TempBackOffStateTimer = 255;

const char genericAjaxSuccess[] PROGMEM = "{\"result\":true";
const char genericAjaxFailure[] PROGMEM = "{\"result\":false";
const char genericAjaxClose[] PROGMEM = "}";

Bounce TempBackoffBounce = Bounce();
byte TempBackOffState = 0;

Dht11 sensor(DHT_DATA_PIN);

#define COLOR_DIVISIONS (int)(1024 / (LIGHT_COLORS+1))+2
byte lightPins[] = {REDLIGHT_PIN,GREENLIGHT_PIN,BLUELIGHT_PIN};

LiquidCrystal lcd(LCD_I2C_ADDR,LCD_En_pin,LCD_Rw_pin,LCD_Rs_pin,LCD_D4_pin,LCD_D5_pin,LCD_D6_pin,LCD_D7_pin,3, 1);

byte mac[] = { 0xDE, 0xAD, 0xB0, 0xEF, 0xFE, 0xAF };
IPAddress ip(192,168,1, 16);
EthernetServer server(80);
EthernetClient client;
EthernetClient serverajax;

boolean ServerConnected = false;

JsonParser<32> parser;
JsonHashTable hashTable;

String HTTP_req;
String AjaxMessage;
String submessage;
String sub;
String sub2;
String sub3;

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
  
  lcd.begin (LCD_COLUMNS,LCD_ROWS);

  //Set up color select pins
  pinMode(COLOR_SELECT_INPUT, INPUT);
  pinMode(DIMMER_INPUT, INPUT);
  pinMode(REDLIGHT_PIN, OUTPUT);
  pinMode(GREENLIGHT_PIN, OUTPUT);
  pinMode(BLUELIGHT_PIN, OUTPUT);
  
  //Temperature Backoff Button
  pinMode(TEMP_BACKOFF_PIN,INPUT_PULLUP);
  TempBackoffBounce.attach(TEMP_BACKOFF_PIN);

  //Setup Outputs
  pinMode(HEATER_RELAY_PIN, OUTPUT);

  //String init
  HTTP_req.reserve(500);
  AjaxMessage.reserve(800);
  submessage.reserve(800);
  sub.reserve(100);
  sub2.reserve(100);
  sub3.reserve(100);

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
  digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);
  heater_on = 0;
  humidifier_on = 0;
}

void updateTempHum(){
  static byte TempUpdateTimer = 255;

  switch (sensor.read()) {
    case Dht11::OK:
      if(sensor.getTemperature() == 0){
        TempHumTimer = t.after(500,updateTempHum);
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
      TempHumTimer = t.after(500,updateTempHum);
      return;
    case Dht11::ERROR_TIMEOUT:
      #ifdef SERIALCOM
        Serial.println(F("Timeout error"));
      #endif
      TempHumTimer = t.after(500,updateTempHum);
      return;
    default:
      #ifdef SERIALCOM
        Serial.println(F("Unknown error"));
      #endif
      TempHumTimer = t.after(500,updateTempHum);
      return;
  }

  //Decide what to do with the heater
  heaterLogic();
  humidifierLogic();
  TempHumTimer = t.after(60000,updateTempHum);
}

void heaterLogic(){
  byte atemp = desired_temperature;
  //Temperature Backoff State is a blanket reduction of 6 degrees
  //  but we want to ensure we don't manually drop below 72
  if(TempBackOffState){
    //If we're above 72, we can drop to as little as 72
    if(atemp > 72){
      atemp = max(72,atemp-6);
    }
  }

  if(temperature == 0){
    //error, make no decisions
    return;
  }
  if((heater_on == 0 && temperature < (atemp-temperature_allowance)) ||
     (heater_on == 1 && temperature < atemp)
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

void humidifierLogic(){
  if(humidity == 0){
    //error, make no decisions
    return;
  }
  if((humidifier_on == 0 && humidity < (desired_humidity-humidity_allowance)) ||
     (humidifier_on == 1 && humidity < (desired_humidity+humidity_allowance))
   ){
     digitalWrite(HUMIDIFIER_RELAY_PIN, HIGH);
     humidifier_on = 1;
     #ifdef SERIALCOM
       Serial.println(F("Turning Humdifier On"));
     #endif
   }else{
     digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);
     humidifier_on = 0;
     #ifdef SERIALCOM
       Serial.println(F("Turning Humidifier Off"));
     #endif
   }
}

void set_color_intensity(byte color, float intensity){
  #ifdef SERIALCOM
    Serial.print(F("Setting color "));
    Serial.print(color);
    Serial.print(F(" to "));
    Serial.println(floor(intensity+0.5));
  #endif
  if(color == (int)(analogRead(COLOR_SELECT_INPUT) / COLOR_DIVISIONS)){
    lcd.setCursor(0,0);
    lcd.print(colors[color]);
    lcd.print(F("    "));
    lcd.print((byte)floor(intensity+0.5f));
    lcd.print(F("   "));
  }
  analogWrite(lightPins[color], (byte)floor(intensity+0.5f));
  colors[color] = intensity;
}

void set_color_gradient(byte red, byte green, byte blue, unsigned long transition_time){
  float gradients[3];
  signed long seconds;
  signed long secondMod;
  byte i;
  byte lowest;

  seconds = max(transition_time/1000,1);
  lowest = 10;
  secondMod = 1;

  gradients[0] = ((float)red - colors[0]) / seconds;
  gradients[1] = ((float)green - colors[1]) / seconds;
  gradients[2] = ((float)blue - colors[2]) / seconds;
  //increase the secondMod until at least one color is incrementing by a whole number
  for(i=0;i<3;i++){
    if(abs(gradients[i])>0.00f && (lowest == 10 || abs(gradients[i]) < abs(gradients[lowest]))){
      lowest = i;
    }
  }
  while(abs(gradients[lowest])>10 && fabs(gradients[lowest])*secondMod < 0.001){
    secondMod++;
  }
  //Stop any existing gradient timer
  t.stop(color_gradient_timer);
  if(abs(gradients[lowest]) > 0 && seconds > 0){
    gradient[0] = gradients[0]*secondMod;
    gradient[1] = gradients[1]*secondMod;
    gradient[2] = gradients[2]*secondMod;
    color_gradient_timer = t.every(1000*secondMod,color_gradient,floor(seconds/secondMod));
    #ifdef SERIALCOM
      Serial.println(F("Setting a Color Gradient"));
      Serial.print(F("From "));
      Serial.print(colors[0]);
      Serial.print(F(" "));
      Serial.print(colors[1]);
      Serial.print(F(" "));
      Serial.println(colors[2]);
      Serial.print(F("At Rate "));
      Serial.print(gradient[0]);
      Serial.print(F(" "));
      Serial.print(gradient[1]);
      Serial.print(F(" "));
      Serial.println(gradient[2]);
      Serial.print(F("Every "));
      Serial.print(1000*secondMod);
      Serial.print(F("ms, "));
      Serial.print(floor(seconds/secondMod));
      Serial.println(F(" times"));
    #endif
  }
}

void color_gradient(){
  for(int i = 0; i < 3; i++){
    set_color_intensity(i,colors[i] + gradient[i]);
  }
  #ifdef SERIALCOM
    Serial.print(F("Gradient changed to "));
    Serial.print(colors[0]);
    Serial.print(F(" "));
    Serial.print(colors[1]);
    Serial.print(F(" "));
    Serial.println(colors[2]);
  #endif
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
    HTTP_req = "";
    int connectLoop = 0;
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    char buffer[60] = {};
    while (client.connected()) {
      connectLoop++;
      if (client.available()) {
        char c = client.read();
        connectLoop = 0;
        HTTP_req += c;

        if(HTTP_req.length() > 800){
          client.stop();
          return;
        }

        if(connectLoop > 1000){
          client.stop();
          return;
        }

        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: application/json"));
          client.println(F("Connection: close"));  // the connection will be closed after completion of the response
          client.println();

          if(HTTP_req.indexOf("/status") == 4){
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
            client.print(F(",\"target_humidity\":"));
            client.print(desired_humidity);
            client.print(F(",\"humidity_variance\":"));
            client.print(humidity_allowance);
            strcpy_P(buffer,genericAjaxClose);
            client.print(buffer);
          }

          if(HTTP_req.indexOf("/set_color/") == 4){
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

          if(HTTP_req.indexOf("/set_gradient/") ==  4){
            sub = HTTP_req.substring(18,100);
            sub = sub.substring(0,sub.indexOf("/"));
            byte red = sub.toInt();
            sub2 = HTTP_req.substring(18+sub.length()+1,100);
            sub2 = sub2.substring(0,sub2.indexOf("/"));
            byte green = sub2.toInt();
            sub3 = HTTP_req.substring(18+sub.length()+sub2.length()+2,100);
            sub3 = sub2.substring(0,sub3.indexOf("/"));
            byte blue = sub3.toInt();
            sub3 = HTTP_req.substring(18+sub.length()+sub2.length()+sub3.length()+3,100);
            unsigned long time = sub3.toInt();

            #ifdef SERIALCOM
              Serial.println(F("Setting Gradient: "));
              Serial.println(red);
              Serial.println(green);
              Serial.println(blue);
              Serial.print(F("over "));
              Serial.print(time);
              Serial.println(F("ms"));
            #endif

            set_color_gradient(red,green,blue,time);

            strcpy_P(buffer,genericAjaxSuccess);
            client.print(buffer);
            strcpy_P(buffer,genericAjaxClose);
            client.print(buffer);
          }

          if(HTTP_req.indexOf("/set_thermostat/") == 4){
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
  unsigned int connectLoop = 0;
  char c;
  AjaxMessage = "";
  if(serverajax.connect(WEBSITE,80)){
    serverajax.print(F("GET "));
    serverajax.print(WEBSITE_PROXY);
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
        AjaxMessage += c;
      }
      if(AjaxMessage.length() > 800){
        serverajax.stop();
        return (char*)F("{\"result\":false,\"message\":\"AJAX Too Long\"}");
      }
      if(connectLoop > 10000){
        serverajax.stop();
        return (char*)F("{\"result\":false,\"message\":\"AJAX Timeout\"}");
      }
    }
    submessage = AjaxMessage.substring(AjaxMessage.indexOf("{"),AjaxMessage.length())+"}";
    char d2[AjaxMessage.length()];
    submessage.toCharArray(d2,submessage.length());
    serverajax.stop();
    if(submessage.length() == 1){
      return (char*)F("{\"result\":false,\"message\":\"Arduino Ajax Abandoned Connection before response\"}");
    }
    return d2;
  }
  serverajax.stop();
  return (char*)F("{\"result\":false,\"message\":\"Arduino Ajax Connection Error\"}");
}

void ServerTime(){
  t.stop(ServerTimeTimer);
  lcd.setCursor(0,3);
  lcd.print("Calling Ajax");
  char* ajax = Ajax("terrarium/get_time");
  lcd.setCursor(0,3);
    lcd.print("Returned");
  boolean result;
  unsigned long var;
  hashTable = parser.parseHashTable(ajax);
  if(!hashTable.success()){
    ServerTimeTimer = t.after(1000,ServerTime);
    lcd.setCursor(0,3);
    lcd.print("HTF: ");
    lcd.print(ajax);
    return;
  }
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);

  result = hashTable.getBool("result");
  if(result == true){
    unsigned long time = hashTable.getLong("now");
    lcd.print(time);
    lcd.print(F("    "));
    //lcd.print(memoryTest());
    lcd.print(F("K"));
    var = hashTable.getLong("temp");
    desired_temperature = var;
    var = hashTable.getLong("var");
    temperature_allowance = var;
    heaterLogic();
    //Set colors
    var = hashTable.getLong("red");
    set_color_intensity(0,var);
    var = hashTable.getLong("green");
    set_color_intensity(1,var);
    var = hashTable.getLong("blue");
    set_color_intensity(2,var);
    var = hashTable.getLong("gradient");
    if(var == 1){
      unsigned long red = hashTable.getLong("gRed");
      unsigned long green = hashTable.getLong("gGreen");
      unsigned long blue = hashTable.getLong("gBlue");
      var = hashTable.getLong("gTime");
      set_color_gradient(red,green,blue,var);
    }
    //Let the server tell us when to check back
    var = hashTable.getLong("cbt");
  }else{
    char* error = hashTable.getString("message");
    lcd.print(error);
    //Re-call in 20 seconds
    var = 20000;
  }
  ServerTimeTimer = t.after(var,ServerTime);
}

void temp_backoff_restore(){
  //Remoe the backoff flag
  TempBackOffState = 0;
  //Blink the lights to show anyone inside that the backoff is over
  set_rgb(20,20,20);
  delay(500);
  set_rgb(200,200,200);
  delay(500);
  set_rgb(20,20,20);
  delay(500);
  set_rgb(200,200,200);
  delay(500);
  set_rgb(20,20,20);
  delay(500);
  set_rgb(200,200,200);
  delay(500);
  //Restore the server info (lighting)
  ServerTime();
  //update our Heater Logic
  heaterLogic();
}

int temp_backoff_check(){
  boolean changed = TempBackoffBounce.update();
  if(changed && TempBackoffBounce.read() == 1){
    TempBackOffState = 1;
    heaterLogic();
    t.stop(TempBackOffStateTimer);
    TempBackOffStateTimer = t.after(3600000,temp_backoff_restore);
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
  temp_backoff_check();

  //Receive http request
  httpServer();
}

