#include <LiquidCrystal_I2C.h>                  // LCD Display Library
#include <Wire.h>                               // I2C Library
#include <Adafruit_MLX90614.h>                  // Temperature sensor Library
#include <ESP8266WiFi.h>                        // Wifi Library
#include <FirebaseArduino.h>                    // Firebase Library
#include <SoftwareSerial.h>                     // Serial Library
#include <ArduinoJson.h>                        // Json Library
#include <NTPClient.h>                          // Date and Time Library
#include <WiFiUdp.h>


#define LF          0x0A                                                // "Enter" character
#define FIREBASE_HOST "pervasive-csg541.firebaseio.com"
#define FIREBASE_AUTH "5Z9S5z0FyamJkNuVQskd3dhKXl7su1qLIvbQlqrI"        // API Key
#define WIFI_SSID "PATIL_2G"                                            // Hotspot id 
#define WIFI_PASSWORD "aatspatil@1980"                                  // Hotspot password


int LED1 = 15; // Pin D8          // Green LED
int LED2 = 13; // Pin D7          // Red LED


LiquidCrystal_I2C lcd(0x27, 16, 2);                     // 0x27 - address of LCD module
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
SoftwareSerial s(D6,D5);                                // D6 - Rx pin; D5 - Tx pin
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

char Name[10];                    // Patients Name
int i;
float tempF, oxy;                 // Temp variable and Oxy variable
int n = 1;
String condition = "";            // condition variable


void setup() {
    Serial.begin(115200);
    s.begin(115200);
    lcd.begin(16,2);
    lcd.init();
    lcd.backlight();
    i = 0;
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);                     
    Serial.print("Connecting to ");
    Serial.print(WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }
    Serial.println();
    Serial.print("Connected to ");
    Serial.println(WIFI_SSID);
    Serial.print("IP Address is : ");
    Serial.println(WiFi.localIP());
    timeClient.begin();
    timeClient.setTimeOffset(19800);                       // 19800 - GMT for India
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    mlx.begin();                                           // Temperature sensor starts
}

void loop() {

    digitalWrite(LED1, LOW);                  // turn the LED off
    digitalWrite(LED2, LOW);                  // turn the LED off
    if (Serial.available() > 0) {
    Name[i] = Serial.read();
    if (Name[i] == LF) {                            // If "Enter" charater encountered
        Name[i] = 0;
        
        timeClient.update();                                      // Get Date and Time
        unsigned long epochTime = timeClient.getEpochTime();
        String Time = timeClient.getFormattedTime();
        struct tm *ptm = gmtime ((time_t *)&epochTime);
        int monthDay = ptm->tm_mday;
        int currentMonth = ptm->tm_mon+1;
        int currentYear = ptm->tm_year+1900;
        String Date = String(monthDay) + "-" + String(currentMonth) + "-" + String(currentYear); 

        delay(1000);
        lcd.setCursor(0, 0);
        lcd.print("Patient Name:");
        lcd.setCursor(6, 1);      
        lcd.print(Name);
        delay(4000);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Place Your Hand");
        lcd.setCursor(2, 1);
        lcd.print("on the sensor");
        delay(3000);
        lcd.clear();
        lcd.setCursor(4, 0);
        lcd.print("Fetching");
        lcd.setCursor(2, 1);      
        lcd.print("Temperature");
        delay(2000);
        
        tempF = mlx.readObjectTempC();            // Get Body Temperature

        delay(1000);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Body Temperature");
        lcd.setCursor(5, 1);      
        lcd.print(tempF);
        lcd.print((char)223);
        lcd.print("C");
        delay(4000);
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Fetching Blood");
        lcd.setCursor(2, 1);      
        lcd.print("Oxygen level");
        delay(4000);
        
        s.write("s");                                         // Send character to arduino
        StaticJsonBuffer<100> jsonBuffer;                     // Json buffer to hold sensor values
        JsonObject& root = jsonBuffer.parseObject(s);
        if (root == JsonObject::invalid())
          return;  
        heart = root["HR"];                                   // read heartrate sent from arduino
        oxy = root["PO"];                                     // read blood oxygen level sent from arduino
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Oxygen level: ");
        lcd.setCursor(7, 1);      
        lcd.print(oxy);
        lcd.print("%");
        delay(4000);
        
        lcd.clear();
        lcd.setCursor(4,0);
        lcd.print("Analysing");
        lcd.setCursor(1,1);
        lcd.print("Patient's data");
        delay(3000);

        if(tempF > 40 || oxy < 90){                 // condition for critical status
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Health Status:");
            lcd.setCursor(4,1);
            lcd.print("CRITICAL");
            condition = "CRITICAL";
            for(int y=0; y<6; y++){                               // Green LED blinks
                digitalWrite(LED1, HIGH); // turn the LED on
                delay(500);
                digitalWrite(LED1, LOW); // turn the LED off
                delay(500);
            }
            
            delay(1000);
        }else{                                      // condition for normal status
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Health Status:");
            lcd.setCursor(4,1);
            lcd.print("NORMAL");
            condition = "NORMAL";
            for(int y=0; y<6; y++){                               // Red LED blinks
                digitalWrite(LED2, HIGH); // turn the LED on
                delay(500);
                digitalWrite(LED2, LOW); // turn the LED off
                delay(500);
            }
            
            delay(1000);
        }
        lcd.clear();
        lcd.setCursor(4,0);
        lcd.print("Uploading");
        lcd.setCursor(1,1);
        lcd.print("Patient's data");
        delay(1000);
        
        Firebase.setString("Date/Patient1/Body_Temperature", String(tempF));        // Uploading Data onto cloud
        Firebase.setString("Date/Patient1/Oxygen_Level", String(oxy));
        Firebase.setString("Date/Patient1/Name", Name);
        Firebase.setString("Date/Patient1/Health_Condition", condition); 
        
        i = -1;
        lcd.clear();
        }
    i++;
    }
}
