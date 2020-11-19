#include "MAX30100_PulseOximeter.h"         // Pulse Oximeter Library
#include <SoftwareSerial.h>                 // Serial Library
#include <ArduinoJson.h>                    // Json Library
 
PulseOximeter pox;
SoftwareSerial espSerial(5, 6);           // Pin 5 - Rx_Pin;    Pin 6 - Tx_Pin

StaticJsonBuffer<100> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
 
void setup()
{
    Serial.begin(115200);
    espSerial.begin(115200);
    Serial.print("Initializing pulse oximeter..");
    
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }
}
 
void loop()
{
    pox.update();                     // Update Sensor values
   
        root["HR"] = pox.getHeartRate();
        root["PO"] = pox.getSpO2();

        float h = pox.getHeartRate();
        float p = pox.getSpO2();
        
        Serial.print("Heart rate:");
        Serial.print(h);
        Serial.print("bpm / SpO2:");
        Serial.print(p);
        Serial.println("%");
        if(espSerial.available()>0)       // wait for character from ESP
        {
            root.printTo(espSerial);      // send sensor values to ESP
        }

}
