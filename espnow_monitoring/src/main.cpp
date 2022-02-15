#include <Wire.h>
#include <WiFi.h>
#include "ESPNowW.h"

const int analogepin = 36;
const byte led_gpio_green = 14;
const byte led_gpio_yellow = 32;
const byte led_gpio_red = 15;

typedef struct struct_message {
  int analogevalue;
} struct_message;

struct_message myData;

uint8_t receiver_mac[] = {0xE0, 0xE2, 0xE6, 0x76, 0x43, 0xE4}; 

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  pinMode(led_gpio_green, OUTPUT);
  pinMode(led_gpio_yellow, OUTPUT);
  pinMode(led_gpio_red, OUTPUT);

    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    ESPNow.init();
    ESPNow.add_peer(receiver_mac);

}

void loop()
{
    delay(1);
    myData.analogevalue = analogRead(analogepin);
    Serial.print(myData.analogevalue);

    if (myData.analogevalue> 3100)
    {
      digitalWrite(led_gpio_green, HIGH); 
      digitalWrite(led_gpio_yellow, LOW);
      digitalWrite(led_gpio_red, LOW);
    }
  else if (myData.analogevalue > 1900 and myData.analogevalue < 3100) {
      digitalWrite(led_gpio_green, LOW); 
      digitalWrite(led_gpio_yellow, HIGH);
      digitalWrite(led_gpio_red, LOW);
  }
  else {
      digitalWrite(led_gpio_green, LOW); 
      digitalWrite(led_gpio_yellow, LOW);
      digitalWrite(led_gpio_red, HIGH);
  }

  ESPNow.send_message(receiver_mac, (uint8_t *) &myData, sizeof (myData));
  Serial.println();
}