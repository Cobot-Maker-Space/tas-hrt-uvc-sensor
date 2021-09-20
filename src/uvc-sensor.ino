#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <DFRobot_LCD.h>
#include <WiFi.h>
#include <Wire.h>

const char* AP_SSID = "Cobot Maker Space 2.4Ghz";
const char* AP_PASSWORD = "r4HJbzjtlKOUtQcnENbU";
const char* MQTT_TOPIC_BASE = "tas/hrt/uvc";

const uint32_t INTERVAL_MILLIS = 100;

const uint8_t LCD_BRIGHTNESS = 255;

const double UVC_POWER_CONSTANT = 0.355;
const uint8_t UVC_SENSOR_ADDRESS = 0X4D;
const uint8_t UVC_SENSOR_RESOLUTION = 12;
const double UVC_SENSOR_VOLTAGE = 3.3;

String mac_address;
DFRobot_LCD lcd(16, 2);
char* mqtt_topic;

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER_ADDR, MQTT_PORT);
Adafruit_MQTT_Publish* queue;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.begin(AP_SSID, AP_PASSWORD);
  while (!WiFi.isConnected()) {
    Serial.println("Waiting for WiFi connection");
    delay(1000);
  }
  mac_address = WiFi.macAddress();
  mqtt_topic = new char[strlen(MQTT_TOPIC_BASE) + 19];
  sprintf(mqtt_topic, "%s/%s", MQTT_TOPIC_BASE, mac_address.c_str());
  queue = new Adafruit_MQTT_Publish(&mqtt, mqtt_topic);
  lcd.init();
  lcd.print(F("UVc: 0.00 mW/cm2"));
  lcd.setCursor(4, 1);
  lcd.print(mac_address.substring(9, 17));
  lcd.setPWM(REG_ONLY, LCD_BRIGHTNESS);
}

void loop() {
  mqtt_connect();
  mqtt.handleSubscriptionPacket(1);
  static uint32_t next_run_time = millis();
  unsigned long now = millis();
  if (now >= next_run_time) {
    next_run_time += INTERVAL_MILLIS;
    double power = adc_to_power(get_uv_adc());
    lcd.setCursor(5, 0);
    lcd.printf("%.2f", power);
    queue->publish(power, 5);
  }
}

void mqtt_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.println("Connecting to MQTT...");
  while ((ret = mqtt.connect()) != 0) {
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);
  }
  Serial.println("MQTT Connected!");
}

inline double adc_to_power(uint32_t adc) {
  // Power units: mW/cm2
  // Range:
  // 3.3V : 0 - 9.3
  // 5V   : 0 - 14.1
  return (UVC_SENSOR_VOLTAGE *
             (adc / pow(2, UVC_SENSOR_RESOLUTION))) /
         UVC_POWER_CONSTANT;
}

uint16_t get_uv_adc() {
  uint16_t result = 0;
  Wire.requestFrom(UVC_SENSOR_ADDRESS, 2u);
  if (Wire.available()) {
    result = Wire.read();
    result = result << 8;
    result += Wire.read();
  }
  return result;
}
