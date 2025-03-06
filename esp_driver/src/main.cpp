#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "Adafruit_NeoPixel.h"

#define LED_PIN 38
#define NUM_LEDS 1

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Replace with your receiver ESP32's MAC address
uint8_t receiverMacAddress[] = {0xD8, 0x3B, 0xDA, 0x43, 0x0A, 0x3C};

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  }
  else
  {
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  }
  pixels.show();
}

struct Joystick
{
  float left;
  float right;
};

Joystick joystick;

void setup()
{
  Serial.begin(115200);

  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.setBrightness(255);
  pixels.show();

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  delay(1000);
}

void loop()
{
  // Wait until joystick data is available on Serial
  if (Serial.available() >= sizeof(joystick))
  {
    Serial.readBytes((char *)&joystick, sizeof(joystick));

    // Send data via ESP-NOW
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&joystick,
                                    sizeof(joystick));

    if (result != ESP_OK)
    {
      Serial.println("Error sending the data");
    }
  }
}