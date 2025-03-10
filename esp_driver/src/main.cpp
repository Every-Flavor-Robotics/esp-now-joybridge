// This is the main driver for the ESP32 ESP-NOW bridge. It will receive
// joystick data from a computer, and send it to other ESP32s over ESP-NOW.

// This is written to be used with the WaveShare ESP32-S3, with a USB port:
// https://www.waveshare.com/product/mcu-tools/development-boards/esp32/esp32-s3-lcd-1.47.htm
// If you are using this board, the code will blink the NeoPixel on the board to
// indicate the status of the data transmission.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Maximum number of ESP-NOW clients
#define MAX_CLIENTS 10

// Comment out the following line if you are not using the WaveShare ESP32-S3
#define WAVESHARE_USB_ESP32

#ifdef WAVESHARE_USB_ESP32
#include "Adafruit_NeoPixel.h"

#define NEOPIXEL_ENABLED

#define LED_PIN 38
#define NUM_LEDS 1

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

// You can optionally define and setup the neopixel if your board has one
// Make sure to include  #define NEOPIXEL_ENABLED if you do define it.

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

// Update the struct to match the data above
struct JoystickData
{
  bool a;
  bool b;
  bool x;
  bool y;
  bool lb;
  bool rb;
  bool dpad_up;
  bool dpad_down;
  bool dpad_left;
  bool dpad_right;
  bool start;
  bool back;
  float lt;
  float rt;
  float left_x;
  float left_y;
  float right_x;
  float right_y;
};

// Struct for service discovery broadcast
struct ServiceAnnouncement
{
  char service_name[16];
};

// Struct for client registration
struct ClientRegistration
{
  char message[16];
};

JoystickData joystick;

// Store registered clients' MAC addresses
uint8_t registeredClients[MAX_CLIENTS][6];
int clientCount = 0;

ServiceAnnouncement announcement = {"JoystickService"};

// Callback for receiving messages (client registration)
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData,
                int len)
{
  ClientRegistration receivedMsg;
  memcpy(&receivedMsg, incomingData, sizeof(receivedMsg));

  if (strcmp(receivedMsg.message, "REGISTER") == 0)
  {
    Serial.printf("New client registered: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  recv_info->src_addr[0], recv_info->src_addr[1],
                  recv_info->src_addr[2], recv_info->src_addr[3],
                  recv_info->src_addr[4], recv_info->src_addr[5]);

    // Add to registered clients list if space available
    if (clientCount < MAX_CLIENTS)
    {
      memcpy(registeredClients[clientCount], recv_info->src_addr, 6);

      // Create a new peer
      esp_now_peer_info_t peerInfo;
      memset(&peerInfo, 0, sizeof(peerInfo));
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      memcpy(peerInfo.peer_addr, recv_info->src_addr, 6);

      if (esp_now_add_peer(&peerInfo) != ESP_OK)
      {
        Serial.println("Failed to add peer");
        return;
      }

      clientCount++;
    }
  }
}

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
  esp_now_register_recv_cb(OnDataRecv);

  //   Create a broadcast peer
  esp_now_peer_info_t broadcastPeer;
  memset(&broadcastPeer, 0, sizeof(broadcastPeer));
  broadcastPeer.channel = 0;
  broadcastPeer.encrypt = false;
  memcpy(broadcastPeer.peer_addr, "\xFF\xFF\xFF\xFF\xFF\xFF", 6);

  if (esp_now_add_peer(&broadcastPeer) != ESP_OK)
  {
    Serial.println("Failed to add broadcast peer");
    return;
  }

  delay(5000);
  Serial.println("Setup complete");
}

unsigned long lastBroadcastTime = 0;
void loop()
{
  // Wait until joystick data is available on Serial
  if (Serial.available() >= sizeof(joystick))
  {
    Serial.readBytes((char *)&joystick, sizeof(joystick));

    for (int i = 0; i < clientCount; i++)
    {
      esp_now_send(registeredClients[i], (uint8_t *)&joystick,
                   sizeof(joystick));
    }
  }

  if (millis() - lastBroadcastTime >= 2000)
  {
    esp_now_send((uint8_t *)"\xFF\xFF\xFF\xFF\xFF\xFF",
                 (uint8_t *)&announcement, sizeof(announcement));
    lastBroadcastTime = millis();
  }
}