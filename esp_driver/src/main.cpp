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
#ifdef NEOPIXEL_ENABLED
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  }
  else
  {
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  }
  pixels.show();
#endif
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

struct InitSerial
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

ServiceAnnouncement announcement;

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

void setup_esp_now()
{
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
}

// --- Serial State Machine Definitions ---
// We now have three states:
//   WAIT_FOR_HEADER: Look for the "STAR" header
//   READ_MSG_TYPE: Read one byte for the message type
//   READ_PAYLOAD: Read the payload based on the message type
enum SerialState
{
  WAIT_FOR_HEADER,
  READ_MSG_TYPE,
  READ_PAYLOAD
};

SerialState serialState = WAIT_FOR_HEADER;
bool serialInitialized = false;
const unsigned long TIMEOUT_MS =
    100;  // Timeout for the state machine in milliseconds
unsigned long lastByteTime = 0;

// Global watchdog timeout for the serial link (e.g., 5000 ms)
const unsigned long DISCONNECT_TIMEOUT = 1000;
unsigned long lastMessageTime = 0;

const int HEADER_SIZE = 4;
char headerBuffer[HEADER_SIZE];
int headerIndex = 0;

uint8_t messageType = 0;
uint16_t payloadLength = 0;
const int MAX_PAYLOAD_SIZE =
    sizeof(JoystickData) + 10;  // Sufficient for our payload
uint8_t payloadBuffer[MAX_PAYLOAD_SIZE];
int payloadIndex = 0;

// --- Process Serial Data ---

void processSerial()
{
  // Process all available bytes from Serial
  while (Serial.available() > 0)
  {
    char inByte = Serial.read();
    lastByteTime = millis();
    lastMessageTime = millis();  // Update global watchdog timer

    switch (serialState)
    {
      case WAIT_FOR_HEADER:
        headerBuffer[headerIndex++] = inByte;
        if (headerIndex == HEADER_SIZE)
        {
          if (memcmp(headerBuffer, "STAR", HEADER_SIZE) == 0)
          {
            // Header found; move to read message type state
            serialState = READ_MSG_TYPE;
          }
          else
          {
            // Shift the header buffer left by one and keep searching
            memmove(headerBuffer, headerBuffer + 1, HEADER_SIZE - 1);
            headerIndex = HEADER_SIZE - 1;
          }
        }
        break;

      case READ_MSG_TYPE:
        // The next byte is the message type.
        messageType = inByte;
        // Decide expected payload length based on the message type.
        if (messageType == 0x01)
        {  // Joystick message
          payloadLength = sizeof(JoystickData);
        }
        else if (messageType == 0x02)
        {  // Init message
          payloadLength = sizeof(InitSerial);
        }
        else
        {
          serialState = WAIT_FOR_HEADER;
          headerIndex = 0;
          break;
        }
        serialState = READ_PAYLOAD;
        payloadIndex = 0;
        break;

      case READ_PAYLOAD:
        payloadBuffer[payloadIndex++] = inByte;
        if (payloadIndex >= payloadLength)
        {
          // Process the payload based on the message type.
          if (messageType == 0x01)
          {
            memcpy(&joystick, payloadBuffer, payloadLength);
            for (int i = 0; i < clientCount; i++)
            {
              esp_now_send(registeredClients[i], (uint8_t *)&joystick,
                           sizeof(joystick));
            }
          }
          else if (messageType == 0x02)
          {
            if (!serialInitialized)
            {
              serialInitialized = true;

              InitSerial init;
              memcpy(&init, payloadBuffer, payloadLength);

              //   Write the init message back to the serial port to finish
              //   handshake
              Serial.write((uint8_t *)&init, sizeof(init));

              //   Set announcement meessage to be init.service_name
              memcpy(announcement.service_name, init.service_name,
                     sizeof(announcement.service_name));

              setup_esp_now();
            }
            else
            {
              // We received an initialization message, when we didn't expect
              // one. Reboot the ESP to resync
              pixels.setPixelColor(0, pixels.Color(0, 255, 255));
              pixels.setBrightness(255);
              pixels.show();

              delay(50);

              ESP.restart();
            }
          }
          // Reset the state machine for the next packet.
          serialState = WAIT_FOR_HEADER;
          headerIndex = 0;
        }
        break;
    }
  }

  // Timeout handling for the serial state machine.
  if ((millis() - lastByteTime) > TIMEOUT_MS)
  {
    serialState = WAIT_FOR_HEADER;
    headerIndex = 0;
    payloadIndex = 0;
  }
}

void setup()
{
  delay(100);
  Serial.begin(115200);

#ifdef NEOPIXEL_ENABLED

  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.setBrightness(255);
  pixels.show();

#endif

  Serial.println("Setup complete");
}

unsigned long lastBroadcastTime = 0;
void loop()
{
  // Process incoming serial data using our state machine
  processSerial();

  if (serialInitialized)
  {
    // Global watchdog: if no valid message is received for DISCONNECT_TIMEOUT,
    // reboot the ESP32.
    if ((millis() - lastMessageTime) > DISCONNECT_TIMEOUT)
    {
      Serial.println("No message received for a while. Rebooting ESP32...");
      delay(100);

      pixels.setPixelColor(0, pixels.Color(0, 255, 255));
      pixels.setBrightness(255);
      pixels.show();

      delay(1000);
      ESP.restart();
    }

    if (millis() - lastBroadcastTime >= 3000)
    {
      esp_now_send((uint8_t *)"\xFF\xFF\xFF\xFF\xFF\xFF",
                   (uint8_t *)&announcement, sizeof(announcement));
      lastBroadcastTime = millis();
    }
  }
}