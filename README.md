<h1 align="center">🎮 ESP-NOW JoyBridge 🎮</h1>

<p align="center">
  Wirelessly transmit joystick commands from your computer to ESP32-based projects using ESP-NOW.
</p>

---
<h1 align="center">🎮 ESP-NOW JoyBridge 🎮</h1>

<p align="center">
  Wirelessly transmit joystick commands from your computer to ESP32-based projects using ESP-NOW.
</p>

---
<h1 align="center">🎮 ESP-NOW JoyBridge 🎮</h1>

<p align="center">
  <strong>Wirelessly transmit joystick commands from your computer to ESP32-based projects using ESP-NOW.</strong>
</p>

---

## 📖 Table of Contents

- [📖 Table of Contents](#-table-of-contents)
- [🎯 Project Objectives](#-project-objectives)
- [✨ Key Features](#-key-features)
- [🛠️ Project Components](#️-project-components)
  - [ESP32 JoyBridge Firmware](#esp32-joybridge-firmware)
  - [JoyBridge Host Client](#joybridge-host-client)
  - [JoyBridge Receiver Library](#joybridge-receiver-library)
- [📦 Installation](#-installation)
  - [Installation via `efr` Tool](#installation-via-efr-tool)
  - [Manual Installation](#manual-installation)
    - [Flashing Firmware](#flashing-firmware)
    - [Running Host Client](#running-host-client)
  - [Setup JoyBridge Receiver Library](#setup-joybridge-receiver-library)
  - [Example Usage](#example-usage)

---

## 🎯 Project Objectives

JoyBridge provides a simple, efficient, and reliable way to wirelessly send joystick commands to ESP32-based projects, such as MotorGo or other compatible platforms. It emphasizes ease of use, real-time responsiveness, and minimal resource usage.

## ✨ Key Features

- **🔧 Minimal Setup:** Easily integrates with virtually any joystick.
- **⚡ Low Latency:** Ultra-responsive joystick controls ideal for real-time applications.
- **🌿 Lightweight:** Low CPU and memory footprint, perfect for resource-constrained environments.

## 🛠️ Project Components

JoyBridge consists of three main components:

### ESP32 JoyBridge Firmware
- Receives joystick commands from a computer via serial.
- Broadcasts joystick data to connected ESP32 devices using ESP-NOW.
- Supports up to 10 simultaneous device connections (configurable in firmware).
- Optimized for the [WaveShare ESP32-S3 1.47-inch Display Development Board](https://www.waveshare.com/esp32-s3-lcd-1.47.htm), but compatible with any ESP32-S3 device (LED indicators may differ).

### JoyBridge Host Client
- Python application using `pygame` to capture joystick input.
- Sends joystick data to the ESP32 bridge via serial.
- The `--service-name` option specifies the unique ESP-NOW service name. Connected devices use this name to discover and join the JoyBridge data stream. Customizing this helps avoid conflicts when running multiple bridges simultaneously. Default service name: `JoystickService`.

### JoyBridge Receiver Library
- Arduino library for easily integrating joystick command reception into your ESP32 projects.
- Automatically handles service discovery, subscription, and data management.

## 📦 Installation

JoyBridge can be installed either via the `efr` CLI tool or manually.

### Installation via `efr` Tool

Install JoyBridge quickly using the `efr` CLI tool:

```bash
efr plugins install joybridge
```

Flash firmware to your ESP32:

```bash
efr joybridge flash
```

Run the JoyBridge Host Client (optionally specifying your service name):

```bash
efr joybridge go --service-name MyJoystickService
```

Then proceed to [Setup JoyBridge Receiver Library](#setup-joybridge-receiver-library).

### Manual Installation

Clone the JoyBridge repository:

```bash
git clone https://github.com/Every-Flavor-Robotics/esp-now-joybridge.git
```

The repository includes:

- `joybridge_firmware`: Firmware for the ESP32 JoyBridge.
- `joybridge_host`: Python host client.

#### Flashing Firmware

The `joybridge_host` is a `uv` project. Flash firmware:

```bash
cd esp-now-joybridge/joybridge_host
uv run main.py flash
```

Alternatively, manually flash using PlatformIO in the `joybridge_firmware` directory.

#### Running Host Client

Run the JoyBridge Host Client, optionally specifying a custom service name:

```bash
uv run main.py --service-name MyJoystickService
```

Then proceed to [Setup JoyBridge Receiver Library](#setup-joybridge-receiver-library).

### Setup JoyBridge Receiver Library

Add the JoyBridge Receiver Library to your ESP32 project by updating your `platformio.ini`:

```ini
lib_deps =
    https://github.com/Every-Flavor-Robotics/esp-now-joybridge-receiver.git
```

### Example Usage

Here's a minimal example demonstrating the JoyBridge Receiver usage:

```cpp
#include <Arduino.h>
#include <joybridge_receiver.h>

JoyBridge::JoyBridgeReceiver receiver;

void setup()
{
  Serial.begin(115200);

  if (!receiver.begin("MyJoystickService"))
  {
    Serial.println("Receiver initialization failed!");
    while (true)
      delay(1000);
  }
}

void loop()
{
  receiver.loop();

  if (receiver.isConnected())
  {
    JoyBridge::JoystickData data = receiver.getJoystickData();
    Serial.println("Joystick Data Received:");
    Serial.printf(
        "A:%d B:%d X:%d Y:%d LB:%d RB:%d DU:%d DD:%d DL:%d DR:%d START:%d BACK:%d\n",
        data.a, data.b, data.x, data.y, data.lb, data.rb, data.dpad_up,
        data.dpad_down, data.dpad_left, data.dpad_right, data.start, data.back);
    Serial.printf("LT:%.2f RT:%.2f LX:%.2f LY:%.2f RX:%.2f RY:%.2f\n",
                  data.lt, data.rt, data.left_x, data.left_y, data.right_x, data.right_y);
  }
  else
  {
    Serial.println("Master not connected.");
  }

  delay(100);
}
```

