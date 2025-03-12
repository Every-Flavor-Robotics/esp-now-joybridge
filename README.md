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

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Project Objectives](#project-objectives)
- [Key Features](#key-features)
- [Project Components](#project-components)
  - [1. ESP32 JoyBridge Firmware](#1-esp32-joybridge-firmware)
  - [2. JoyBridge Host Client](#2-joybridge-host-client)
  - [3. JoyBridge Receiver Library](#3-joybridge-receiver-library)
- [Installation](#installation)
  - [Installation via `efr` Tool](#installation-via-efr-tool)
  - [Manual Installation](#manual-installation)
    - [Flashing Firmware](#flashing-firmware)
    - [Running Host Client](#running-host-client)
  - [Setup JoyBridge Receiver Library](#setup-joybridge-receiver-library)
  - [Example Usage](#example-usage)

---

## Project Objectives

JoyBridge provides an easy, efficient method to wirelessly send joystick commands to ESP32-based projects, such as MotorGo or other compatible platforms. It emphasizes simplicity, low latency, and efficient resource use.

## Key Features

- **Minimal Setup:** Easily integrates with virtually any joystick.
- **Low Latency:** Ensures quick, responsive joystick controls.
- **Lightweight:** Low CPU and memory footprint, ideal for low-resource systems.

## Project Components

JoyBridge consists of three main components:

### 1. ESP32 JoyBridge Firmware
- Receives joystick commands from a host computer over serial.
- Broadcasts commands to connected ESP32 devices using ESP-NOW.
- Supports up to 10 simultaneous device connections (configurable in firmware).
- Initially designed for the [WaveShare ESP32-S3 1.47-inch Display Development Board](https://www.waveshare.com/esp32-s3-lcd-1.47.htm), but works with any ESP32-S3 device. (LED indicators may differ.)

### 2. JoyBridge Host Client
- Python application utilizing `pygame` to capture joystick input.
- Sends joystick data to the ESP32 bridge via serial.
- The `--service-name` parameter identifies the ESP-NOW service advertised by the JoyBridge firmware. Receivers use this name to discover and connect to the joystick data stream. Using unique service names allows running multiple JoyBridge setups simultaneously without conflicts. The default service name is `JoystickService`.

### 3. JoyBridge Receiver Library
- Arduino library to easily integrate joystick commands into your ESP32 projects.
- Automatically manages ESP-NOW service discovery, subscription, and data handling.

## Installation

JoyBridge can be installed via the `efr` CLI tool or manually from the repository.

### Installation via `efr` Tool

If you have the `efr` CLI tool installed, run:

```bash
efr plugins install joybridge
```

Flash JoyBridge firmware to your ESP32:

```bash
efr joybridge flash
```

Run the JoyBridge Host Client with an optional custom service name:

```bash
efr joybridge go --service-name JoystickService
```

Then proceed to [Setup JoyBridge Receiver Library](#setup-joybridge-receiver-library).

### Manual Installation

Clone the JoyBridge repository:

```bash
git clone https://github.com/Every-Flavor-Robotics/esp-now-joybridge.git
```

This repository contains:

- `joybridge_firmware`: Firmware for the ESP32 bridge.
- `joybridge_host`: Python host client for sending joystick commands.

#### Flashing Firmware

The `joybridge_host` uses `uv`. Flash the firmware by running:

```bash
cd esp-now-joybridge/joybridge_host
uv run main.py flash
```

Alternatively, flash manually via PlatformIO from the `joybridge_firmware` directory.

#### Running Host Client

Start the JoyBridge Host Client, optionally specifying a custom service name:

```bash
uv run main.py --service-name JoystickService
```

Then proceed to [Setup JoyBridge Receiver Library](#setup-joybridge-receiver-library).

### Setup JoyBridge Receiver Library

Integrate JoyBridge into your ESP32 project by adding this library to your PlatformIO project (`platformio.ini`):

```ini
lib_deps =
    https://github.com/Every-Flavor-Robotics/esp-now-joybridge-receiver.git
```

### Example Usage

Below is a minimal example demonstrating usage of the JoyBridge Receiver Library:

```cpp
#include <Arduino.h>
#include <joybridge_receiver.h>

JoyBridge::JoyBridgeReceiver receiver;

void setup()
{
  Serial.begin(115200);

  if (!receiver.begin("JoystickService"))
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