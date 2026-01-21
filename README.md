# 📡 ESP32 ESP-NOW Walkie-Talkie (Learning Prototype)

A DIY **half-duplex walkie-talkie** built using two ESP32 boards, **ESP-NOW** for wireless communication, and **I2S audio** (digital microphone + amplifier).

This project was developed as a **learning prototype** to explore real-time audio streaming, ESP-NOW limitations, I2S audio handling, and embedded system integration.

---

## ✨ Project Overview

This project implements a **Push-To-Talk (PTT) walkie-talkie** using:

- **ESP-NOW** for low-latency peer-to-peer communication  
- **I2S digital microphone** for audio capture  
- **I2S audio amplifier** for playback  
- **IMA-ADPCM (4-bit)** compression to fit audio into ESP-NOW packet limits  

Both devices run the **same firmware**.  
When the PTT button is pressed, the device transmits audio; when released, it switches to receive mode.

---

## 🧠 Key Features

- Half-duplex walkie-talkie (Push-To-Talk)
- Real-time audio streaming over ESP-NOW
- Single firmware for both devices
- I2S RX + TX on the same ESP32 peripheral
- LED status indicators (Power / TX / Link)
- Fully standalone (no Wi-Fi router or access point required)

---

## 🧩 Hardware Used

- ESP32 DevKit / NodeMCU ESP32  
- INMP441 I2S digital microphone  
- MAX98357A I2S audio amplifier  
- 8Ω 0.5W ~1.4″ speaker  
- Push-To-Talk button  
- Status LEDs  
- 2× 18650 Li-ion batteries  
- Mini360 buck converter (5V supply)  
- 3D-printed enclosure (prototype)

---

## 🔌 Pin Connections

### I2S Audio

| Signal | ESP32 GPIO | Notes |
|------|-----------|------|
| I2S BCLK | GPIO 26 | Shared between mic & amp |
| I2S LRCLK | GPIO 25 | Shared between mic & amp |
| Mic Data (SD) | GPIO 34 | Input-only pin |
| Speaker Data (DIN) | GPIO 22 | Output to MAX98357A |

### Controls & Indicators

| Component | ESP32 GPIO |
|---------|-----------|
| Push-To-Talk Button | GPIO 5 |
| Power LED | GPIO 2 |
| TX (Unmute) LED | GPIO 14 |
| Link / Activity LED | GPIO 4 |

---

## 🔊 Audio Pipeline

INMP441 Microphone
        ↓ (I2S RX)
ESP32
        ↓ 16-bit PCM @ 16 kHz
IMA-ADPCM Encoding (4-bit)
        ↓
ESP-NOW Wireless Packet
        ↓
ESP32 (Peer Device)
        ↓ ADPCM Decode
PCM Audio
        ↓ (I2S TX)
MAX98357A Amplifier
        ↓
Speaker
---

### 📶 Performance & Limitations

- Indoor range: ~10–15 meters (concrete building)

- Wireless protocol: ESP-NOW (2.4 GHz)

- Mode: Half-duplex only (no simultaneous talk & listen)

- Audio quality: Clear speech, bandwidth-limited (expected with ADPCM)

- Enclosure: Early prototype (battery compartment missing)

This project focuses on learning and experimentation, not long-range RF performance.

---

### 🧪 Current Status

✅ Audio transmission and reception working

✅ Push-To-Talk logic stable

✅ Wireless communication verified

❌ Limited indoor range

❌ Enclosure design needs improvement

---

### 🚀 Possible Improvements

External antenna or ESP32 module with u.FL connector

Jitter buffer and packet-loss handling

Automatic Gain Control (AGC)

Improved enclosure acoustics

Better power management

Custom PCB revision

Higher-quality codec (e.g., Opus)

---

### 📁 Repository Structure
.
├── firmware
│   └── esp_1
│       └── esp_1.ino
├── 3d-model
│   └── enclosure_prototype.stl
└── README.md
