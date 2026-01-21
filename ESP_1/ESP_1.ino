/*
  ESP-NOW Walkie-Talkie (Half-duplex, PTT)
  - Single firmware for both boards. Set PEER_ADDR to the other board's MAC.
  - I2S (RX+TX) single peripheral: mic on GPIO34, clocks on 26/25, speaker data on GPIO22.
  - PTT button on GPIO5 (pulls to GND when pressed).
  - LEDs: PWR = GPIO2, TX = GPIO4, LINK = GPIO15.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "driver/i2s.h"

// ----------------- USER CONFIG: set the peer MAC to the other board -----------------
uint8_t PEER_ADDR[6] = { 0x38,0x18,0x2B,0xB2,0x84,0x90 }; // <-- replace with peer MAC //og esp: 0xD0, 0xEF, 0x76, 0x32, 0x4C, 0xFC
// ------------------------------------------------------------------------------------

// Pins (match your PCB)
#define I2S_BCLK   26
#define I2S_LRCLK  25
#define I2S_MIC_SD 34
#define I2S_SPK_D  22

#define PTT_PIN    5
#define LED_PWR    2
#define LED_TX     14
#define LED_LINK   4

// Audio parameters
#define SAMPLE_RATE 16000
#define FRAME_MS    20
#define FRAME_SAMPLES ((SAMPLE_RATE * FRAME_MS) / 1000) // 320 samples
#define ADPCM_MAX_OUT_BYTES (3 + (FRAME_SAMPLES/2) + 4)

#define I2S_BUF_LEN 512
#define I2S_DMA_BUFS 6

// IMA ADPCM tables
const int indexTable[16] = { -1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8 };
const int stepTable[89] = {
  7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,107,118,
  130,143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,1060,
  1166,1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,
  7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,32767
};

// State vars
bool isTransmitting = false;
volatile bool pttPressed = false;
unsigned long lastLinkMs = 0;
const unsigned long LINK_LED_TIMEOUT = 2000; // ms to keep LINK LED on after packet

// Forward declarations
void setupI2S();
int ima_adpcm_encode_frame(const int16_t *inBuf, uint8_t *outBuf);
void ima_adpcm_decode_frame(const uint8_t *in, size_t in_len, int16_t *outSamples);

// -------------- ESP-NOW receive callback (runs in RTOS context) ----------------
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // If currently transmitting, ignore incoming audio (PTT prevents RX while TX)
  if (pttPressed) return;

  if (len <= 3) return; // invalid
  static int16_t pcmFrame[FRAME_SAMPLES];
  ima_adpcm_decode_frame(incomingData, (size_t)len, pcmFrame);

  // write PCM to I2S (blocking) — play the received audio
  size_t bytesWritten = 0;
  i2s_write(I2S_NUM_0, pcmFrame, FRAME_SAMPLES * sizeof(int16_t), &bytesWritten, portMAX_DELAY);

  // Link LED pulse
  digitalWrite(LED_LINK, HIGH);
  lastLinkMs = millis();
}

// -------------- ESP-NOW send status callback (optional) -------------------------
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // we don't need special handling here
  (void)mac_addr;
  (void)status;
}

// -------------------- setup -----------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(50);

  // LEDs and button
  pinMode(LED_PWR, OUTPUT); digitalWrite(LED_PWR, HIGH); // power LED on
  pinMode(LED_TX, OUTPUT);  digitalWrite(LED_TX, LOW);
  pinMode(LED_LINK, OUTPUT); digitalWrite(LED_LINK, LOW);

  pinMode(PTT_PIN, INPUT_PULLUP);

  // I2S init
  setupI2S();

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // channel 1 — change if you need

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // add peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, PEER_ADDR, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    // may already exist; ignore
  }

  Serial.println("Walkie ready. Hold PTT to speak.");
}

// -------------------- main loop ------------------------------------------------
void loop() {
  // read PTT (active LOW)
  bool pressed = (digitalRead(PTT_PIN) == LOW);

  // Update LINK LED timeout
  if (digitalRead(LED_LINK) && (millis() - lastLinkMs > LINK_LED_TIMEOUT)) {
    digitalWrite(LED_LINK, LOW);
  }

  if (pressed) {
    // Enter transmit mode
    if (!pttPressed) {
      pttPressed = true;
      digitalWrite(LED_TX, HIGH);
      Serial.println("PTT pressed: TX start");
      // brief small delay to avoid transient clicks
      delay(10);
    }
    // Read PCM frame from mic (blocking until FRAME_SAMPLES available)
    static int16_t pcmFrame[FRAME_SAMPLES];
    size_t readBytes = 0;
    esp_err_t r = i2s_read(I2S_NUM_0, pcmFrame, FRAME_SAMPLES * sizeof(int16_t), &readBytes, portMAX_DELAY);
    if (r != ESP_OK || readBytes < FRAME_SAMPLES * sizeof(int16_t)) {
      // failed read — skip this iteration
      return;
    }

    // Optionally apply small software gain here (careful of clipping):
    // for (int i=0;i<FRAME_SAMPLES;i++) { int32_t s = pcmFrame[i]*1.3; if(s>32767) s=32767; if(s<-32768) s=-32768; pcmFrame[i]=s; }

    // encode ADPCM
    static uint8_t adpcmBuf[ADPCM_MAX_OUT_BYTES];
    int outLen = ima_adpcm_encode_frame(pcmFrame, adpcmBuf);

    // send via ESP-NOW
    esp_err_t res = esp_now_send(PEER_ADDR, adpcmBuf, outLen);
    (void)res; // ignore result; optional: check & retry
    // loop to next frame quickly (no delay to keep 20ms cadence). small yield:
    yield();

  } else {
    // Not pressed => listening mode
    if (pttPressed) {
      // just released
      pttPressed = false;
      digitalWrite(LED_TX, LOW);
      Serial.println("PTT released: RX mode");
      // small delay to allow peer to switch to RX
      delay(10);
    }
    // while listening, do nothing here — incoming packets handled in onDataRecv()
    delay(5); // small idle
  }
}

// -------------------- I2S setup -------------------------------------------------
void setupI2S() {
  // single I2S port used for both RX and TX (master)
  i2s_config_t i2s_cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = I2S_DMA_BUFS,
    .dma_buf_len = I2S_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t pin_cfg = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCLK,
    .data_out_num = I2S_SPK_D,
    .data_in_num = I2S_MIC_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_cfg);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

// -------------------- IMA ADPCM encoder ---------------------------------------
int ima_adpcm_encode_frame(const int16_t *inBuf, uint8_t *outBuf) {
  // header: predictor (int16 LE) + index (1 byte)
  int predictor = inBuf[0];
  int index = 0;
  int step = stepTable[index];

  outBuf[0] = (uint8_t)(predictor & 0xFF);
  outBuf[1] = (uint8_t)((predictor >> 8) & 0xFF);
  outBuf[2] = (uint8_t)index;
  size_t outPos = 3;
  uint8_t curByte = 0;
  bool nibbleLow = true;

  for (int i = 1; i < FRAME_SAMPLES; ++i) {
    int sample = inBuf[i];
    int diff = sample - predictor;
    uint8_t code = 0;
    int tmpStep = step;
    if (diff < 0) { code = 8; diff = -diff; }
    if (diff >= tmpStep) { code |= 4; diff -= tmpStep; }
    tmpStep >>= 1;
    if (diff >= tmpStep) { code |= 2; diff -= tmpStep; }
    tmpStep >>= 1;
    if (diff >= tmpStep) { code |= 1; }
    int delta = 0;
    if (code & 4) delta += step;
    if (code & 2) delta += step >> 1;
    if (code & 1) delta += step >> 2;
    delta += step >> 3;
    if (code & 8) predictor -= delta; else predictor += delta;
    if (predictor > 32767) predictor = 32767;
    if (predictor < -32768) predictor = -32768;
    index += indexTable[code & 0x0F];
    if (index < 0) index = 0;
    if (index > 88) index = 88;
    step = stepTable[index];

    if (nibbleLow) {
      curByte = (code & 0x0F);
      nibbleLow = false;
    } else {
      curByte |= ((code & 0x0F) << 4);
      outBuf[outPos++] = curByte;
      curByte = 0;
      nibbleLow = true;
    }
  }
  if (!nibbleLow) {
    outBuf[outPos++] = curByte;
  }
  return (int)outPos;
}

// -------------------- IMA ADPCM decoder ---------------------------------------
void ima_adpcm_decode_frame(const uint8_t *in, size_t in_len, int16_t *outSamples) {
  if (in_len < 3) return;
  int16_t predictor = (int16_t)((in[1] << 8) | in[0]);
  int index = in[2];
  if (index < 0) index = 0;
  if (index > 88) index = 88;
  int step = stepTable[index];

  size_t outPos = 0;
  for (size_t i = 3; i < in_len && outPos < FRAME_SAMPLES; ++i) {
    uint8_t byte = in[i];
    for (int nib = 0; nib < 2 && outPos < FRAME_SAMPLES; ++nib) {
      uint8_t code = (nib == 0) ? (byte & 0x0F) : (byte >> 4);
      int diff = step >> 3;
      if (code & 4) diff += step;
      if (code & 2) diff += step >> 1;
      if (code & 1) diff += step >> 2;
      if (code & 8) predictor -= diff; else predictor += diff;
      if (predictor > 32767) predictor = 32767;
      if (predictor < -32768) predictor = -32768;
      index += indexTable[code];
      if (index < 0) index = 0;
      if (index > 88) index = 88;
      step = stepTable[index];
      outSamples[outPos++] = (int16_t)predictor;
    }
  }
  while (outPos < FRAME_SAMPLES) outSamples[outPos++] = 0;
}
