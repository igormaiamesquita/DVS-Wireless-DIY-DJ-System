/*
========================================================
  ScratchPad S3 - v10  (SD + ENCODER EC11)
========================================================
  Base: v8 (scratch de sample real + motor slip/low-heat + crossfader).
  Novo nesta versao:
   - Cartao SD (modulo HW-125) lendo a pasta /scratch
   - Encoder EC11: GIRAR navega os samples, CLICAR (SW) carrega o selecionado
   - 6 botoes (leitura simples com debounce) p/ funcoes ao vivo (provisorio)
   - Fallback: se o SD falhar ou /scratch estiver vazia, gera sample de teste

  Pinos do SD foram ajustados pro jeito que voce soldou:
        MISO = 11   |   MOSI = 12   (invertidos de proposito - ok no S3)

  IMPORTANTE (carregar arquivos sem tirar o cartao):
   Use o sketch separado "SD_USB_Drive.ino" (modo pendrive USB MSC) pra copiar
   os WAV pro cartao pelo PC. Estrutura no cartao:
        /scratch/   -> samples curtos pra esfregar (WAV 16-bit mono 22050 Hz)
        /beats/     -> batidas (usado na proxima etapa, streaming)

  Config Arduino IDE desta v10: igual a sua de sempre
   (USB Mode: Hardware CDC and JTAG | CDC On Boot: Disabled).
========================================================
*/

#include <SimpleFOC.h>
#include <driver/i2s.h>
#include <math.h>
#include "esp_heap_caps.h"
#include <SPI.h>
#include <SD.h>

// =====================================================
// Motor / DRV8313 / AS5600
// =====================================================
#define DRV_IN1 4
#define DRV_IN2 5
#define DRV_IN3 6
#define AS5600_SDA 8
#define AS5600_SCL 9

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(DRV_IN1, DRV_IN2, DRV_IN3);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

const float TARGET_VEL  = -12.0;
const float NOMINAL_VEL = TARGET_VEL;

const float RAMP_RATE = 3.0;
const float DEADBAND  = 2.0;
const float V_RUN     = 2.0;   // torque quando livre
const float V_YIELD   = 0.6;   // torque quando voce intervem (cede / esfria)

// =====================================================
// Hall / crossfader
// =====================================================
#define HALL_PIN 1
int hallMin = 4095, hallMax = 0;
const int  CUT_THRESH = 30;
const bool CUT_INVERT = true;
const int  HYST = 5;

// =====================================================
// SD (modulo HW-125)  -- MISO/MOSI invertidos conforme a solda
// =====================================================
#define SD_SCK  13
#define SD_MISO 11
#define SD_MOSI 12
#define SD_CS   10
#define SD_FREQ 10000000        // 10 MHz; se der erro de leitura, baixe p/ 4000000

SPIClass spiSD(FSPI);

// =====================================================
// Encoder EC11
// =====================================================
#define ENC_A  40
#define ENC_B  41
#define ENC_SW 42

// =====================================================
// Botoes (GND comum)  -- 45/46 sao strapping (nao segurar no boot)
// =====================================================
const uint8_t BTN_PINS[6] = {18, 38, 39, 7, 45, 46};

// =====================================================
// I2S / UDA1334A
// =====================================================
#define I2S_BCLK 15
#define I2S_WSEL 16
#define I2S_DIN  17
#define SAMPLE_RATE 22050

// =====================================================
// Estado de audio (compartilhado)
// =====================================================
volatile float gVol         = 0.90f;   // volume mestre (botoes ajustam)
volatile float gSpeedFactor = 0.0f;    // ratio de reproducao (do prato)
volatile bool  gCut         = false;   // crossfader
volatile bool  gMuteScratch = false;   // mute manual (botao)

const float MAX_RATIO_FWD = 1.10f;
const float MAX_RATIO_REV = 3.00f;
const float DEADZONE      = 0.015f;

// =====================================================
// Buffer do sample (PSRAM)
// =====================================================
int16_t* gSample = nullptr;
int32_t  gSampleLen = 0;
volatile bool gAudioReady = false;     // false durante troca de sample -> silencio
volatile int32_t scratchPos = 0;       // ponto-fixo (8 bits de fracao)

// =====================================================
// Lista de arquivos /scratch
// =====================================================
#define MAX_FILES 64
String scratchFiles[MAX_FILES];
int    scratchCount = 0;
int    selIndex     = 0;

// =====================================================
// WRAP / interpolacao
// =====================================================
static inline int32_t wrapPos(int32_t p) {
  int32_t maxp = gSampleLen << 8;
  if (maxp <= 0) return 0;
  while (p >= maxp) p -= maxp;
  while (p < 0)     p += maxp;
  return p;
}
static inline int16_t interpMono(int32_t pos) {
  int frame = pos >> 8;
  int next  = frame + 1;
  if (next >= gSampleLen) next = 0;
  float frac = (pos & 0xFF) / 256.0f;
  float s = gSample[frame] + (gSample[next] - gSample[frame]) * frac;
  return (int16_t)s;
}

// =====================================================
// Carrega um WAV (16-bit) do SD pra PSRAM (scan de chunks)
// =====================================================
int16_t* loadWavToPSRAM(const char* path, int32_t* lenOut) {
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.printf("Nao abriu %s\n", path); return nullptr; }

  char tag[4];
  f.read((uint8_t*)tag, 4);                 // "RIFF"
  f.seek(8);
  f.read((uint8_t*)tag, 4);                 // "WAVE"

  uint16_t channels = 1, bits = 16;
  uint32_t rate = SAMPLE_RATE, dataSize = 0, dataPos = 0;

  while (f.available()) {
    char id[4];
    if (f.read((uint8_t*)id, 4) != 4) break;
    uint8_t sb[4]; f.read(sb, 4);
    uint32_t sz = sb[0] | (sb[1] << 8) | (sb[2] << 16) | (sb[3] << 24);

    if (memcmp(id, "fmt ", 4) == 0) {
      uint8_t fmt[16]; f.read(fmt, 16);
      channels = fmt[2] | (fmt[3] << 8);
      rate     = fmt[4] | (fmt[5] << 8) | (fmt[6] << 16) | (fmt[7] << 24);
      bits     = fmt[14] | (fmt[15] << 8);
      if (sz > 16) f.seek(f.position() + (sz - 16));
    } else if (memcmp(id, "data", 4) == 0) {
      dataSize = sz; dataPos = f.position(); break;
    } else {
      f.seek(f.position() + sz + (sz & 1));  // pula chunk (+ padding impar)
    }
  }

  if (dataSize == 0 || bits != 16) {
    Serial.println("WAV invalido (precisa ser 16-bit PCM).");
    f.close(); return nullptr;
  }
  if (rate != SAMPLE_RATE)
    Serial.printf("AVISO: %s a %u Hz (esperado %d) -> pitch desloca.\n", path, rate, SAMPLE_RATE);

  f.seek(dataPos);
  uint32_t totalSamples = dataSize / 2;
  int32_t len = (channels == 2) ? totalSamples / 2 : totalSamples;

  int16_t* buf = (int16_t*)heap_caps_malloc(len * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  if (!buf) { Serial.println("Sem PSRAM."); f.close(); return nullptr; }

  if (channels == 2) {
    int16_t st[2];
    for (int32_t i = 0; i < len; i++) {
      f.read((uint8_t*)st, 4);
      buf[i] = (int16_t)(((int)st[0] + st[1]) / 2);
    }
  } else {
    uint8_t* p = (uint8_t*)buf;
    uint32_t toRead = (uint32_t)len * 2;
    while (toRead) {
      int n = f.read(p, toRead > 4096 ? 4096 : toRead);
      if (n <= 0) break;
      p += n; toRead -= n;
    }
  }
  f.close();
  if (lenOut) *lenOut = len;
  return buf;
}

// =====================================================
// Lista os .wav de uma pasta
// =====================================================
int scanFolder(const char* dir, String* list, int maxN) {
  File d = SD.open(dir);
  if (!d || !d.isDirectory()) { Serial.printf("Pasta %s nao existe.\n", dir); return 0; }

  int n = 0;
  File e = d.openNextFile();
  while (e && n < maxN) {
    if (!e.isDirectory()) {
      String nm = e.name();
      // so o nome do arquivo (sem caminho), p/ filtrar lixo do macOS
      String base = nm;
      int slash = base.lastIndexOf('/');
      if (slash >= 0) base = base.substring(slash + 1);

      String low = base; low.toLowerCase();
      // ignora ocultos/AppleDouble (._arquivo, .Spotlight...) e pega so .wav
      if (!base.startsWith(".") && low.endsWith(".wav")) {
        list[n++] = nm.startsWith("/") ? nm : (String(dir) + "/" + nm);
      }
    }
    e = d.openNextFile();
  }
  d.close();
  return n;
}

// =====================================================
// Sample de teste (fallback sem SD)
// =====================================================
void gerarSampleTeste() {
  float secs = 1.0f;
  gSampleLen = (int32_t)(secs * SAMPLE_RATE);
  gSample = (int16_t*)heap_caps_malloc(gSampleLen * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  if (!gSample) { gSampleLen = 0; return; }
  for (int32_t i = 0; i < gSampleLen; i++) {
    float t = (float)i / SAMPLE_RATE;
    float vib = 1.0f + 0.02f * sinf(2.0f * PI * 6.0f * t);
    float base = 220.0f * vib;
    float v = 0.5f * sinf(2*PI*base*t) + 0.3f * sinf(2*PI*base*2*t) + 0.2f * sinf(2*PI*base*3*t);
    float env = 1.0f;
    if (t < 0.02f) env = t / 0.02f;
    if (t > secs - 0.02f) env = (secs - t) / 0.02f;
    gSample[i] = (int16_t)(v * env * 27500.0f);
  }
  Serial.println("Fallback: sample de teste gerado.");
}

// =====================================================
// Troca o sample em runtime (com swap seguro p/ a audioTask)
// =====================================================
void aplicarSample(int idx) {
  if (idx < 0 || idx >= scratchCount) return;
  Serial.printf("Carregando [%d]: %s\n", idx, scratchFiles[idx].c_str());

  int32_t newLen = 0;
  int16_t* nb = loadWavToPSRAM(scratchFiles[idx].c_str(), &newLen);
  if (!nb) { Serial.println("Falha ao carregar."); return; }

  gAudioReady = false;          // audioTask passa a mandar silencio
  delay(30);                    // garante que ela saiu do bloco atual
  int16_t* old = gSample;
  gSample = nb;
  gSampleLen = newLen;
  scratchPos = 0;
  gAudioReady = true;
  if (old) free(old);
  Serial.printf("OK: %d frames (%.2fs)\n", newLen, (float)newLen / SAMPLE_RATE);
}

// =====================================================
// AUDIO TASK (nucleo 0)
// =====================================================
void audioTask(void *param) {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_WSEL,
    .data_out_num = I2S_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);

  const int FRAMES = 256;
  int16_t buffer[FRAMES * 2];

  for (;;) {
    if (!gAudioReady || gSampleLen <= 0) {
      memset(buffer, 0, sizeof(buffer));
      size_t w; i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &w, portMAX_DELAY);
      continue;
    }

    float sf  = gSpeedFactor;
    bool  mute = gCut || gMuteScratch;
    float vol = gVol;

    if (sf >  MAX_RATIO_FWD) sf =  MAX_RATIO_FWD;
    if (sf < -MAX_RATIO_REV) sf = -MAX_RATIO_REV;
    if (fabsf(sf) < DEADZONE) sf = 0.0f;
    int32_t step = (int32_t)(sf * 256.0f);

    int32_t pos = scratchPos;
    for (int i = 0; i < FRAMES; i++) {
      pos = wrapPos(pos + step);
      int16_t s = mute ? 0 : (int16_t)(interpMono(pos) * vol);
      buffer[i * 2]     = s;
      buffer[i * 2 + 1] = s;
    }
    scratchPos = pos;

    size_t written;
    i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &written, portMAX_DELAY);
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("=== ScratchPad S3 v10 (SD + encoder) ===");

  // ---- entradas ----
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  for (int i = 0; i < 6; i++) pinMode(BTN_PINS[i], INPUT_PULLUP);

  // ---- SD ----
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (SD.begin(SD_CS, spiSD, SD_FREQ)) {
    Serial.println("SD OK.");
    scratchCount = scanFolder("/scratch", scratchFiles, MAX_FILES);
    Serial.printf("%d sample(s) em /scratch\n", scratchCount);
    if (scratchCount > 0) {
      gSample = loadWavToPSRAM(scratchFiles[0].c_str(), &gSampleLen);
      selIndex = 0;
    }
  } else {
    Serial.println("SD nao iniciou.");
  }
  if (!gSample) gerarSampleTeste();
  gAudioReady = (gSampleLen > 0);

  // ---- motor ----
  Wire.begin(AS5600_SDA, AS5600_SCL);
  Wire.setClock(400000);
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 10;
  driver.pwm_frequency = 20000;
  driver.init();
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity;
  motor.voltage_limit = V_RUN;
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 0.2;
  motor.PID_velocity.output_ramp = 100;
  motor.LPF_velocity.Tf = 0.05;

  motor.init();
  motor.initFOC();
  Serial.println("Motor pronto.");

  xTaskCreatePinnedToCore(audioTask, "audio", 8192, NULL, 1, NULL, 0);
  Serial.println("Rodando!");
}

// =====================================================
// ENCODER (poll de quadratura no loop)
// =====================================================
void lerEncoder() {
  static const int8_t tbl[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  static uint8_t last = 0;
  static int8_t acc = 0;

  uint8_t ab = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  acc += tbl[(last << 2) | ab];
  last = ab;

  if (acc >= 4) {                           // um detente p/ frente
    acc = 0;
    if (selIndex < scratchCount - 1) selIndex++;
    Serial.printf("> sel %d: %s\n", selIndex,
                  scratchCount ? scratchFiles[selIndex].c_str() : "(vazio)");
  } else if (acc <= -4) {                    // um detente p/ tras
    acc = 0;
    if (selIndex > 0) selIndex--;
    Serial.printf("> sel %d: %s\n", selIndex,
                  scratchCount ? scratchFiles[selIndex].c_str() : "(vazio)");
  }

  // clique = carrega o selecionado
  static bool lastSw = HIGH;
  static uint32_t tSw = 0;
  bool sw = digitalRead(ENC_SW);
  if (lastSw == HIGH && sw == LOW && millis() - tSw > 200) {
    tSw = millis();
    aplicarSample(selIndex);
  }
  lastSw = sw;
}

// =====================================================
// BOTOES (debounce simples)  -- funcoes provisorias
// =====================================================
void onButton(int i) {
  switch (i) {
    case 0: if (selIndex < scratchCount - 1) { selIndex++; aplicarSample(selIndex); } break; // proximo
    case 1: if (selIndex > 0)                { selIndex--; aplicarSample(selIndex); } break; // anterior
    case 2: gMuteScratch = !gMuteScratch; break;                                             // mute
    case 3: gVol -= 0.1f; if (gVol < 0)    gVol = 0;    break;                               // vol -
    case 4: gVol += 0.1f; if (gVol > 1.2f) gVol = 1.2f; break;                               // vol +
    case 5: scratchPos = 0; break;                                                           // reset pos
  }
  Serial.printf("BTN%d  (vol=%.1f mute=%d)\n", i + 1, gVol, gMuteScratch);
}

void lerBotoes() {
  static bool last[6] = {HIGH,HIGH,HIGH,HIGH,HIGH,HIGH};
  static uint32_t t[6] = {0,0,0,0,0,0};
  for (int i = 0; i < 6; i++) {
    bool s = digitalRead(BTN_PINS[i]);
    if (last[i] == HIGH && s == LOW && millis() - t[i] > 200) {
      t[i] = millis();
      onButton(i);
    }
    last[i] = s;
  }
}

// =====================================================
// LOOP (nucleo 1): motor + sensores + encoder + botoes
// =====================================================
unsigned long lastHall = 0;
unsigned long lastMicros = 0;
float setVel = 0;

void loop() {
  motor.loopFOC();

  unsigned long now = micros();
  float dt = (lastMicros == 0) ? 0 : (now - lastMicros) * 1e-6f;
  lastMicros = now;

  float v = motor.shaftVelocity();

  // setpoint que escorrega + torque dinamico (anti-calor)
  float dir    = (TARGET_VEL < 0) ? -1.0f : 1.0f;
  float vDir   = v * dir;
  float setDir = setVel * dir;
  float tgtDir = fabs(TARGET_VEL);

  if (vDir < setDir - DEADBAND) {
    setDir = vDir;
    motor.voltage_limit = V_YIELD;
    motor.PID_velocity.reset();
  } else {
    setDir += RAMP_RATE * dt;
    if (setDir > tgtDir) setDir = tgtDir;
    motor.voltage_limit = V_RUN;
  }
  if (setDir < 0) setDir = 0;

  setVel = setDir * dir;
  motor.move(setVel);

  gSpeedFactor = v / NOMINAL_VEL;

  // entradas
  lerEncoder();
  lerBotoes();

  // crossfader
  if (millis() - lastHall > 20) {
    lastHall = millis();
    int raw = analogRead(HALL_PIN);
    if (raw < hallMin) hallMin = raw;
    if (raw > hallMax) hallMax = raw;

    int pos = 64;
    if (hallMax - hallMin > 100) {
      pos = map(raw, hallMin, hallMax, 0, 127);
      pos = constrain(pos, 0, 127);
    }
    int p = CUT_INVERT ? (127 - pos) : pos;
    static bool cutState = false;
    if (!cutState && p < CUT_THRESH) cutState = true;
    else if (cutState && p > CUT_THRESH + HYST) cutState = false;
    gCut = cutState;
  }
}
