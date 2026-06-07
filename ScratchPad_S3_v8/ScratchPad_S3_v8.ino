/*
========================================================
  ScratchPad S3 - v8  (SCRATCH DE SAMPLE REAL)
========================================================
  Base: seu v7 validado (motor slip + crossfader + 2 nucleos + I2S API antiga)
  Upgrade: engine de audio do repo DVS-Wireless (ponto-fixo + interpolacao + wrap)
           trocando a melodia sintetizada por SCRUB de um sample PCM real.

  Como funciona o casamento das duas logicas:
   - loop() (nucleo 1): motor FOC + setpoint que escorrega + le velocidade do prato.
       Ja calculavamos  sf = v / NOMINAL_VEL  -> esse e o RATIO de reproducao
       (1.0 = tom normal, negativo = reverso). Mesma ideia do repo do amigo.
   - audioTask (nucleo 0): usa sf para avancar um PONTEIRO FRACIONARIO dentro de
       um buffer PCM (ponto-fixo: 8 bits de fracao). Interpola linearmente entre
       amostras -> toca em qualquer velocidade, reverso suave, e faz loop (wrap).

  Sem SD: gera um loop de teste em PSRAM no boot (kick + hat + baixo) pra voce
  ja sentir o scratch com PCM real. Quando trocar o SD, ligue USE_SD 1.

  Preservado do v7 (NAO mexer na essencia):
   - mapa de GPIOs, I2S API antiga (I2S_CHANNEL_FMT_RIGHT_LEFT / STAND_I2S)
   - separacao por nucleos (audio bloqueia em i2s_write = pacing natural)
   - sensacao de vinil: setpoint que cede ao segurar e sobe gradual ao soltar
   - audio travado pra nao "correr" alem do tom normal pra frente
========================================================
*/

#include <SimpleFOC.h>
#include <driver/i2s.h>
#include <math.h>
#include "esp_heap_caps.h"

// =====================================================
// CONFIG: fonte do sample
// =====================================================
#define USE_SD 0          // 0 = gera loop de teste em PSRAM | 1 = carrega WAV do SD

#if USE_SD
  #include <SPI.h>
  #include <SD.h>
  #define SD_SCK  13
  #define SD_MISO 12
  #define SD_MOSI 11
  #define SD_CS   10
  const char* WAV_PATH = "/loop.wav";   // 16-bit PCM, mono, 22050 Hz (ideal)
#endif

// =====================================================
// Motor / DRV8313 / AS5600   (GPIOs definitivos)
// =====================================================
#define DRV_IN1 4
#define DRV_IN2 5
#define DRV_IN3 6
#define AS5600_SDA 8
#define AS5600_SCL 9

BLDCMotor motor = BLDCMotor(7);                                  // 7 pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(DRV_IN1, DRV_IN2, DRV_IN3);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

const float TARGET_VEL  = -12.0;
const float NOMINAL_VEL = TARGET_VEL;

// ---- Comportamento do prato (knobs do v7) ----
const float RAMP_RATE = 4.0;     // rad/s por segundo: quao gradual sobe de volta
const float DEADBAND  = 2.0;     // folga antes do motor ceder

// =====================================================
// Hall / crossfader
// =====================================================
#define HALL_PIN 1
int hallMin = 4095, hallMax = 0;
const int  CUT_THRESH = 30;
const bool CUT_INVERT = true;
const int  HYST = 5;

// =====================================================
// I2S / UDA1334A   (API antiga - roda liso no S3)
// =====================================================
#define I2S_BCLK 15
#define I2S_WSEL 16
#define I2S_DIN  17
#define SAMPLE_RATE 22050
#define OUTPUT_GAIN 0.90f

// =====================================================
// Limites de reproducao (ratio)
// =====================================================
// Pra frente: travado perto do tom normal (evita "correr"/overshoot, igual v7).
// Pro reverso: deixa puxar mais rapido na mao pra dar o "zip" do scratch.
const float MAX_RATIO_FWD = 1.10f;
const float MAX_RATIO_REV = 3.00f;
const float DEADZONE      = 0.015f;   // abaixo disso, prato parado

// =====================================================
// Compartilhado entre nucleos
// =====================================================
volatile float gSpeedFactor = 0.0f;   // sf = ratio de reproducao (do prato)
volatile bool  gCut         = false;  // crossfader cortou

// =====================================================
// Buffer de sample (PSRAM) + estado de reproducao
// =====================================================
int16_t* gSample   = nullptr;   // PCM mono 16-bit
int32_t  gSampleLen = 0;        // numero de frames (amostras mono)

// posicao em ponto-fixo: 8 bits de fracao  (igual repo DVS)
volatile int32_t wavPos = 0;

// =====================================================
// WRAP da posicao (ponto-fixo)  -- do repo DVS
// =====================================================
static inline int32_t wrapPos(int32_t p) {
  int32_t maxp = gSampleLen << 8;
  if (maxp <= 0) return 0;
  while (p >= maxp) p -= maxp;
  while (p < 0)     p += maxp;
  return p;
}

// =====================================================
// Interpolacao linear (mono)  -- adaptado do repo DVS
// =====================================================
static inline int16_t interpMono(int32_t pos) {
  int frame = pos >> 8;
  int next  = frame + 1;
  if (next >= gSampleLen) next = 0;

  float frac = (pos & 0xFF) / 256.0f;
  float s = gSample[frame] + (gSample[next] - gSample[frame]) * frac;
  return (int16_t)s;
}

// =====================================================
// GERA loop de teste em PSRAM (kick + hat + baixo)
// =====================================================
void gerarSampleTeste() {
  const float BPM  = 120.0f;
  const int   BARS = 1;                       // 1 compasso
  float secs = (60.0f / BPM) * 4.0f * BARS;   // 4 tempos
  gSampleLen = (int32_t)(secs * SAMPLE_RATE);

  gSample = (int16_t*)heap_caps_malloc(gSampleLen * sizeof(int16_t),
                                       MALLOC_CAP_SPIRAM);
  if (!gSample) {
    Serial.println("FALHA ao alocar sample na PSRAM!");
    gSampleLen = 0;
    return;
  }

  float beatSec = 60.0f / BPM;

  for (int32_t i = 0; i < gSampleLen; i++) {
    float t      = (float)i / SAMPLE_RATE;
    float inBeat = fmodf(t, beatSec);          // tempo dentro do tempo atual
    int   beat   = (int)(t / beatSec) % 4;
    float v = 0.0f;

    // baixo continuo (sub) ~ 80 Hz com leve harmonico
    v += 0.30f * sinf(2.0f * PI * 80.0f * t);
    v += 0.10f * sinf(2.0f * PI * 160.0f * t);

    // kick nos tempos (envelope rapido + sweep de pitch)
    if (inBeat < 0.12f) {
      float env = expf(-inBeat * 30.0f);
      float f   = 120.0f - 80.0f * (inBeat / 0.12f);   // 120->40 Hz
      v += 0.85f * env * sinf(2.0f * PI * f * t);
    }

    // hi-hat no contratempo (ruido com decay curto)
    if (beat % 2 == 1 && inBeat < 0.05f) {
      float env = expf(-inBeat * 120.0f);
      v += 0.25f * env * ((random(0, 2000) - 1000) / 1000.0f);
    }

    if (v > 1.0f)  v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    gSample[i] = (int16_t)(v * 12000.0f);
  }

  Serial.printf("Sample de teste: %d frames (%.2fs) na PSRAM\n",
                gSampleLen, secs);
}

// =====================================================
// (Opcional) CARREGA WAV do SD pra PSRAM
// =====================================================
#if USE_SD
bool carregarWavSD(const char* path) {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 20000000)) {
    Serial.println("SD nao iniciou.");
    return false;
  }
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.println("WAV nao encontrado."); return false; }

  // header WAV canonico (44 bytes); le campos basicos
  uint8_t hdr[44];
  f.read(hdr, 44);
  uint16_t channels = hdr[22] | (hdr[23] << 8);
  uint32_t rate     = hdr[24] | (hdr[25] << 8) | (hdr[26] << 16) | (hdr[27] << 24);
  uint16_t bits     = hdr[34] | (hdr[35] << 8);
  uint32_t dataSize = hdr[40] | (hdr[41] << 8) | (hdr[42] << 16) | (hdr[43] << 24);

  if (bits != 16) { Serial.println("Use WAV 16-bit PCM."); f.close(); return false; }
  if (rate != SAMPLE_RATE)
    Serial.printf("AVISO: WAV a %u Hz (esperado %d) -> pitch sai deslocado.\n",
                  rate, SAMPLE_RATE);

  uint32_t totalSamples = dataSize / 2;                    // amostras 16-bit
  gSampleLen = (channels == 2) ? totalSamples / 2 : totalSamples;

  gSample = (int16_t*)heap_caps_malloc(gSampleLen * sizeof(int16_t),
                                       MALLOC_CAP_SPIRAM);
  if (!gSample) { Serial.println("Sem PSRAM pro WAV."); f.close(); return false; }

  if (channels == 2) {
    int16_t st[2];
    for (int32_t i = 0; i < gSampleLen; i++) {
      f.read((uint8_t*)st, 4);
      gSample[i] = (int16_t)(((int)st[0] + st[1]) / 2);   // estereo -> mono
    }
  } else {
    f.read((uint8_t*)gSample, gSampleLen * 2);
  }
  f.close();
  Serial.printf("WAV carregado: %d frames mono.\n", gSampleLen);
  return true;
}
#endif

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
    if (gSampleLen <= 0) {                 // ainda sem sample: silencio
      memset(buffer, 0, sizeof(buffer));
      size_t w; i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &w, portMAX_DELAY);
      continue;
    }

    float sf  = gSpeedFactor;              // ratio de reproducao do prato
    bool  cut = gCut;

    // clamp assimetrico (frente travada, reverso mais solto)
    if (sf >  MAX_RATIO_FWD) sf =  MAX_RATIO_FWD;
    if (sf < -MAX_RATIO_REV) sf = -MAX_RATIO_REV;
    if (fabsf(sf) < DEADZONE) sf = 0.0f;

    // step em ponto-fixo: 1.0x = 256 (avanca 1 amostra por amostra)
    int32_t step = (int32_t)(sf * 256.0f);

    int32_t pos = wavPos;
    for (int i = 0; i < FRAMES; i++) {
      pos = wrapPos(pos + step);
      int16_t s = cut ? 0 : (int16_t)(interpMono(pos) * OUTPUT_GAIN);
      buffer[i * 2]     = s;
      buffer[i * 2 + 1] = s;
    }
    wavPos = pos;                          // mantem posicao mesmo no cut (DVS-like)

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
  Serial.println("=== ScratchPad S3 v8 ===");

  // ---- carrega/gera o sample ANTES da task de audio ----
#if USE_SD
  if (!carregarWavSD(WAV_PATH)) {
    Serial.println("Fallback: gerando sample de teste.");
    gerarSampleTeste();
  }
#else
  gerarSampleTeste();
#endif

  // ---- motor (igual v7) ----
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
  motor.voltage_limit = 3.0;
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 0.3;
  motor.PID_velocity.output_ramp = 100;   // o slew real eh feito pelo setpoint
  motor.LPF_velocity.Tf = 0.05;

  motor.init();
  motor.initFOC();
  Serial.println("Motor pronto.");

  xTaskCreatePinnedToCore(audioTask, "audio", 8192, NULL, 1, NULL, 0);
  Serial.println("Rodando!");
}

// =====================================================
// LOOP (nucleo 1): motor + sensores   (igual v7)
// =====================================================
unsigned long lastHall = 0;
unsigned long lastMicros = 0;
float setVel = 0;   // setpoint que escorrega

void loop() {
  motor.loopFOC();

  unsigned long now = micros();
  float dt = (lastMicros == 0) ? 0 : (now - lastMicros) * 1e-6f;
  lastMicros = now;

  float v = motor.shaftVelocity();         // velocidade real (com sinal)

  // ---- setpoint que escorrega (sensacao de vinil) ----
  float dir    = (TARGET_VEL < 0) ? -1.0f : 1.0f;
  float vDir   = v * dir;
  float setDir = setVel * dir;
  float tgtDir = fabs(TARGET_VEL);

  if (vDir < setDir - DEADBAND) {
    setDir = vDir + DEADBAND;              // prato freado -> setpoint cede
  } else {
    setDir += RAMP_RATE * dt;             // livre -> sobe devagar ate o alvo
    if (setDir > tgtDir) setDir = tgtDir;
  }
  if (setDir < 0) setDir = 0;

  setVel = setDir * dir;
  motor.move(setVel);

  // ---- ratio de reproducao = velocidade real / nominal ----
  // (clamp fino fica na audioTask; aqui so passa o valor)
  gSpeedFactor = v / NOMINAL_VEL;

  // ---- crossfader (cut quadrado com histerese) ----
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
