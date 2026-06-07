/*
========================================================
  ScratchPad S3 - v9  (SCRATCH + BATIDA DE FUNDO / MIX)
========================================================
  Objetivo: tocar uma BATIDA de acompanhamento em ritmo constante e fazer o
  SCRATCH por cima, misturando os dois em software e mandando pro UDA1334A.

  Dois canais (somados na audioTask):
   - CANAL SCRATCH : engine da v8 (scrub seguindo o prato, ponto-fixo + interp).
                     O crossfader corta SO esse canal (voce chopa o scratch).
   - CANAL BATIDA  : toca em 1.0x continuo, NAO depende do prato nem do
                     crossfader (e o acompanhamento que segue tocando).

  Sem SD: gera os dois buffers em PSRAM (batida kick/snare/hat + sample vocal
  pra scratchar). Quando trocar o SD, ligue USE_SD 1 e ele carrega:
        /beat.wav     -> batida de fundo
        /scratch.wav  -> sample do scratch
  (16-bit PCM, mono, 22050 Hz de preferencia).

  Hardware/arquitetura preservados do v7/v8:
   - GPIOs definitivos, I2S API antiga, 2 nucleos, motor slip, crossfader.
========================================================
*/

#include <SimpleFOC.h>
#include <driver/i2s.h>
#include <math.h>
#include "esp_heap_caps.h"

// =====================================================
// CONFIG: fonte dos samples
// =====================================================
#define USE_SD 0          // 0 = gera buffers em PSRAM | 1 = carrega WAV do SD

#if USE_SD
  #include <SPI.h>
  #include <SD.h>
  #define SD_SCK  13
  #define SD_MISO 12
  #define SD_MOSI 11
  #define SD_CS   10
  const char* BEAT_PATH    = "/beat.wav";
  const char* SCRATCH_PATH = "/scratch.wav";
#endif

// =====================================================
// Motor / DRV8313 / AS5600   (GPIOs definitivos)
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
// I2S / UDA1334A
// =====================================================
#define I2S_BCLK 15
#define I2S_WSEL 16
#define I2S_DIN  17
#define SAMPLE_RATE 22050

// =====================================================
// MIX (ganhos independentes + anti-clip)
// =====================================================
const float SCRATCH_GAIN = 0.85f;   // volume do canal scratch
const float BEAT_GAIN     = 0.65f;   // volume da batida de fundo

// limites de reproducao do scratch (ratio)
const float MAX_RATIO_FWD = 1.10f;
const float MAX_RATIO_REV = 3.00f;
const float DEADZONE      = 0.015f;

// =====================================================
// Compartilhado entre nucleos
// =====================================================
volatile float gSpeedFactor = 0.0f;   // ratio de reproducao do prato (scratch)
volatile bool  gCut         = false;  // crossfader cortou o scratch

// =====================================================
// Buffers (PSRAM)
// =====================================================
int16_t* gScratch    = nullptr;   int32_t gScratchLen = 0;   // scrub pelo prato
int16_t* gBeat       = nullptr;   int32_t gBeatLen    = 0;   // batida fixa

// posicoes de reproducao
volatile int32_t scratchPos = 0;   // ponto-fixo (8 bits de fracao)
int32_t          beatPos    = 0;   // frames inteiros (avanca 1 por amostra)

// =====================================================
// WRAP / interpolacao do canal scratch
// =====================================================
static inline int32_t wrapScratch(int32_t p) {
  int32_t maxp = gScratchLen << 8;
  if (maxp <= 0) return 0;
  while (p >= maxp) p -= maxp;
  while (p < 0)     p += maxp;
  return p;
}

static inline int16_t interpScratch(int32_t pos) {
  int frame = pos >> 8;
  int next  = frame + 1;
  if (next >= gScratchLen) next = 0;
  float frac = (pos & 0xFF) / 256.0f;
  float s = gScratch[frame] + (gScratch[next] - gScratch[frame]) * frac;
  return (int16_t)s;
}

// =====================================================
// Helpers de alocacao PSRAM
// =====================================================
int16_t* alocaPSRAM(int32_t frames) {
  return (int16_t*)heap_caps_malloc(frames * sizeof(int16_t), MALLOC_CAP_SPIRAM);
}

// =====================================================
// GERA batida de teste (kick / snare / hat) em PSRAM
// =====================================================
void gerarBeatTeste() {
  const float BPM = 95.0f;
  float beatSec = 60.0f / BPM;
  float secs = beatSec * 4.0f;            // 1 compasso (4 tempos)
  gBeatLen = (int32_t)(secs * SAMPLE_RATE);

  gBeat = alocaPSRAM(gBeatLen);
  if (!gBeat) { Serial.println("Sem PSRAM pra batida!"); gBeatLen = 0; return; }

  for (int32_t i = 0; i < gBeatLen; i++) {
    float t      = (float)i / SAMPLE_RATE;
    int   beat   = (int)(t / beatSec) % 4;
    float inBeat = fmodf(t, beatSec);
    float inHalf = fmodf(t, beatSec * 0.5f);
    float v = 0.0f;

    // kick nos tempos 0 e 2
    if ((beat == 0 || beat == 2) && inBeat < 0.15f) {
      float env = expf(-inBeat * 28.0f);
      float f   = 130.0f - 90.0f * (inBeat / 0.15f);
      v += 0.90f * env * sinf(2.0f * PI * f * t);
    }
    // snare nos tempos 1 e 3 (tom + ruido)
    if ((beat == 1 || beat == 3) && inBeat < 0.18f) {
      float env = expf(-inBeat * 22.0f);
      v += 0.45f * env * sinf(2.0f * PI * 190.0f * t);
      v += 0.45f * env * ((random(0, 2000) - 1000) / 1000.0f);
    }
    // hat nas colcheias
    if (inHalf < 0.03f) {
      float env = expf(-inHalf * 150.0f);
      v += 0.22f * env * ((random(0, 2000) - 1000) / 1000.0f);
    }

    if (v > 1.0f)  v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    gBeat[i] = (int16_t)(v * 12000.0f);
  }
  Serial.printf("Batida de teste: %d frames (%.2fs)\n", gBeatLen, secs);
}

// =====================================================
// GERA sample "vocal" de teste (pra scratchar) em PSRAM
// =====================================================
void gerarScratchTeste() {
  float secs = 1.0f;                       // frase curta pra esfregar
  gScratchLen = (int32_t)(secs * SAMPLE_RATE);

  gScratch = alocaPSRAM(gScratchLen);
  if (!gScratch) { Serial.println("Sem PSRAM pro scratch!"); gScratchLen = 0; return; }

  for (int32_t i = 0; i < gScratchLen; i++) {
    float t = (float)i / SAMPLE_RATE;
    // formantes + vibrato -> som tipo "aah/uow", bom pra scratch
    float vib  = 1.0f + 0.02f * sinf(2.0f * PI * 6.0f * t);
    float base = 220.0f * vib;
    float v = 0.0f;
    v += 0.50f * sinf(2.0f * PI * base * t);
    v += 0.30f * sinf(2.0f * PI * base * 2.0f * t);
    v += 0.20f * sinf(2.0f * PI * base * 3.0f * t);
    // envelope suave nas pontas (evita clique no loop)
    float env = 1.0f;
    if (t < 0.02f)        env = t / 0.02f;
    if (t > secs - 0.02f) env = (secs - t) / 0.02f;
    v *= env;
    gScratch[i] = (int16_t)(v * 11000.0f);
  }
  Serial.printf("Sample scratch de teste: %d frames (%.2fs)\n", gScratchLen, secs);
}

// =====================================================
// (Opcional) CARREGA WAV do SD pra PSRAM
// =====================================================
#if USE_SD
bool carregarWav(const char* path, int16_t** dst, int32_t* lenOut) {
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.printf("WAV nao encontrado: %s\n", path); return false; }

  uint8_t hdr[44];
  f.read(hdr, 44);
  uint16_t channels = hdr[22] | (hdr[23] << 8);
  uint32_t rate     = hdr[24] | (hdr[25] << 8) | (hdr[26] << 16) | (hdr[27] << 24);
  uint16_t bits     = hdr[34] | (hdr[35] << 8);
  uint32_t dataSize = hdr[40] | (hdr[41] << 8) | (hdr[42] << 16) | (hdr[43] << 24);

  if (bits != 16) { Serial.println("Use WAV 16-bit PCM."); f.close(); return false; }
  if (rate != SAMPLE_RATE)
    Serial.printf("AVISO: %s a %u Hz (esperado %d).\n", path, rate, SAMPLE_RATE);

  uint32_t totalSamples = dataSize / 2;
  int32_t len = (channels == 2) ? totalSamples / 2 : totalSamples;

  int16_t* buf = alocaPSRAM(len);
  if (!buf) { Serial.println("Sem PSRAM pro WAV."); f.close(); return false; }

  if (channels == 2) {
    int16_t st[2];
    for (int32_t i = 0; i < len; i++) {
      f.read((uint8_t*)st, 4);
      buf[i] = (int16_t)(((int)st[0] + st[1]) / 2);
    }
  } else {
    f.read((uint8_t*)buf, len * 2);
  }
  f.close();
  *dst = buf; *lenOut = len;
  Serial.printf("WAV %s: %d frames mono.\n", path, len);
  return true;
}
#endif

// =====================================================
// AUDIO TASK (nucleo 0) -- mix scratch + batida
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
    float sf  = gSpeedFactor;
    bool  cut = gCut;

    // clamp do ratio do scratch
    if (sf >  MAX_RATIO_FWD) sf =  MAX_RATIO_FWD;
    if (sf < -MAX_RATIO_REV) sf = -MAX_RATIO_REV;
    if (fabsf(sf) < DEADZONE) sf = 0.0f;
    int32_t step = (int32_t)(sf * 256.0f);

    int32_t sp = scratchPos;
    int32_t bp = beatPos;

    for (int i = 0; i < FRAMES; i++) {
      // --- canal scratch (segue o prato, cortado pelo crossfader) ---
      int32_t mixv = 0;
      if (gScratchLen > 0) {
        sp = wrapScratch(sp + step);
        if (!cut)
          mixv += (int32_t)(interpScratch(sp) * SCRATCH_GAIN);
      }
      // --- canal batida (ritmo fixo, sempre tocando) ---
      if (gBeatLen > 0) {
        mixv += (int32_t)(gBeat[bp] * BEAT_GAIN);
        bp++;
        if (bp >= gBeatLen) bp = 0;
      }
      // --- soma com clamp anti-clip ---
      if (mixv >  32767) mixv =  32767;
      if (mixv < -32768) mixv = -32768;

      buffer[i * 2]     = (int16_t)mixv;
      buffer[i * 2 + 1] = (int16_t)mixv;
    }
    scratchPos = sp;
    beatPos    = bp;

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
  Serial.println("=== ScratchPad S3 v9 (scratch + batida) ===");

  // ---- samples ANTES da task de audio ----
#if USE_SD
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  bool sdOk = SD.begin(SD_CS, SPI, 20000000);
  if (!sdOk) Serial.println("SD nao iniciou -> usando buffers de teste.");

  if (!(sdOk && carregarWav(BEAT_PATH, &gBeat, &gBeatLen)))
    gerarBeatTeste();
  if (!(sdOk && carregarWav(SCRATCH_PATH, &gScratch, &gScratchLen)))
    gerarScratchTeste();
#else
  gerarBeatTeste();
  gerarScratchTeste();
#endif

  // ---- motor (igual v7/v8) ----
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
  motor.PID_velocity.output_ramp = 100;
  motor.LPF_velocity.Tf = 0.05;

  motor.init();
  motor.initFOC();
  Serial.println("Motor pronto.");

  xTaskCreatePinnedToCore(audioTask, "audio", 8192, NULL, 1, NULL, 0);
  Serial.println("Rodando!");
}

// =====================================================
// LOOP (nucleo 1): motor + sensores   (igual v7/v8)
// =====================================================
unsigned long lastHall = 0;
unsigned long lastMicros = 0;
float setVel = 0;   // setpoint que escorrega

void loop() {
  motor.loopFOC();

  unsigned long now = micros();
  float dt = (lastMicros == 0) ? 0 : (now - lastMicros) * 1e-6f;
  lastMicros = now;

  float v = motor.shaftVelocity();

  // setpoint que escorrega (sensacao de vinil)
  float dir    = (TARGET_VEL < 0) ? -1.0f : 1.0f;
  float vDir   = v * dir;
  float setDir = setVel * dir;
  float tgtDir = fabs(TARGET_VEL);

  if (vDir < setDir - DEADBAND) {
    setDir = vDir + DEADBAND;
  } else {
    setDir += RAMP_RATE * dt;
    if (setDir > tgtDir) setDir = tgtDir;
  }
  if (setDir < 0) setDir = 0;

  setVel = setDir * dir;
  motor.move(setVel);

  // ratio de reproducao do scratch = velocidade real / nominal
  gSpeedFactor = v / NOMINAL_VEL;

  // crossfader (corta o canal scratch)
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
