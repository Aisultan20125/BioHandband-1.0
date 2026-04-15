#include <Wire.h>
#include <U8g2lib.h>
#include <MPU6050.h>
#include <avr/pgmspace.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
MPU6050 mpu;

#define PIN_EEG    A0
#define PIN_PULSE  A1
#define PIN_BUZZER 9

#define INTERVAL_SAMPLE    20ul
#define INTERVAL_CLASSIFY  2000ul
#define INTERVAL_DISPLAY   250ul
#define INTERVAL_TELEMETRY 1000ul
#define INTERVAL_UISWITCH  8000ul

#define FP_SCALE      256L
#define EMA_ALPHA_FP  26L

#define CYCLE_MS         5400000ul
#define REQUIRED_CYCLES  6
#define BEEP_FREQ        2000
#define BEEP_DUR_MS      500ul

#define STAGE_AWAKE  0
#define STAGE_LIGHT  1
#define STAGE_DEEP   2
#define STAGE_REM    3

const char s0[] PROGMEM = "AWAKE";
const char s1[] PROGMEM = "LIGHT";
const char s2[] PROGMEM = "DEEP ";
const char s3[] PROGMEM = "REM  ";
const char* const STAGE_TBL[] PROGMEM = { s0, s1, s2, s3 };

#define UI_HOME    0
#define UI_GRAPH   1
#define UI_NUMERIC 2
static uint8_t  uiMode    = UI_HOME;
static uint32_t tUISwitch = 0;

#define GRAPH_LEN 32
static uint8_t graphBuf[GRAPH_LEN];
static uint8_t graphHead = 0;

#define VAR_WIN 16
static int16_t varBuf[VAR_WIN];
static uint8_t varHead = 0;

#define IBI_WIN 8
static uint16_t ibiBuf[IBI_WIN];
static uint8_t  ibiHead = 0;

static int32_t  emaFP        = 0;
static uint32_t eegVariance  = 0;
static uint32_t hrvVariance  = 0;
static uint16_t bpm          = 0;
static uint32_t lastBeatMs   = 0;
static int16_t  ax, ay, az;
static int16_t  gx, gy, gz;
static uint32_t activityLevel = 0;

static uint8_t  sleepStage   = STAGE_AWAKE;
static bool     sleepStarted = false;
static uint32_t sleepStartMs = 0;
static uint8_t  cycleCount   = 0;
static uint32_t lastCycleMs  = 0;
static bool     alarmArmed   = false;
static bool     alarmFiring  = false;
static uint32_t alarmStartMs = 0;

static uint32_t tSample    = 0;
static uint32_t tClassify  = 0;
static uint32_t tDisplay   = 0;
static uint32_t tTelemetry = 0;

void bootBeep(void);
void bootAnimation(void);
void doSample(uint32_t now);
void doClassify(uint32_t now);
void doDisplay(uint32_t now);
void doTelemetry(uint32_t now);
void detectBeat(int16_t raw, uint32_t now);
void computeVariance(void);
void computeHRV(void);
void updateActivity(void);
void drawHome(void);
void drawGraph(void);
void drawNumeric(void);
void stageName(char* dst, uint8_t id);

void setup() {
    Serial.begin(115200);
    Serial.println(F("Timestamp_ms,EEG_Raw,Stage_ID,BPM,Accel_X,Accel_Y,Accel_Z"));

    Wire.begin();
    Wire.setClock(400000L);

    u8g2.begin();
    u8g2.setFont(u8g2_font_5x7_tf);

    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    pinMode(PIN_BUZZER, OUTPUT);
    noTone(PIN_BUZZER);

    bootBeep();
    bootAnimation();

    emaFP = (int32_t)analogRead(PIN_EEG) * FP_SCALE;

    memset(graphBuf, 0, sizeof(graphBuf));
    memset(varBuf,   0, sizeof(varBuf));
    memset(ibiBuf,   0, sizeof(ibiBuf));

    uint32_t now = millis();
    tSample = tClassify = tDisplay = tTelemetry = tUISwitch = now;
    lastCycleMs = now;
}

void loop() {
    uint32_t now = millis();

    if (now - tSample    >= INTERVAL_SAMPLE)    { tSample    = now; doSample(now);    }
    if (now - tClassify  >= INTERVAL_CLASSIFY)  { tClassify  = now; doClassify(now);  }
    if (now - tDisplay   >= INTERVAL_DISPLAY)   { tDisplay   = now; doDisplay(now);   }
    if (now - tTelemetry >= INTERVAL_TELEMETRY) { tTelemetry = now; doTelemetry(now); }
    if (now - tUISwitch  >= INTERVAL_UISWITCH)  { tUISwitch  = now; uiMode = (uiMode + 1) % 3; }

    if (alarmFiring && (now - alarmStartMs >= BEEP_DUR_MS)) {
        noTone(PIN_BUZZER);
        alarmFiring = false;
    }
    if (alarmArmed && !alarmFiring && sleepStage == STAGE_LIGHT) {
        tone(PIN_BUZZER, BEEP_FREQ);
        alarmFiring  = true;
        alarmStartMs = now;
        alarmArmed   = false;
    }
}

void doSample(uint32_t now) {
    int16_t eegRaw = analogRead(PIN_EEG);
    int32_t xFP = (int32_t)eegRaw * FP_SCALE;
    emaFP = emaFP + (EMA_ALPHA_FP * (xFP - emaFP) / FP_SCALE);
    int16_t emaVal = (int16_t)(emaFP / FP_SCALE);

    varBuf[varHead] = emaVal;
    varHead = (varHead + 1) % VAR_WIN;
    if (varHead == 0) computeVariance();

    graphBuf[graphHead] = (uint8_t)(((uint32_t)(emaVal) * 63UL) / 1023UL);
    graphHead = (graphHead + 1) % GRAPH_LEN;

    int16_t pulseRaw = analogRead(PIN_PULSE);
    detectBeat(pulseRaw, now);

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    updateActivity();
}

void detectBeat(int16_t raw, uint32_t now) {
    static bool above = false;
    if (!above && raw > 512) {
        above = true;
        if (lastBeatMs != 0) {
            uint32_t ibi = now - lastBeatMs;
            if (ibi > 300UL && ibi < 2000UL) {
                bpm = (uint16_t)(60000UL / ibi);
                ibiBuf[ibiHead] = (uint16_t)ibi;
                ibiHead = (ibiHead + 1) % IBI_WIN;
                if (ibiHead == 0) computeHRV();
            }
        }
        lastBeatMs = now;
    } else if (above && raw < 480) {
        above = false;
    }
}

void computeVariance(void) {
    int32_t sum = 0, sumSq = 0;
    for (uint8_t i = 0; i < VAR_WIN; i++) {
        int32_t v = varBuf[i];
        sum   += v;
        sumSq += v * v;
    }
    int32_t mean = sum / VAR_WIN;
    int32_t var  = sumSq / VAR_WIN - mean * mean;
    eegVariance  = (var > 0) ? (uint32_t)var : 0;
}

void computeHRV(void) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < IBI_WIN; i++) sum += ibiBuf[i];
    int32_t mean = sum / IBI_WIN;
    int32_t sq   = 0;
    for (uint8_t i = 0; i < IBI_WIN; i++) {
        int32_t d = (int32_t)ibiBuf[i] - mean;
        sq += d * d;
    }
    hrvVariance = (uint32_t)(sq / IBI_WIN);
}

void updateActivity(void) {
    static int16_t pax = 0, pay = 0, paz = 0;
    int32_t da = abs((int32_t)ax - pax)
               + abs((int32_t)ay - pay)
               + abs((int32_t)az - paz);
    activityLevel = (activityLevel + (uint32_t)da) >> 1;
    pax = ax; pay = ay; paz = az;
}

void doClassify(uint32_t now) {
    bool highMovement  = (activityLevel > 3000UL);
    bool highEEGVar    = (eegVariance   > 2000UL);
    bool lowMovement   = (activityLevel < 500UL);
    bool veryLowEEGVar = (eegVariance   < 400UL);
    bool modEEGVar     = (eegVariance  >= 400UL && eegVariance <= 2000UL);
    bool lowBPM        = (bpm > 0 && bpm < 65);
    bool highHRV       = (hrvVariance   > 800UL);

    uint8_t stage;
    if      (highMovement || highEEGVar)               stage = STAGE_AWAKE;
    else if (lowMovement && veryLowEEGVar && lowBPM)   stage = STAGE_DEEP;
    else if (lowMovement && highHRV && modEEGVar)       stage = STAGE_REM;
    else                                                stage = STAGE_LIGHT;

    if (!sleepStarted && stage != STAGE_AWAKE) {
        sleepStarted = true;
        sleepStartMs = now;
        lastCycleMs  = now;
    }
    if (sleepStarted && (now - lastCycleMs >= CYCLE_MS)) {
        cycleCount++;
        lastCycleMs = now;
    }

    sleepStage = stage;

    if (cycleCount >= REQUIRED_CYCLES && stage == STAGE_LIGHT && !alarmFiring) {
        alarmArmed = true;
    }
}

void doDisplay(uint32_t now) {
    (void)now;
    u8g2.firstPage();
    do {
        switch (uiMode) {
            case UI_HOME:    drawHome();    break;
            case UI_GRAPH:   drawGraph();   break;
            case UI_NUMERIC: drawNumeric(); break;
        }
    } while (u8g2.nextPage());
}

void drawHome(void) {
    char buf[10];
    char stgBuf[6];
    stageName(stgBuf, sleepStage);

    u8g2.setFont(u8g2_font_7x13B_tf);
    u8g2.drawStr(4, 13, F("BioHeadband"));
    u8g2.setFont(u8g2_font_5x7_tf);

    u8g2.drawStr(0, 26, F("Stage:"));
    u8g2.drawStr(44, 26, stgBuf);

    u8g2.drawStr(0, 37, F("BPM:"));
    itoa((int)bpm, buf, 10);
    u8g2.drawStr(30, 37, buf);

    u8g2.drawStr(0, 48, F("Activity:"));
    ltoa((long)(activityLevel >> 5), buf, 10);
    u8g2.drawStr(56, 48, buf);

    u8g2.drawStr(0, 59, F("Cycles:"));
    itoa(cycleCount, buf, 10);
    u8g2.drawStr(48, 59, buf);

    if (alarmArmed || alarmFiring) {
        u8g2.drawStr(90, 59, F("[ALM]"));
    }
}

void drawGraph(void) {
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 7, F("EEG Waveform"));
    u8g2.drawHLine(0, 9, 128);

    for (uint8_t i = 0; i < GRAPH_LEN; i++) {
        uint8_t idx = (graphHead + i) % GRAPH_LEN;
        uint8_t y = 63 - ((uint16_t)graphBuf[idx] * 54UL / 63UL) - 10;
        uint8_t x = (uint8_t)i * 4;
        u8g2.drawVLine(x, y, 2);
    }
}

void drawNumeric(void) {
    char buf[12];
    u8g2.setFont(u8g2_font_5x7_tf);

    int32_t ema10bit = emaFP / FP_SCALE;
    int32_t uV = ema10bit * 4882L / 1000L;

    u8g2.drawStr(0,  8, F("EEG uV:"));
    ltoa(uV, buf, 10);
    u8g2.drawStr(50, 8, buf);

    u8g2.drawStr(0, 17, F("Var:"));
    ltoa((long)eegVariance, buf, 10);
    u8g2.drawStr(28, 17, buf);

    u8g2.drawStr(0, 26, F("HRV ms2:"));
    ltoa((long)hrvVariance, buf, 10);
    u8g2.drawStr(52, 26, buf);

    u8g2.drawStr(0, 35, F("BPM:"));
    itoa((int)bpm, buf, 10);
    u8g2.drawStr(28, 35, buf);

    u8g2.drawStr(0, 44, F("AX:"));
    itoa(ax, buf, 10);
    u8g2.drawStr(20, 44, buf);

    u8g2.drawStr(0, 53, F("AY:"));
    itoa(ay, buf, 10);
    u8g2.drawStr(20, 53, buf);

    u8g2.drawStr(64, 44, F("AZ:"));
    itoa(az, buf, 10);
    u8g2.drawStr(84, 44, buf);

    char stgBuf[6];
    stageName(stgBuf, sleepStage);
    u8g2.drawStr(64, 53, stgBuf);
}

void doTelemetry(uint32_t now) {
    char buf[12];
    ltoa((long)now, buf, 10);
    Serial.print(buf); Serial.print(',');
    itoa((int)(emaFP / FP_SCALE), buf, 10);
    Serial.print(buf); Serial.print(',');
    Serial.print(sleepStage); Serial.print(',');
    Serial.print(bpm); Serial.print(',');
    itoa(ax, buf, 10); Serial.print(buf); Serial.print(',');
    itoa(ay, buf, 10); Serial.print(buf); Serial.print(',');
    itoa(az, buf, 10); Serial.println(buf);
}

void bootBeep(void) {
    for (uint8_t i = 0; i < 2; i++) {
        tone(PIN_BUZZER, BEEP_FREQ);
        delay(150);
        noTone(PIN_BUZZER);
        if (i == 0) delay(100);
    }
}

void bootAnimation(void) {
    for (uint8_t pct = 0; pct <= 100; pct += 4) {
        u8g2.firstPage();
        do {
            u8g2.setFont(u8g2_font_7x13B_tf);
            u8g2.drawStr(8, 22, F("BioHeadband"));
            u8g2.setFont(u8g2_font_5x7_tf);
            u8g2.drawStr(18, 36, F("Initializing..."));
            u8g2.drawFrame(4, 46, 120, 10);
            uint8_t fill = (uint8_t)((uint16_t)pct * 116UL / 100UL);
            if (fill) u8g2.drawBox(6, 48, fill, 6);
        } while (u8g2.nextPage());
        delay(25);
    }
    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_7x13B_tf);
        u8g2.drawStr(28, 36, F("READY!"));
    } while (u8g2.nextPage());
    delay(600);
}

void stageName(char* dst, uint8_t id) {
    if (id > 3) id = 0;
    strcpy_P(dst, (const char*)pgm_read_word(&STAGE_TBL[id]));
}
