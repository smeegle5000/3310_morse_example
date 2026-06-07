#include <HardwareSerial.h>
#include <U8g2lib.h>

HardwareSerial RF_SERIAL(PC11, PC10);

#define PIN_RF_ENABLE  PB9
#define PIN_RF_POWER   PC13
#define PIN_RF_PTT     PC0
#define PIN_AUDIO_IN   PA0

// ---- RF config ----
#define FREQ           433500000

// ---- Tone detect tuning ----
#define SAMPLES        32
#define TONE_THRESHOLD 120
#define DEBOUNCE_ON    0
#define DEBOUNCE_OFF   0
#define RESET_TIMEOUT_MS 3000

// ---- CW decoder tuning ----
#define EMA_ALPHA      0.10f
#define UNIT_SEED_MS   50.0f
#define DOT_DASH_THRESH 2.0f
#define INTRA_THRESH   2.0f
#define WORD_THRESH    5.0f
#define MAX_ELEMENTS   8
uint8_t REPS = 0;
// set this anywhere — 0 = wide open, 100 = very tight
uint8_t squelchLevel = 50;
uint8_t DISP_CENTER_POINT = 0;
unsigned long beeptime = 0;
int beephappened = 0;
#define DISP_PIN_SCK PB3
#define DISP_PIN_SDIN PB5
#define DISP_PIN_CS PB6
#define DISP_PIN_DC PB7
#define DISP_PIN_RST PB8

static char msgBuffer[128] = "";

const uint8_t KB_NUM_ROW = 5;
const uint8_t KB_NUM_COL = 3;
const uint8_t KB_ROW_PINS[] = {PA10, PA9, PA8, PC9, PC8};
const uint8_t KB_COL_PINS[] = {PB12, PC3, PC1};

U8G2_PCD8544_84X48_F_4W_SW_SPI u8g2(U8G2_R0, DISP_PIN_SCK, DISP_PIN_SDIN, DISP_PIN_CS, DISP_PIN_DC, DISP_PIN_RST);

// ---- Morse tables ----
struct MorseEntry { const char* code; char ch; };

// Letters A–Z
static const MorseEntry LETTERS[] = {
  {".-",   'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..",  'D'},
  {".",    'E'}, {"..-.", 'F'}, {"--.",  'G'}, {"....", 'H'},
  {"..",   'I'}, {".---", 'J'}, {"-.-",  'K'}, {".-..", 'L'},
  {"--",   'M'}, {"-.",   'N'}, {"---",  'O'}, {".--.", 'P'},
  {"--.-", 'Q'}, {".-.",  'R'}, {"...",  'S'}, {"-",    'T'},
  {"..-",  'U'}, {"...-", 'V'}, {".--",  'W'}, {"-..-", 'X'},
  {"-.--", 'Y'}, {"--..", 'Z'},
};

// Digits 0–9
static const MorseEntry DIGITS[] = {
  {"-----", '0'}, {".----", '1'}, {"..---", '2'}, {"...--", '3'},
  {"....-", '4'}, {".....", '5'}, {"-....", '6'}, {"--...", '7'},
  {"---..", '8'}, {"----.", '9'},
};

// ITU punctuation & prosigns
static const MorseEntry SYMBOLS[] = {
  {".-.-.-", '.'}, {"--..--", ','}, {"..--..", '?'}, {".----.", '\''},
  {"-.-.--", '!'}, {"-..-.",  '/'}, {"-.--.",  '('}, {"-.--.-", ')'},
  {".-...",  '&'}, {"---...",  ':'}, {"-.-.-.", ';'}, {"-...-",  '='},
  {".-.-.",  '+'}, {"-....-", '-'}, {"..--.-", '_'}, {".-..-.", '"'},
  {"...-..-",'$'}, {".--.-.", '@'}, {".-...",  '&'},
  // Prosigns (rendered as <XX>)
  {".-.-",   0},  // over — printed as <AR>
  {"...-.-", 0},  // end of work — <SK>
  {"-...-",  0},  // wait — <AS>  (also = sign, first match wins)
  {"-.-..",  0},  // understood — <KN>
};
static const char* PROSIGN_NAMES[] = {"<AR>","<SK>","<AS>","<KN>"};
static const int PROSIGN_OFFSET = 18;

// ---- RF helpers ----
void rfWrite(uint8_t reg, uint16_t val) {
  char buf[32];
  sprintf(buf, "AT+POKE=%d,%d\r\n", reg, val);
  RF_SERIAL.print(buf);
  delay(50);
  while(RF_SERIAL.available()) RF_SERIAL.read();
}

uint16_t rfRead(uint8_t reg) {
  char buf[32];
  sprintf(buf, "AT+PEEK=%d\r\n", reg);
  RF_SERIAL.print(buf);
  delay(50);
  String resp = RF_SERIAL.readStringUntil('\n');
  return (uint16_t)resp.toInt();
}

void reloadRF() {
  uint16_t val = rfRead(48);
  rfWrite(48, val & ~96);
  rfWrite(48, val);
}

void setFreq(uint32_t freq_hz) {
  uint32_t val = (uint32_t)((freq_hz / 1000.0f) * 16.0f);
  rfWrite(41, (val >> 16) & 0xFFFF);
  rfWrite(42, val & 0xFFFF);
  reloadRF();
}

void initRF() {
  rfWrite(48, 1); delay(160);
  rfWrite(48, 4);
  rfWrite(4, 4048);   rfWrite(10, 31776); rfWrite(19, 41216);
  rfWrite(31, 4097);  rfWrite(49, 49);    rfWrite(51, 17573);
  rfWrite(52, 11145); rfWrite(65, 16674); rfWrite(66, 4178);
  rfWrite(67, 256);   rfWrite(68, 2047);  rfWrite(89, 4368);
  rfWrite(71, 32559); rfWrite(79, 11362); rfWrite(83, 148);
  rfWrite(84, 10812); rfWrite(85, 129);   rfWrite(86, 2818);
  rfWrite(87, 7168);  rfWrite(88, 40157); rfWrite(90, 1755);
  rfWrite(99, 5805);  rfWrite(103, 1576); rfWrite(104, 1509);
  rfWrite(105, 1365); rfWrite(106, 1208); rfWrite(107, 766);
  rfWrite(108, 477);  rfWrite(109, 177);  rfWrite(110, 3970);
  rfWrite(111, 378);  rfWrite(112, 76);   rfWrite(113, 3869);
  rfWrite(114, 3473); rfWrite(115, 2622); rfWrite(116, 2319);
  rfWrite(117, 2099); rfWrite(118, 2054);
  rfWrite(48, 16548); delay(160);
  rfWrite(48, 16550); delay(160);
  rfWrite(48, 16390); delay(160);
  rfWrite(64, 49);
  rfWrite(21, 7936);  rfWrite(50, 30052); rfWrite(58, 16579);
  rfWrite(60, 6964);  rfWrite(63, 10705); rfWrite(72, 3939);
  rfWrite(98, 65454);
  rfWrite(127, 1);
  rfWrite(6, 36);   rfWrite(7, 532);   rfWrite(8, 548);
  rfWrite(9, 788);  rfWrite(10, 804);  rfWrite(11, 836);
  rfWrite(12, 900); rfWrite(13, 4996); rfWrite(14, 7044);
  rfWrite(15, 16260); rfWrite(18, 57579);
  rfWrite(127, 0);
  reloadRF();
  setFreq(FREQ);

  uint16_t r = rfRead(48);
  r = (r & ~96) | 32;
  r = r | 8;
  rfWrite(48, r);
  rfWrite(73, 3478);
  rfWrite(44, 255);
  rfWrite(58, 33021);
}

// ---- Tone detect ----
bool tonePresent = false;



void setSquelch(uint8_t level) {
  // level 0   = most sensitive (-130/-135 dBm) → open on almost anything
  // level 100 = least sensitive (-70/-75 dBm)  → only strong signals
  // linear map: level 0→open=-130, level 100→open=-70
  int8_t openDbm  = -130 + (int8_t)((60.0f / 100.0f) * level);
  int8_t closeDbm = openDbm - 5; // close threshold always 5dB below open

  uint8_t openVal  = (uint8_t)(137 + openDbm);
  uint8_t closeVal = (uint8_t)(137 + closeDbm);

  uint16_t reg73 = ((uint16_t)openVal << 7) | closeVal;
  rfWrite(73, reg73);
}

void updateToneDetect() {
  static unsigned long edgeTime = 0;
  static bool rawTone = false;

  int mn = 4095, mx = 0;
  for (int i = 0; i < SAMPLES; i++) {
    int v = analogRead(PIN_AUDIO_IN);
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  bool above = (mx - mn) > TONE_THRESHOLD;

  if (above != rawTone) {
    rawTone = above;
    edgeTime = millis();
  }

  unsigned long debounce = tonePresent ? DEBOUNCE_OFF : DEBOUNCE_ON;
  if (rawTone != tonePresent && (millis() - edgeTime) >= debounce) {
    tonePresent = rawTone;
  }
}

// ---- CW decoder ----
static float    unitMs    = UNIT_SEED_MS;
static char     elements[MAX_ELEMENTS + 1];
static int      elemCount = 0;
static bool     lastTone  = false;
static uint32_t keyStart  = 0;
static uint32_t gapStart  = 0;
static bool     haveElement = false;

void updateUnit(float ms) {
  unitMs = EMA_ALPHA * ms + (1.0f - EMA_ALPHA) * unitMs;
}

char classifyElement(uint32_t ms) {
  float d = (float)ms;
  if (d < unitMs * DOT_DASH_THRESH) { updateUnit(d);        return '.'; }
  else                               { updateUnit(d / 3.0f); return '-'; }
}

void resetDecoder() {
  elemCount   = 0;
  haveElement = false;
  unitMs      = UNIT_SEED_MS;
  keyStart    = 0;
  gapStart    = 0;
  lastTone    = false;
  memset(elements, 0, sizeof(elements));
}

char decodeBuffer() {
  if (elemCount == 0) return 0;
  elements[elemCount] = '\0';
  for (auto& e : LETTERS) if (strcmp(elements, e.code) == 0) return e.ch;
  for (auto& e : DIGITS)  if (strcmp(elements, e.code) == 0) return e.ch;
  for (int i = 0; i < (int)(sizeof(SYMBOLS)/sizeof(SYMBOLS[0])); i++) {
    if (strcmp(elements, SYMBOLS[i].code) == 0) {
      if (SYMBOLS[i].ch != 0) return SYMBOLS[i].ch;
      Serial.print(PROSIGN_NAMES[i - PROSIGN_OFFSET]);
      return 0;
    }
  }
  Serial.print('['); Serial.print(elements); Serial.print(']');
  return 0;
}

void flushChar() {
  if (elemCount == 0) return;
  char ch = decodeBuffer();
  if (ch != 0) {
    Serial.print(ch);
    size_t len = strlen(msgBuffer);
    if (len < sizeof(msgBuffer) - 1) {
      msgBuffer[len]   = ch;
      msgBuffer[len+1] = '\0';
    }
  }
  elemCount   = 0;
  haveElement = false;
}

void updateDecoder() {
  uint32_t now = millis();

  if (tonePresent && !lastTone) {
    // key down
    keyStart = now;
    if (haveElement && gapStart > 0) {
      uint32_t gapMs = now - gapStart;
      if      ((float)gapMs >= unitMs * WORD_THRESH)  { flushChar(); Serial.print(' '); }
      else if ((float)gapMs >= unitMs * INTRA_THRESH) { flushChar(); }
    }
  }

  if (!tonePresent && lastTone) {
    // key up
    uint32_t dur = now - keyStart;
    if (dur >= (uint32_t)(unitMs * 0.5f)) {
      char el = classifyElement(dur);
      if (elemCount < MAX_ELEMENTS) elements[elemCount++] = el;
      if (elemCount >= MAX_ELEMENTS) {resetDecoder(); return;}
      haveElement = true;
    }
    gapStart = now;
  }

  // timeout flush
  if (!tonePresent && haveElement && gapStart > 0) {
    static uint32_t lastSpacePrint = 0;
    uint32_t idleMs = now - gapStart;
    if ((float)idleMs >= unitMs * WORD_THRESH) {
      flushChar();
      if (gapStart != lastSpacePrint) {
        Serial.print(' ');
        size_t len = strlen(msgBuffer);
        if (len < sizeof(msgBuffer) - 1) {
          msgBuffer[len]   = ' ';
          msgBuffer[len+1] = '\0';
        }
        lastSpacePrint = gapStart;
      }
    }
    if (!tonePresent && gapStart > 0) {
      uint32_t idleMs = millis() - gapStart;
      if (idleMs > 3000) {  // 3 seconds no signal
        flushChar();
        resetDecoder();
      }
    } 
  }

  lastTone = tonePresent;
}

// ---- setup ----
void setup() {
    for (uint8_t i = 0; i < KB_NUM_ROW; i++) {  //increment through all row pins, define as input pins, hardware pull down resistors are in place
    pinMode(KB_ROW_PINS[i], INPUT);
  }
  for (uint8_t j = 0; j < KB_NUM_COL; j++) {  //increment through all col pins, define as output pins, and set to high
    pinMode(KB_COL_PINS[j], OUTPUT);
  }
  digitalWrite(PC3, HIGH);  //manually trigger the row for the * button
  u8g2.begin();
  u8g2.setContrast(110);
  u8g2.setFontMode(1);
  u8g2.setBitmapMode(1);
  u8g2.setFont(u8g2_font_NokiaLargeBold_tr);
  DISP_CENTER_POINT = u8g2_GetStrWidth(u8g2.getU8g2(), "setting up");    //get the offset of the text to centre it on the screen
  u8g2.setCursor(42 - (DISP_CENTER_POINT / 2), 23);                         //42 = display horizontal midpoint, 23 = vertical point on screen i want to draw at
  u8g2.print("setting up");
  u8g2.sendBuffer();
  u8g2.clearBuffer();
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(PIN_AUDIO_IN, INPUT_ANALOG);

  pinMode(PIN_RF_ENABLE, OUTPUT);
  pinMode(PIN_RF_POWER, OUTPUT);
  pinMode(PIN_RF_PTT, OUTPUT);

  digitalWrite(PIN_RF_PTT, HIGH);
  digitalWrite(PIN_RF_POWER, LOW);
  digitalWrite(PIN_RF_ENABLE, LOW);
  delay(100);
  digitalWrite(PIN_RF_ENABLE, HIGH);
  delay(100);

  RF_SERIAL.begin(9600);
  delay(100);

  initRF();
  Serial.println("CW RX ready.");
}

// ---- loop ----
void loop() {
  updateToneDetect();
  updateDecoder();
  //if (tonePresent == true && beephappened == 0) beephappened = 1, beeptime = millis();
  //if (tonePresent == false && beephappened == 1) beephappened = 0, Serial.print("beep length:"), Serial.println(millis() - beeptime);
  if (REPS == 100){
    if (digitalRead(PA9) == HIGH) squelchLevel++, setSquelch(squelchLevel);
    if (digitalRead(PA8) == HIGH) squelchLevel--, setSquelch(squelchLevel);
    u8g2.clearBuffer();
   // Serial.print("ADC:");
    //Serial.println(analogRead(PIN_AUDIO_IN));
    u8g2.setCursor(42,10);
    u8g2.setFont(u8g2_font_NokiaSmallPlain_tr);
    u8g2.print(squelchLevel);
    if (DISP_CENTER_POINT < 0) msgBuffer[0] = '\0';
    DISP_CENTER_POINT = u8g2_GetStrWidth(u8g2.getU8g2(), msgBuffer);    //get the offset of the text to centre it on the screen
    u8g2.setCursor(80 - (DISP_CENTER_POINT), 23);                         //42 = display horizontal midpoint, 23 = vertical point on screen i want to draw at
    u8g2.print(msgBuffer);
    u8g2.sendBuffer();
    REPS = 0;
  } else {
    REPS++;
  }
}