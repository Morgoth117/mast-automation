#include <Wire.h>
#include <U8g2lib.h>
#include <EEPROM.h>

// ---- OLED (I2C, 128x32) ----
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ----- Pins -----
#define STEP_PIN 8
#define DIR_PIN  9
#define EN_PIN   10

#define ENC_A 2
#define ENC_B 3
#define BTN   4

// ----- Motion constants -----
const float MM_PER_STEP = 0.01;   // TR8x2 lead screw, full step (2mm / 200 = 0.01mm)
volatile long stepPosition = 0;
volatile int lastEncA = 0;
volatile uint32_t lastEncChangeUs = 0;

bool enabled = false;

// ----- UI state -----
enum UiMode : uint8_t {
  MODE_RUN = 0,
  MODE_MENU,
  MODE_SAVE_LETTER,
  MODE_SAVE_NUMBER,
  MODE_LOAD_LIST,
  MODE_MOVING
};

volatile UiMode mode = MODE_RUN;

// ----- Step pulse -----
const uint8_t STEP_PULSE_HIGH_US = 3;  // DRV8825 min 1.9us
const uint8_t STEP_PULSE_LOW_US = 3;   // DRV8825 min 1.9us

inline void doStep() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_HIGH_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_PULSE_LOW_US);
}

void setDriverEnabled(bool isEnabled) {
  enabled = isEnabled;
  digitalWrite(EN_PIN, enabled ? LOW : HIGH);
}

// Go-to speed control: smaller = faster (but clamped for reliability)
const uint16_t MOVE_STEP_DELAY_US = 1000;  // slow, known-good
const uint16_t MIN_MOVE_STEP_DELAY_US = 400;  // clamp to avoid squeal/stall

// How often to refresh the OLED during GO-TO (ms). Lower = smoother but slower.
const uint16_t MOVING_UI_REFRESH_MS = 100;

// ---------- EEPROM slot system (with validity marker) ----------
// Slots: P1..P9, G1..G9, C1..C9 => 27 slots
const char SLOT_LETTERS[3] = {'P','G','C'};
const uint8_t LETTER_COUNT = 3;
const uint8_t NUM_PER_LETTER = 9;

struct __attribute__((packed)) SlotData {
  uint8_t sig;   // 0xA5 means "valid"
  long pos;      // stepPosition
};

const uint8_t SLOT_SIG = 0xA5;

uint8_t slotIndex(uint8_t letterIdx, uint8_t number1to9) {
  return (letterIdx * NUM_PER_LETTER) + (number1to9 - 1); // 0..26
}

int slotAddr(uint8_t idx) {
  return idx * (int)sizeof(SlotData);
}

bool slotExists(uint8_t idx) {
  SlotData d;
  EEPROM.get(slotAddr(idx), d);
  return (d.sig == SLOT_SIG);
}

long slotReadPos(uint8_t idx) {
  SlotData d;
  EEPROM.get(slotAddr(idx), d);
  return d.pos;
}

void slotWritePos(uint8_t idx, long pos) {
  SlotData d;
  d.sig = SLOT_SIG;
  d.pos = pos;
  EEPROM.put(slotAddr(idx), d);
}

// ---------- Persistent "last position" record ----------
struct __attribute__((packed)) LastPosData {
  uint8_t sig;   // 0x5A means "valid last pos"
  long pos;
};

const uint8_t LAST_SIG = 0x5A;

// Put this AFTER slot storage. With packed structs this is safe & compact.
const int LASTPOS_ADDR = (int)(27 * sizeof(SlotData));

void saveLastPositionNow(long p) {
  LastPosData cur;
  EEPROM.get(LASTPOS_ADDR, cur);

  // Only write if changed (reduces wear)
  if (cur.sig == LAST_SIG && cur.pos == p) return;

  LastPosData out;
  out.sig = LAST_SIG;
  out.pos = p;
  EEPROM.put(LASTPOS_ADDR, out);
}

bool loadLastPosition(long &outPos) {
  LastPosData d;
  EEPROM.get(LASTPOS_ADDR, d);
  if (d.sig != LAST_SIG) return false;
  outPos = d.pos;
  return true;
}

// Periodic autosave (low write rate)
const uint32_t AUTOSAVE_MS = 5000;
uint32_t lastAutoSaveMs = 0;
long lastSavedPos = 0;

// ---------- Menu ----------
const char* MENU_ITEMS[] = {"Resume", "Save Point", "Load Point", "Zero Cal"};
const uint8_t MENU_COUNT = sizeof(MENU_ITEMS) / sizeof(MENU_ITEMS[0]);
uint8_t menuIndex = 0;

// ---------- Save selection ----------
uint8_t saveLetterIdx = 0; // 0=P,1=G,2=C
uint8_t saveNumber = 1;    // 1..9
bool saveCancelSelected = false; // for letter screen

// ---------- Load list of existing saves ----------
struct SaveLabel {
  char letter;
  uint8_t number;
  uint8_t idx;   // slot idx 0..26
};

SaveLabel loadList[27];
uint8_t loadCount = 0;
int8_t loadIndex = 0; // selection index in loadList
bool loadCancelItem = false;

// ---------- Moving target ----------
volatile long targetPosition = 0;
volatile bool moveCancelRequested = false;

// Encoder navigation delta (when in menu modes)
volatile int8_t uiDelta = 0;

// ----- Encoder ISR (DUMB / OG stepping) -----
// - In MODE_RUN: if enabled, steps motor exactly like OG.
// - In any other mode: DOES NOT step motor; only uiDelta for scrolling.
void encoderISR() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);

  if (a != lastEncA) {
    if (mode == MODE_RUN) {
      if (enabled) {
        if (a == b) {
          stepPosition++;
          digitalWrite(DIR_PIN, HIGH);
        } else {
          stepPosition--;
          digitalWrite(DIR_PIN, LOW);
        }
        doStep();
      }
    } else {
      uiDelta += (a == b) ? 1 : -1;
    }
  }

  lastEncA = a;
}

// ----- Build load list (existing saves only) -----
void rebuildLoadList() {
  loadCount = 0;
  for (uint8_t li = 0; li < LETTER_COUNT; li++) {
    for (uint8_t n = 1; n <= NUM_PER_LETTER; n++) {
      uint8_t idx = slotIndex(li, n);
      if (slotExists(idx)) {
        loadList[loadCount].letter = SLOT_LETTERS[li];
        loadList[loadCount].number = n;
        loadList[loadCount].idx = idx;
        loadCount++;
      }
    }
  }
  loadIndex = 0;
  loadCancelItem = false;
}

// ----- Button handling (short vs long press) -----
const uint16_t DEBOUNCE_MS  = 30;
const uint16_t LONGPRESS_MS = 650;

bool btnLastStable = HIGH;
bool btnLastRead   = HIGH;
uint32_t btnLastChangeMs = 0;

bool btnIsDown = false;
uint32_t btnDownMs = 0;
bool longPressFired = false;

bool readButtonEvent(bool &shortPress, bool &longPress) {
  shortPress = false;
  longPress  = false;

  bool btnRead = digitalRead(BTN);

  if (btnRead != btnLastRead) {
    btnLastChangeMs = millis();
    btnLastRead = btnRead;
  }

  bool btnStable = btnLastStable;
  if ((millis() - btnLastChangeMs) > DEBOUNCE_MS) {
    btnStable = btnRead;
  }

  if (btnLastStable == HIGH && btnStable == LOW) {
    btnIsDown = true;
    btnDownMs = millis();
    longPressFired = false;
  }

  if (btnIsDown && !longPressFired && btnStable == LOW) {
    if ((millis() - btnDownMs) >= LONGPRESS_MS) {
      longPressFired = true;
      longPress = true;
    }
  }

  if (btnLastStable == LOW && btnStable == HIGH) {
    btnIsDown = false;
    if (!longPressFired) shortPress = true;
  }

  btnLastStable = btnStable;
  return true;
}

// ----- Drawing helpers -----
void drawRun() {
  float mm = stepPosition * MM_PER_STEP;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Quill Height:");

  char buf[24];
  dtostrf(mm, 0, 2, buf);
  char line2[32];
  snprintf(line2, sizeof(line2), "%s mm", buf);
  u8g2.drawStr(0, 24, line2);

  u8g2.drawStr(0, 32, enabled ? "ENABLED" : "DISABLED");
  u8g2.sendBuffer();
}

void drawMenu() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "MENU");

  uint8_t top = (menuIndex > 0) ? (menuIndex - 1) : 0;

  for (uint8_t i = 0; i < 2; i++) {
    uint8_t item = top + i;
    if (item >= MENU_COUNT) break;
    int y = 22 + (i * 10);
    if (item == menuIndex) u8g2.drawStr(0, y, ">");
    u8g2.drawStr(10, y, MENU_ITEMS[item]);
  }

  u8g2.sendBuffer();
}

void drawSaveLetter() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Save Point");
  u8g2.drawStr(0, 22, "Pick Letter:");

  u8g2.drawStr(0, 32, "  P   G   C   X");

  int xCaret = saveCancelSelected ? (10 + (3 * 24)) : (10 + (saveLetterIdx * 24));
  u8g2.drawStr(xCaret, 30, "^");

  u8g2.sendBuffer();
}

void drawSaveNumber() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Save Point");

  char line1[32];
  snprintf(line1, sizeof(line1), "Slot: %c%u", SLOT_LETTERS[saveLetterIdx], saveNumber);
  u8g2.drawStr(0, 22, line1);

  u8g2.drawStr(0, 32, "Btn=Save  (LP=Cancel)");
  u8g2.sendBuffer();
}

void drawLoadList() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Load Point");

  if (loadCount == 0) {
    u8g2.drawStr(0, 24, "No saves found.");
    u8g2.drawStr(0, 32, "> Cancel");
    u8g2.sendBuffer();
    return;
  }

  int totalItems = loadCount + 1; // +Cancel
  int sel = loadCancelItem ? loadCount : loadIndex;

  int top = sel;
  if (top > 0) top -= 1;
  if (top > totalItems - 2) top = totalItems - 2;
  if (top < 0) top = 0;

  for (int i = 0; i < 2; i++) {
    int item = top + i;
    int y = 22 + (i * 10);

    if (item == sel) u8g2.drawStr(0, y, ">");

    if (item < (int)loadCount) {
      char label[16];
      snprintf(label, sizeof(label), "%c%u", loadList[item].letter, loadList[item].number);
      u8g2.drawStr(10, y, label);
    } else {
      u8g2.drawStr(10, y, "Cancel");
    }
  }

  u8g2.sendBuffer();
}

void drawMoving() {
  float mmNow = stepPosition * MM_PER_STEP;
  float mmTgt = targetPosition * MM_PER_STEP;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Moving...");

  char line1[32], a[16], b[16];
  dtostrf(mmNow, 0, 2, a);
  dtostrf(mmTgt, 0, 2, b);
  snprintf(line1, sizeof(line1), "%s -> %s mm", a, b);
  u8g2.drawStr(0, 24, line1);

  u8g2.drawStr(0, 32, "LP=Cancel");
  u8g2.sendBuffer();
}

// ----- Move routine (blocking, but cancelable) -----
// Throttles OLED refresh so speed changes actually matter.
void moveToTarget(long tgt) {
  targetPosition = tgt;
  moveCancelRequested = false;
  mode = MODE_MOVING;

  uint32_t lastUiMs = 0;

  while (stepPosition != targetPosition) {
    bool sp=false, lp=false;
    readButtonEvent(sp, lp);
    if (lp) { moveCancelRequested = true; break; }

    if (stepPosition < targetPosition) {
      digitalWrite(DIR_PIN, HIGH);
      stepPosition++;
    } else {
      digitalWrite(DIR_PIN, LOW);
      stepPosition--;
    }

    doStep();
    uint16_t stepDelay = MOVE_STEP_DELAY_US;
    if (stepDelay < MIN_MOVE_STEP_DELAY_US) stepDelay = MIN_MOVE_STEP_DELAY_US;
    delayMicroseconds(stepDelay);

    // Refresh UI occasionally (NOT every step)
    uint32_t now = millis();
    if ((now - lastUiMs) >= MOVING_UI_REFRESH_MS) {
      drawMoving();
      lastUiMs = now;
    }
  }

  // Final draw when done/canceled
  drawMoving();

  // Save last-known position after a move attempt (even if canceled, it reflects reality)
  saveLastPositionNow(stepPosition);
  lastSavedPos = stepPosition;
  lastAutoSaveMs = millis();
}

// ----- Setup -----
void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(BTN,   INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  lastEncA = digitalRead(ENC_A);
  setDriverEnabled(false);

  // Restore last-known position (if available)
  long p;
  if (loadLastPosition(p)) {
    stepPosition = p;
    lastSavedPos = p;
  } else {
    lastSavedPos = stepPosition;
  }

  lastAutoSaveMs = millis();
}

// ----- Loop -----
void loop() {
  // consume uiDelta
  int8_t d = 0;
  noInterrupts();
  d = uiDelta;
  uiDelta = 0;
  interrupts();

  // button events
  bool shortPress=false, longPress=false;
  readButtonEvent(shortPress, longPress);

  // LONG PRESS:
  // - In RUN: open menu
  // - In menus: cancel/back to menu
  if (longPress) {
    if (mode == MODE_RUN) {
      mode = MODE_MENU;
      menuIndex = 0;
    } else if (mode != MODE_MOVING) {
      mode = MODE_MENU;
    }
  }

  // Handle encoder navigation in UI modes (not moving)
  if (mode != MODE_RUN && mode != MODE_MOVING && d != 0) {
    if (mode == MODE_MENU) {
      int16_t idx = (int16_t)menuIndex + d;
      if (idx < 0) idx = 0;
      if (idx >= (int16_t)MENU_COUNT) idx = MENU_COUNT - 1;
      menuIndex = (uint8_t)idx;
    } else if (mode == MODE_SAVE_LETTER) {
      int16_t sel = saveCancelSelected ? 3 : saveLetterIdx; // 0..3
      sel += d;
      if (sel < 0) sel = 0;
      if (sel > 3) sel = 3;

      if (sel == 3) saveCancelSelected = true;
      else { saveCancelSelected = false; saveLetterIdx = (uint8_t)sel; }
    } else if (mode == MODE_SAVE_NUMBER) {
      int16_t n = (int16_t)saveNumber + d;
      if (n < 1) n = 1;
      if (n > 9) n = 9;
      saveNumber = (uint8_t)n;
    } else if (mode == MODE_LOAD_LIST) {
      if (loadCount == 0) {
        loadCancelItem = true;
      } else {
        int totalItems = loadCount + 1;
        int sel = loadCancelItem ? loadCount : loadIndex;
        sel += d;
        if (sel < 0) sel = 0;
        if (sel > totalItems - 1) sel = totalItems - 1;

        if (sel == loadCount) loadCancelItem = true;
        else { loadCancelItem = false; loadIndex = (int8_t)sel; }
      }
    }
  }

  // SHORT PRESS actions
  if (shortPress) {
    if (mode == MODE_RUN) {
      setDriverEnabled(!enabled);
    } else if (mode == MODE_MENU) {
      switch (menuIndex) {
        case 0: // Resume
          mode = MODE_RUN;
          break;

        case 1: // Save Point
          saveLetterIdx = 0;
          saveNumber = 1;
          saveCancelSelected = false;
          mode = MODE_SAVE_LETTER;
          break;

        case 2: // Load Point
          rebuildLoadList();
          loadCancelItem = false;
          loadIndex = 0;
          mode = MODE_LOAD_LIST;
          break;

        case 3: // Zero Cal
          stepPosition = 0;
          saveLastPositionNow(stepPosition);
          lastSavedPos = stepPosition;
          lastAutoSaveMs = millis();
          mode = MODE_RUN;
          break;
      }
    } else if (mode == MODE_SAVE_LETTER) {
      if (saveCancelSelected) mode = MODE_MENU;
      else mode = MODE_SAVE_NUMBER;
    } else if (mode == MODE_SAVE_NUMBER) {
      uint8_t idx = slotIndex(saveLetterIdx, saveNumber);
      slotWritePos(idx, stepPosition);
      saveLastPositionNow(stepPosition);
      lastSavedPos = stepPosition;
      lastAutoSaveMs = millis();
      mode = MODE_MENU;
    } else if (mode == MODE_LOAD_LIST) {
      if (loadCount == 0) {
        mode = MODE_MENU;
      } else if (loadCancelItem) {
        mode = MODE_MENU;
      } else {
        uint8_t idx = loadList[loadIndex].idx;
        long tgt = slotReadPos(idx);
        moveToTarget(tgt);
        mode = MODE_RUN;
      }
    }
  }

  // Autosave last position (RUN only), low write-rate
  if (mode == MODE_RUN) {
    uint32_t now = millis();
    if ((now - lastAutoSaveMs) >= AUTOSAVE_MS) {
      long p = stepPosition;
      if (p != lastSavedPos) {
        saveLastPositionNow(p);
        lastSavedPos = p;
      }
      lastAutoSaveMs = now;
    }
  }

  // Draw screen (don’t fight the moving screen)
  UiMode m;
  noInterrupts();
  m = mode;
  interrupts();

  if (m == MODE_RUN) drawRun();
  else if (m == MODE_MENU) drawMenu();
  else if (m == MODE_SAVE_LETTER) drawSaveLetter();
  else if (m == MODE_SAVE_NUMBER) drawSaveNumber();
  else if (m == MODE_LOAD_LIST) drawLoadList();
  else if (m == MODE_MOVING) {/* drawMoving handled in move loop */}

  delay(50);
}
