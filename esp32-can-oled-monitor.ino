// ESP32 + TJA1050: считыватель меню alight по протоколу LEVCAN.
//
// v3 — корректная сборка multi-frame через CTS, как в levcan.c::objectRXproceed.
//
// Когда alight шлёт ответ длиной >8 байт, он использует TCP-режим:
//   1) RTS=1, EoM=0, Parity=0 — первые 8 байт
//   2) ждёт от нас CTS-кадр (Request=1, RTS=1, EoM=0, Parity=expected, length=0)
//   3) RTS=0, EoM=0/1, Parity=... — следующие байты
//   4) при EoM=1 мы шлём финальный CTS (RTS=0, EoM=1, Parity=last)
//
// step_inc=8, parity = ~((position + 7) / 8) & 1.
//
// Пины:
//   OLED: SDA=GPIO5, SCL=GPIO4
//   CAN:  TX=GPIO25, RX=GPIO26  → TJA1050

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

extern "C" {
  #include "driver/twai.h"
}

// ===== OLED =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_SDA 5
#define OLED_SCL 4

// ===== CAN =====
#define CAN_TX GPIO_NUM_25
#define CAN_RX GPIO_NUM_26

// ===== LEVCAN =====
static const uint8_t  MY_ADDR     = 100;
static const uint8_t  TARGET_ADDR = 98;

static const uint16_t MSG_PARAM_REQUEST     = 0x399;
static const uint16_t MSG_PARAM_DATA        = 0x39A;
static const uint16_t MSG_PARAM_DESCRIPTOR  = 0x39B;
static const uint16_t MSG_PARAM_NAME        = 0x39C;
static const uint16_t MSG_PARAM_TEXT        = 0x39D;
static const uint16_t MSG_PARAM_VALUE       = 0x39E;

static const uint16_t LCP_REQ_FULL_ENTRY     = 0x1F;
static const uint16_t LCP_REQ_DIRECTORY_INFO = 0x20;

enum LCP_Type {
  LCP_Folder = 0, LCP_Label, LCP_Bool, LCP_Enum, LCP_Bitfield32,
  LCP_Int32, LCP_Uint32, LCP_Int64, LCP_Uint64, LCP_Float,
  LCP_Double, LCP_Decimal32, LCP_String, LCP_End = 0xFF
};

// ===== Пределы =====
#define MAX_DIRS         16
#define MAX_ENTRIES_DIR  16
#define NAME_BUF         48
#define TEXT_BUF         64
#define DESC_BUF         32
#define ASM_BUF          80   // буфер сборки одного канала

// ===== Логирование =====
static bool VERBOSE_RX = true;
static bool VERBOSE_CTS = true;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===== Хранилище меню =====
struct EntryInfo {
  uint8_t  type;
  uint8_t  mode;
  uint16_t entryIdx;
  uint16_t varSize;
  uint16_t descSize;
  uint16_t textSize;
  bool     hasData, hasName, hasDesc, hasText, hasValue;
  uint16_t childDir;       // для Folder
  char     name[NAME_BUF];
  char     text[TEXT_BUF];
  uint8_t  desc[DESC_BUF]; uint8_t descLen;
  uint8_t  value[16];      uint8_t valueLen;
};

struct DirInfo {
  bool     known;
  uint16_t entrySize;
  uint16_t nameSize;
  bool     fullyRead;
  char     name[NAME_BUF];
  EntryInfo entries[MAX_ENTRIES_DIR];
};

DirInfo dirs[MAX_DIRS];
uint8_t totalDirsKnown = 1;

// ===== ID pack/unpack =====
struct LcHeader {
  uint8_t  source, target;
  uint16_t msgId;
  uint8_t  eom, parity, rts, prio;
};

static uint32_t packId(const LcHeader& h) {
  uint32_t id = 0;
  id |= (uint32_t)(h.source & 0x7F);
  id |= (uint32_t)(h.target & 0x7F) << 7;
  id |= (uint32_t)(h.msgId  & 0x3FF) << 14;
  id |= (uint32_t)(h.eom    & 0x1)   << 24;
  id |= (uint32_t)(h.parity & 0x1)   << 25;
  id |= (uint32_t)(h.rts    & 0x1)   << 26;
  id |= (uint32_t)(h.prio   & 0x3)   << 27;
  return id;
}

static LcHeader unpackId(uint32_t id) {
  LcHeader h;
  h.source = id & 0x7F;
  h.target = (id >> 7) & 0x7F;
  h.msgId  = (id >> 14) & 0x3FF;
  h.eom    = (id >> 24) & 0x1;
  h.parity = (id >> 25) & 0x1;
  h.rts    = (id >> 26) & 0x1;
  h.prio   = (id >> 27) & 0x3;
  return h;
}

// ===== TWAI =====
bool initCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 16;
  g_config.rx_queue_len = 64;
  g_config.alerts_enabled = TWAI_ALERT_RX_DATA | TWAI_ALERT_BUS_ERROR
                          | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF
                          | TWAI_ALERT_TX_FAILED;

  // 1 Mbit/s, SP=87.5% — клон BTR alight'а
  twai_timing_config_t t_config = {
    .brp = 10, .tseg_1 = 6, .tseg_2 = 1, .sjw = 2, .triple_sampling = false
  };
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  return true;
}

bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len, bool rtr = false) {
  twai_message_t m = {};
  m.identifier = id; m.extd = 1; m.rtr = rtr ? 1 : 0;
  m.data_length_code = len;
  for (uint8_t i = 0; i < len && i < 8; i++) m.data[i] = data[i];
  return twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK;
}

bool reqDir(uint16_t dirIndex) {
  LcHeader h{}; h.source=MY_ADDR; h.target=TARGET_ADDR; h.msgId=MSG_PARAM_REQUEST;
  h.eom=1; h.parity=1; h.rts=1; h.prio=0;
  uint8_t pl[4] = {
    (uint8_t)(LCP_REQ_DIRECTORY_INFO & 0xFF), (uint8_t)(LCP_REQ_DIRECTORY_INFO >> 8),
    (uint8_t)(dirIndex & 0xFF), (uint8_t)(dirIndex >> 8)
  };
  return sendFrame(packId(h), pl, 4);
}

bool reqEntry(uint16_t dirIndex, uint16_t entryIndex) {
  LcHeader h{}; h.source=MY_ADDR; h.target=TARGET_ADDR; h.msgId=MSG_PARAM_REQUEST;
  h.eom=1; h.parity=1; h.rts=1; h.prio=0;
  uint8_t pl[6] = {
    (uint8_t)(LCP_REQ_FULL_ENTRY & 0xFF), (uint8_t)(LCP_REQ_FULL_ENTRY >> 8),
    (uint8_t)(dirIndex & 0xFF), (uint8_t)(dirIndex >> 8),
    (uint8_t)(entryIndex & 0xFF), (uint8_t)(entryIndex >> 8)
  };
  return sendFrame(packId(h), pl, 6);
}

// ===== Имена =====
const char* typeName(uint8_t t) {
  switch (t) {
    case LCP_Folder: return "Folder"; case LCP_Label: return "Label";
    case LCP_Bool: return "Bool"; case LCP_Enum: return "Enum";
    case LCP_Bitfield32: return "Bits"; case LCP_Int32: return "Int32";
    case LCP_Uint32: return "UInt32"; case LCP_Int64: return "Int64";
    case LCP_Uint64: return "UInt64"; case LCP_Float: return "Float";
    case LCP_Double: return "Double"; case LCP_Decimal32: return "Decimal";
    case LCP_String: return "String"; default: return "?";
  }
}

const char* msgIdName(uint16_t m) {
  switch (m) {
    case 0x399: return "Req"; case 0x39A: return "Data"; case 0x39B: return "Desc";
    case 0x39C: return "Name"; case 0x39D: return "Text"; case 0x39E: return "Value";
    default:    return "?";
  }
}

// ===== CP1251 → транслит ASCII =====
// alight шлёт строки в win-1251. Чтобы Serial / Adafruit GFX могли показать —
// конвертируем русские буквы в латинскую транслитерацию.
const char* cp1251_translit_lower[] = {
  "a","b","v","g","d","e","zh","z","i","y","k","l","m","n","o","p",
  "r","s","t","u","f","h","ts","ch","sh","sch","'","y","'","e","yu","ya"
};
const char* cp1251_translit_upper[] = {
  "A","B","V","G","D","E","Zh","Z","I","Y","K","L","M","N","O","P",
  "R","S","T","U","F","H","Ts","Ch","Sh","Sch","'","Y","'","E","Yu","Ya"
};

void cp1251ToAscii(const uint8_t* src, uint8_t srcLen, char* dst, size_t dstSize) {
  size_t di = 0;
  for (uint8_t si = 0; si < srcLen && di + 1 < dstSize; si++) {
    uint8_t c = src[si];
    if (c == 0) break;
    if (c < 0x80) {
      dst[di++] = (char)c;
    } else if (c == 0xA8) {              // Ё
      if (di + 2 < dstSize) { dst[di++]='Y'; dst[di++]='o'; }
    } else if (c == 0xB8) {              // ё
      if (di + 2 < dstSize) { dst[di++]='y'; dst[di++]='o'; }
    } else if (c >= 0xC0 && c <= 0xDF) { // А..Я
      const char* t = cp1251_translit_upper[c - 0xC0];
      while (*t && di + 1 < dstSize) dst[di++] = *t++;
    } else if (c >= 0xE0 && c <= 0xFF) { // а..я
      const char* t = cp1251_translit_lower[c - 0xE0];
      while (*t && di + 1 < dstSize) dst[di++] = *t++;
    } else {
      dst[di++] = '?';
    }
  }
  dst[di] = 0;
}

// ===== Парсинг готовых ответов =====
void onDirData(uint8_t dirIdx, const uint8_t* d, uint16_t len) {
  if (len < 6 || dirIdx >= MAX_DIRS) return;
  uint16_t entrySize = d[0] | (d[1] << 8);
  uint16_t nameSize  = d[2] | (d[3] << 8);
  uint16_t actualDir = d[4] | (d[5] << 8);
  if (actualDir != dirIdx) {
    Serial.printf("    DirData mismatch: expected dir=%u got dir=%u (drop)\n", dirIdx, actualDir);
    return;
  }
  if (dirs[dirIdx].known) {
    Serial.printf("    DirData[%u]: already known (drop)\n", dirIdx);
    return;
  }
  if (entrySize == 0 || entrySize > MAX_ENTRIES_DIR) {
    Serial.printf("    DirData[%u]: bogus entrySize=%u (drop)\n", dirIdx, entrySize);
    return;
  }
  if (nameSize == 0 || nameSize > NAME_BUF) {
    Serial.printf("    DirData[%u]: bogus nameSize=%u (drop)\n", dirIdx, nameSize);
    return;
  }
  dirs[dirIdx].known = true;
  dirs[dirIdx].entrySize = entrySize;
  dirs[dirIdx].nameSize  = nameSize;
  Serial.printf("    DirData[%u]: entries=%u name=%u\n", dirIdx, entrySize, nameSize);
}

void onEntryData(uint8_t dirIdx, uint16_t expectedEntryIdx, const uint8_t* d, uint16_t len) {
  if (dirIdx >= MAX_DIRS) return;
  if (len < 4) return;
  uint16_t respEntryIdx = d[2] | (d[3] << 8);
  if (respEntryIdx != expectedEntryIdx) {
    Serial.printf("    EntryData mismatch: expected entry=%u got entry=%u (drop)\n",
                  expectedEntryIdx, respEntryIdx);
    return;
  }
  if (expectedEntryIdx >= MAX_ENTRIES_DIR) return;
  EntryInfo& e = dirs[dirIdx].entries[expectedEntryIdx];
  e.type = d[0]; e.mode = d[1]; e.entryIdx = respEntryIdx;
  if (len >= 6)  e.varSize  = d[4] | (d[5] << 8);
  if (len >= 8)  e.descSize = d[6] | (d[7] << 8);
  if (len >= 10) e.textSize = d[8] | (d[9] << 8);
  e.hasData = true;
  Serial.printf("    EntryData[%u,%u]: type=%s mode=0x%02X varSz=%u descSz=%u textSz=%u\n",
                dirIdx, expectedEntryIdx, typeName(e.type), e.mode, e.varSize, e.descSize, e.textSize);
}

void onName(uint8_t dirIdx, int16_t entryIdx, const uint8_t* d, uint16_t len) {
  // строка cp1251 с возможным \0
  uint16_t n = len;
  for (uint16_t i = 0; i < n; i++) if (d[i] == 0) { n = i; break; }

  char buf[NAME_BUF];
  cp1251ToAscii(d, n, buf, sizeof(buf));

  if (entryIdx < 0) {
    if (dirIdx < MAX_DIRS) {
      strncpy(dirs[dirIdx].name, buf, NAME_BUF - 1);
      dirs[dirIdx].name[NAME_BUF - 1] = 0;
    }
    Serial.printf("    DirName[%u] = \"%s\" (raw %u bytes)\n", dirIdx, buf, len);
  } else if (dirIdx < MAX_DIRS && entryIdx < MAX_ENTRIES_DIR) {
    EntryInfo& e = dirs[dirIdx].entries[entryIdx];
    strncpy(e.name, buf, NAME_BUF - 1); e.name[NAME_BUF - 1] = 0;
    e.hasName = true;
    Serial.printf("    EntryName[%u,%u] = \"%s\" (raw %u bytes)\n", dirIdx, entryIdx, buf, len);
  }
}

void onText(uint8_t dirIdx, uint8_t entryIdx, const uint8_t* d, uint16_t len) {
  if (dirIdx >= MAX_DIRS || entryIdx >= MAX_ENTRIES_DIR) return;
  EntryInfo& e = dirs[dirIdx].entries[entryIdx];
  uint16_t n = len; for (uint16_t i = 0; i < n; i++) if (d[i]==0) { n=i; break; }
  cp1251ToAscii(d, n, e.text, TEXT_BUF);
  e.hasText = true;
  Serial.printf("    EntryText[%u,%u] = \"%s\" (%u bytes)\n", dirIdx, entryIdx, e.text, len);
}

void onDesc(uint8_t dirIdx, uint8_t entryIdx, const uint8_t* d, uint16_t len) {
  if (dirIdx >= MAX_DIRS || entryIdx >= MAX_ENTRIES_DIR) return;
  EntryInfo& e = dirs[dirIdx].entries[entryIdx];
  uint16_t n = (len < DESC_BUF) ? len : DESC_BUF;
  memcpy(e.desc, d, n); e.descLen = n; e.hasDesc = true;
}

void onValue(uint8_t dirIdx, uint8_t entryIdx, const uint8_t* d, uint16_t len) {
  if (dirIdx >= MAX_DIRS || entryIdx >= MAX_ENTRIES_DIR) return;
  EntryInfo& e = dirs[dirIdx].entries[entryIdx];
  uint16_t n = (len < sizeof(e.value)) ? len : sizeof(e.value);
  memcpy(e.value, d, n); e.valueLen = n; e.hasValue = true;
}

// ===== Сборщик multi-frame по каналам =====
//   alight шлёт каждое поле (Data/Name/Desc/Text/Value) в отдельном TCP-объекте,
//   все они могут одновременно собираться. Делаем по одному ассемблеру на каждое.

struct Channel {
  bool     active;
  uint16_t msgId;
  uint16_t pos;
  uint8_t  expectedParity; // ждём такой parity у следующего кадра
  uint8_t  buf[ASM_BUF];
};

// Каналы для пяти типов ответа.
Channel channels[5];
const uint16_t channelMsgIds[5] = {
  MSG_PARAM_DATA, MSG_PARAM_DESCRIPTOR, MSG_PARAM_NAME, MSG_PARAM_TEXT, MSG_PARAM_VALUE
};

int channelIndexFor(uint16_t msgId) {
  for (int i = 0; i < 5; i++) if (channelMsgIds[i] == msgId) return i;
  return -1;
}

void resetChannel(int i) {
  channels[i].active = false;
  channels[i].pos = 0;
  channels[i].msgId = channelMsgIds[i];
  channels[i].expectedParity = 0;
}

void resetAllChannels() {
  for (int i = 0; i < 5; i++) resetChannel(i);
}

// CTS — отправить контрольный кадр в alight.
// rts=1, eom=0 — "продолжай слать"; rts=0, eom=1 — "финал, всё принято".
// КЛЮЧЕВОЕ: CTS идёт как CAN remote-frame (RTR=1), тк `Request` в LEVCAN
// маппится на КАН-левел RTR-бит (см. levcan.h:72 "//RTR bit").
bool sendCTS(uint16_t msgId, uint8_t parity, uint8_t rts, uint8_t eom) {
  LcHeader h{};
  h.source = MY_ADDR;        // мы — приёмник
  h.target = TARGET_ADDR;
  h.msgId  = msgId;
  h.parity = parity;
  h.rts    = rts;            // в LEVCAN это RTS_CTS бит
  h.eom    = eom;
  h.prio   = 0;
  if (VERBOSE_CTS)
    Serial.printf("      <- CTS msg=0x%X par=%u rts=%u eom=%u (RTR)\n", msgId, parity, rts, eom);
  return sendFrame(packId(h), nullptr, 0, /*rtr=*/true);
}

// Состояние ожидания (что мы запросили)
struct {
  bool waiting;
  bool isDir;
  uint8_t dirIdx;
  uint8_t entryIdx; // 0xFF для dir
} expecting;

// Когда channel целиком собран (получен EoM=1) — диспатчим в onXxx.
void dispatchChannel(int idx) {
  Channel& c = channels[idx];
  uint16_t msgId = c.msgId;
  Serial.printf("    [done %s, %u bytes]\n", msgIdName(msgId), c.pos);

  if (!expecting.waiting) { resetChannel(idx); return; }
  uint8_t dirIdx = expecting.dirIdx;
  uint8_t entryIdx = expecting.entryIdx;

  switch (msgId) {
    case MSG_PARAM_DATA:
      if (expecting.isDir) onDirData(dirIdx, c.buf, c.pos);
      else                 onEntryData(dirIdx, entryIdx, c.buf, c.pos);
      break;
    case MSG_PARAM_NAME:
      if (expecting.isDir) onName(dirIdx, -1, c.buf, c.pos);
      else                 onName(dirIdx, (int16_t)entryIdx, c.buf, c.pos);
      break;
    case MSG_PARAM_DESCRIPTOR:
      if (!expecting.isDir) onDesc(dirIdx, entryIdx, c.buf, c.pos);
      break;
    case MSG_PARAM_TEXT:
      if (!expecting.isDir) onText(dirIdx, entryIdx, c.buf, c.pos);
      break;
    case MSG_PARAM_VALUE:
      if (!expecting.isDir) onValue(dirIdx, entryIdx, c.buf, c.pos);
      break;
  }
  resetChannel(idx);
}

void printRx(const twai_message_t& m, const LcHeader& h) {
  if (!VERBOSE_RX) return;
  Serial.printf("      RX 0x%X src=%u msg=0x%X(%s) eom=%u par=%u rts=%u",
                m.identifier, h.source, h.msgId, msgIdName(h.msgId),
                h.eom, h.parity, h.rts);
  Serial.print(" data=");
  for (uint8_t i = 0; i < m.data_length_code; i++) {
    if (m.data[i] < 16) Serial.print("0");
    Serial.print(m.data[i], HEX); Serial.print(' ');
  }
  Serial.println();
}

// ===== Стат =====
struct RxStats { uint32_t total=0, data=0, name=0, desc=0, text=0, value=0, other=0, ctsOut=0; };
RxStats rxStats;

// ===== Обработка кадра =====
void handleFrame(const twai_message_t& m) {
  if (!m.extd || m.rtr) return;
  LcHeader h = unpackId(m.identifier);
  if (h.source != TARGET_ADDR || h.target != MY_ADDR) return;

  rxStats.total++;
  switch (h.msgId) {
    case MSG_PARAM_DATA: rxStats.data++; break;
    case MSG_PARAM_NAME: rxStats.name++; break;
    case MSG_PARAM_DESCRIPTOR: rxStats.desc++; break;
    case MSG_PARAM_TEXT: rxStats.text++; break;
    case MSG_PARAM_VALUE: rxStats.value++; break;
    default: rxStats.other++; break;
  }
  printRx(m, h);

  int chIdx = channelIndexFor(h.msgId);
  if (chIdx < 0) return; // не наш канал

  Channel& c = channels[chIdx];

  // Single frame: rts=1, eom=1 — данные сразу целиком
  if (h.rts == 1 && h.eom == 1) {
    if (c.active) {
      Serial.printf("    [%s] RTS+EoM while accumulating (%u bytes) → reset\n", msgIdName(h.msgId), c.pos);
    }
    resetChannel(chIdx);
    c.active = true;
    if (m.data_length_code <= ASM_BUF) {
      memcpy(c.buf, m.data, m.data_length_code);
      c.pos = m.data_length_code;
    }
    // Финальный ACK для single-frame TCP: rts=0, eom=1, parity по формуле.
    uint8_t ackParity = (~(((c.pos + 7) / 8))) & 1;
    sendCTS(h.msgId, ackParity, 0, 1);
    rxStats.ctsOut++;
    dispatchChannel(chIdx);
    return;
  }

  // Начало multi-frame: rts=1, eom=0
  if (h.rts == 1 && h.eom == 0) {
    resetChannel(chIdx);
    c.active = true;
    if (m.data_length_code <= ASM_BUF) {
      memcpy(c.buf, m.data, m.data_length_code);
      c.pos = m.data_length_code;
    }
    // Parity для следующего кадра по правилу levcan:
    //   parity = ~((pos + step_inc - 1) / step_inc) & 1, step_inc=8
    uint8_t nextParity = (~(((c.pos + 7) / 8)) ) & 1;
    c.expectedParity = nextParity;
    sendCTS(h.msgId, nextParity, 1, 0);
    rxStats.ctsOut++;
    return;
  }

  // Продолжение: rts=0
  if (h.rts == 0) {
    if (!c.active) {
      Serial.printf("    [%s] continuation without active assembly → drop\n", msgIdName(h.msgId));
      return;
    }
    // дописываем
    uint16_t take = m.data_length_code;
    if (c.pos + take > ASM_BUF) take = ASM_BUF - c.pos;
    memcpy(c.buf + c.pos, m.data, take);
    c.pos += take;

    if (h.eom == 1) {
      // финальный CTS: rts=0, eom=1
      uint8_t lastParity = (~(((c.pos + 7) / 8))) & 1;
      sendCTS(h.msgId, lastParity, 0, 1);
      rxStats.ctsOut++;
      dispatchChannel(chIdx);
    } else {
      uint8_t nextParity = (~(((c.pos + 7) / 8))) & 1;
      c.expectedParity = nextParity;
      sendCTS(h.msgId, nextParity, 1, 0);
      rxStats.ctsOut++;
    }
    return;
  }
}

// ===== FSM обхода =====
enum WalkState { WS_IDLE, WS_REQ_DIR, WS_REQ_ENTRY, WS_DONE };
WalkState walkState = WS_IDLE;
uint8_t  curDir = 0, curEntry = 0;
uint32_t reqSentMs = 0;
uint8_t  retryCount = 0;
const uint32_t REQ_TIMEOUT_MS = 1000;
const uint8_t  MAX_RETRIES    = 2;

bool dirComplete(uint8_t i) { return dirs[i].known && dirs[i].name[0] != 0; }

bool entryComplete(const EntryInfo& e) {
  if (!e.hasData) return false;
  if (!e.hasName) return false;
  // если тип Folder — desc/text/value не обязательны
  if (e.type == LCP_Folder || e.type == LCP_Label) return true;
  // ждём text если ожидался
  if (e.textSize > 0 && !e.hasText) return false;
  return true;
}

bool isExpectedResponseComplete() {
  if (!expecting.waiting) return false;
  if (expecting.isDir) return dirComplete(expecting.dirIdx);
  return entryComplete(dirs[expecting.dirIdx].entries[expecting.entryIdx]);
}

void drainRx() {
  twai_message_t m;
  uint16_t n = 0;
  while (twai_receive(&m, 0) == ESP_OK) n++;
  if (n) Serial.printf("    drained %u stale\n", n);
}

void issueRequest() {
  drainRx();
  resetAllChannels();
  reqSentMs = millis();
  if (walkState == WS_REQ_DIR) {
    Serial.printf("--> reqDir(%u)\n", curDir);
    expecting = { true, true, curDir, 0xFF };
    if (!reqDir(curDir)) Serial.println("    TX failed");
  } else if (walkState == WS_REQ_ENTRY) {
    Serial.printf("--> reqEntry(%u, %u)\n", curDir, curEntry);
    expecting = { true, false, curDir, curEntry };
    EntryInfo& e = dirs[curDir].entries[curEntry];
    e.hasData = e.hasName = e.hasDesc = e.hasText = e.hasValue = false;
    if (!reqEntry(curDir, curEntry)) Serial.println("    TX failed");
  }
}

void startWalk() {
  for (uint8_t i = 0; i < MAX_DIRS; i++) {
    memset(&dirs[i], 0, sizeof(dirs[i]));
    for (uint8_t j = 0; j < MAX_ENTRIES_DIR; j++) dirs[i].entries[j].childDir = 0xFFFF;
  }
  resetAllChannels();
  totalDirsKnown = 1;
  curDir = 0; curEntry = 0; retryCount = 0;
  walkState = WS_REQ_DIR;
  Serial.println("=== START WALK ===");
}

void advanceWalk() {
  if (walkState == WS_REQ_DIR) {
    DirInfo& d = dirs[curDir];
    Serial.printf("    Dir %u (\"%s\") complete: entries=%u\n", curDir, d.name, d.entrySize);
    if (d.entrySize == 0 || d.entrySize > MAX_ENTRIES_DIR) {
      curDir++;
      walkState = (curDir >= totalDirsKnown || curDir >= MAX_DIRS) ? WS_DONE : WS_REQ_DIR;
    } else {
      curEntry = 0; walkState = WS_REQ_ENTRY;
    }
    retryCount = 0;
  } else if (walkState == WS_REQ_ENTRY) {
    EntryInfo& e = dirs[curDir].entries[curEntry];
    if (e.type == LCP_Folder && totalDirsKnown < MAX_DIRS) {
      e.childDir = totalDirsKnown++;
      Serial.printf("    folder \"%s\" → dir %u\n", e.name, e.childDir);
    }
    curEntry++;
    if (curEntry >= dirs[curDir].entrySize) {
      dirs[curDir].fullyRead = true;
      curDir++;
      if (curDir >= totalDirsKnown || curDir >= MAX_DIRS) walkState = WS_DONE;
      else { curEntry = 0; walkState = WS_REQ_DIR; }
    }
    retryCount = 0;
  }
}

// ===== printMenu =====
void printMenu() {
  Serial.println("\n========= MENU TREE =========");
  for (uint8_t i = 0; i < totalDirsKnown && i < MAX_DIRS; i++) {
    DirInfo& d = dirs[i];
    Serial.printf("[Dir %u] \"%s\" (entries=%u)\n", i, d.name, d.entrySize);
    for (uint8_t j = 0; j < d.entrySize && j < MAX_ENTRIES_DIR; j++) {
      EntryInfo& e = d.entries[j];
      Serial.printf("  [%u] %-8s mode=0x%02X \"%s\"", j, typeName(e.type), e.mode, e.name);
      if (e.hasText && e.text[0]) Serial.printf(" fmt=\"%s\"", e.text);
      if (e.hasValue) {
        Serial.print(" val=");
        for (uint8_t k = 0; k < e.valueLen; k++) {
          if (e.value[k] < 16) Serial.print("0");
          Serial.print(e.value[k], HEX); Serial.print(' ');
        }
      }
      if (e.childDir != 0xFFFF) Serial.printf(" → dir %u", e.childDir);
      Serial.println();
    }
  }
  Serial.println("========= END =========\n");
}

// ===== OLED =====
char oledLine1[24] = "boot";
uint8_t oledPage = 0;
uint32_t lastOledMs = 0, lastPageMs = 0;

void drawScreen() {
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  if (walkState == WS_DONE && totalDirsKnown > 0) {
    uint8_t i = oledPage % totalDirsKnown;
    DirInfo& d = dirs[i];
    display.printf("D%u: %s\n", i, d.name);
    for (uint8_t j = 0; j < d.entrySize && j < 6; j++) {
      EntryInfo& e = d.entries[j];
      display.printf("%c %.18s\n", e.type == LCP_Folder ? '>' : ' ', e.name);
    }
  } else {
    display.println("aLight LEVCAN");
    display.println(oledLine1);
    display.printf("dir %u/%u entry %u\n", curDir, totalDirsKnown, curEntry);
  }
  display.display();
}

bool initDisplay() {
  Wire.begin(OLED_SDA, OLED_SCL);
  return display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false);
}

void setup() {
  Serial.begin(115200); delay(500);
  bool oledOk = initDisplay();
  strcpy(oledLine1, "init"); if (oledOk) drawScreen();
  Serial.println();
  Serial.println("=== ESP32 LEVCAN reader v3 (CTS+cp1251) ===");
  Serial.printf("My addr=%u, target=%u, 1Mbps\n", MY_ADDR, TARGET_ADDR);
  if (!initCAN()) {
    Serial.println("TWAI init FAILED"); strcpy(oledLine1, "CAN FAIL");
    if (oledOk) drawScreen(); while (true) delay(1000);
  }
  Serial.println("TWAI started"); strcpy(oledLine1, "ready");
  if (oledOk) drawScreen();
  delay(500);
  startWalk(); issueRequest();
}

void loop() {
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);

  if (expecting.waiting && isExpectedResponseComplete()) {
    expecting.waiting = false;
    advanceWalk();
    if (walkState != WS_DONE) issueRequest();
    else {
      Serial.println("=== WALK DONE ===");
      Serial.printf("RxStats: total=%u data=%u name=%u desc=%u text=%u value=%u other=%u ctsOut=%u\n",
                    rxStats.total, rxStats.data, rxStats.name, rxStats.desc,
                    rxStats.text, rxStats.value, rxStats.other, rxStats.ctsOut);
      printMenu();
      strcpy(oledLine1, "DONE");
    }
  }

  if (expecting.waiting && (millis() - reqSentMs > REQ_TIMEOUT_MS)) {
    retryCount++;
    Serial.printf("    timeout (retry %u/%u)\n", retryCount, MAX_RETRIES);
    if (retryCount > MAX_RETRIES) {
      Serial.println("    skip");
      expecting.waiting = false;
      if (walkState == WS_REQ_DIR) {
        walkState = WS_DONE;
        Serial.println("=== WALK ABORTED (dir timeout) ===");
        Serial.printf("RxStats: total=%u data=%u name=%u desc=%u text=%u value=%u other=%u ctsOut=%u\n",
                      rxStats.total, rxStats.data, rxStats.name, rxStats.desc,
                      rxStats.text, rxStats.value, rxStats.other, rxStats.ctsOut);
        printMenu();
        strcpy(oledLine1, "DONE (partial)");
      } else if (walkState == WS_REQ_ENTRY) {
        curEntry++;
        if (curEntry >= dirs[curDir].entrySize) {
          dirs[curDir].fullyRead = true;
          curDir++;
          if (curDir >= totalDirsKnown || curDir >= MAX_DIRS) walkState = WS_DONE;
          else { curEntry = 0; walkState = WS_REQ_DIR; }
        }
        retryCount = 0;
      }
      if (walkState != WS_DONE) issueRequest();
      else { printMenu(); strcpy(oledLine1, "DONE (partial)"); }
    } else {
      issueRequest();
    }
  }

  uint32_t alerts;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_BUS_OFF) { Serial.println("ALERT: BUS OFF -> recovering"); twai_initiate_recovery(); }
  }

  if (millis() - lastOledMs > 250) { lastOledMs = millis(); drawScreen(); }
  if (walkState == WS_DONE && millis() - lastPageMs > 3000) { lastPageMs = millis(); oledPage++; }
}
