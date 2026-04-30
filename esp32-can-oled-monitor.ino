// ESP32 + TJA1050 + SSD1306 + 4 кнопки: интерактивный обозреватель LEVCAN-меню.
//
// v4 — мультиузловое обнаружение, ленивая подгрузка одного уровня, live-обновление RO-параметров.
//
// Кнопки:
//   GPIO13 = Back   (вверх по дереву / отмена)
//   GPIO14 = Up     (курсор вверх / уменьшить)
//   GPIO27 = Down   (курсор вниз  / увеличить)
//   GPIO33 = Select (войти / применить)
//
// Пины:
//   OLED: SDA=GPIO5, SCL=GPIO4
//   CAN:  TX=GPIO25, RX=GPIO26  → TJA1050
//
// Архитектура:
//   1) DISCOVER : пассивно слушаем 0x380 AddressClaimed + периодически шлём
//                 broadcast 0x388 NodeName request → растёт список узлов
//   2) DEVICES  : рисуем список устройств, кнопки Up/Down/Select
//   3) LOAD_DIR : загружаем имена и типы записей одного уровня меню
//                 (DirInfo + entry-data для каждой entry)
//   4) BROWSE   : рисуем меню, кнопки Up/Down/Select/Back, фоном поллим
//                 значения для entries с mode & LCP_LiveUpdate
//   5) LOAD_VAL : при Select на лист (не Folder) — подгружаем полные text/desc/value
//
// Полный обход всего дерева (для документирования меню) можно включить
// через #define ENABLE_FULL_WALKER ниже — отключён по умолчанию.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/twai.h"

// ====== HW ======
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_SDA 5
#define OLED_SCL 4
#define CAN_TX GPIO_NUM_25
#define CAN_RX GPIO_NUM_26
#define BTN_BACK   13
#define BTN_UP     14
#define BTN_DOWN   27
#define BTN_SELECT 33

// ====== LEVCAN ======
static const uint8_t  MY_ADDR     = 100;
static const uint8_t  BROADCAST   = 127;
static const uint16_t MSG_ADDRESS_CLAIMED   = 0x380;
static const uint16_t MSG_NODE_NAME         = 0x388;
static const uint16_t MSG_DEVICE_NAME       = 0x389;
static const uint16_t MSG_PARAM_REQUEST     = 0x399;
static const uint16_t MSG_PARAM_DATA        = 0x39A;
static const uint16_t MSG_PARAM_DESCRIPTOR  = 0x39B;
static const uint16_t MSG_PARAM_NAME        = 0x39C;
static const uint16_t MSG_PARAM_TEXT        = 0x39D;
static const uint16_t MSG_PARAM_VALUE       = 0x39E;
static const uint16_t LCP_REQ_DATA           = 0x01;
static const uint16_t LCP_REQ_NAME           = 0x02;
static const uint16_t LCP_REQ_TEXT           = 0x04;
static const uint16_t LCP_REQ_DESCRIPTOR     = 0x08;
static const uint16_t LCP_REQ_VARIABLE       = 0x10;
static const uint16_t LCP_REQ_DATA_NAME      = 0x03;  // Data + Name
static const uint16_t LCP_REQ_FULL_ENTRY     = 0x1F;
static const uint16_t LCP_REQ_DIRECTORY_INFO = 0x20;

enum LCP_Type {
  LCP_Folder=0, LCP_Label, LCP_Bool, LCP_Enum, LCP_Bitfield32,
  LCP_Int32, LCP_Uint32, LCP_Int64, LCP_Uint64, LCP_Float,
  LCP_Double, LCP_Decimal32, LCP_String, LCP_End=0xFF
};

#define LCP_MODE_RO       0x01
#define LCP_MODE_WO       0x02
#define LCP_MODE_LIVE_UPD 0x04
#define LCP_MODE_LIVE_CHG 0x08

// ====== Пределы ======
#define MAX_NODES        8
#define MAX_ENTRIES_DIR  20
#define NAME_BUF         48
#define TEXT_BUF         96
#define DESC_BUF         32
#define ASM_BUF          128
#define DEPTH_STACK      8

// ====== Опции ======
static const bool VERBOSE_RX  = true;
static const bool VERBOSE_CTS = false;
// #define ENABLE_FULL_WALKER 1   // раскомментируй чтобы по B+S при загрузке снять полный дамп

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ====== Структуры данных ======
struct Node {
  bool     present;
  uint8_t  addr;
  char     deviceName[NAME_BUF];  // LC_SYS_DeviceName  (0x389) — модель/firmware-имя
  char     nodeName[NAME_BUF];    // LC_SYS_NodeName    (0x388) — пользовательское имя из дескриптора
  bool     hasDeviceName;
  bool     hasNodeName;
  uint32_t lastSeenMs;
};

struct EntryInfo {
  uint8_t  type;
  uint8_t  mode;
  uint16_t entryIdx;
  uint16_t varSize;
  uint16_t descSize;
  uint16_t textSize;
  bool     hasData, hasName, hasDesc, hasText, hasValue;
  char     name[NAME_BUF];
  char     text[TEXT_BUF];
  uint8_t  desc[DESC_BUF]; uint8_t descLen;
  uint8_t  value[16];      uint8_t valueLen;
};

struct DirInfo {
  bool     known;
  uint16_t entrySize;
  uint16_t nameSize;
  char     name[NAME_BUF];
  EntryInfo entries[MAX_ENTRIES_DIR];
};

// ====== Глобалы ======
Node nodes[MAX_NODES];
uint8_t nodeCount = 0;
int     selectedNode = -1;          // индекс в nodes[]
uint8_t targetAddr = 0;              // адрес активного устройства

DirInfo curDir;                      // текущая открытая директория
uint16_t curDirIndex = 0;
uint8_t  cursor = 0;                 // курсор в меню
uint8_t  scrollTop = 0;              // верхняя видимая строка

// стек "куда возвращаться по Back"
struct StackFrame { uint16_t dirIndex; uint8_t cursor; uint8_t scrollTop; };
StackFrame dirStack[DEPTH_STACK];
uint8_t    dirStackPos = 0;

// ====== ID pack/unpack ======
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

// ====== TWAI ======
bool initCAN() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 16; g.rx_queue_len = 64;
  g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED;
  twai_timing_config_t t = { .brp = 10, .tseg_1 = 6, .tseg_2 = 1, .sjw = 2, .triple_sampling = false };
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}
bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len, bool rtr=false) {
  twai_message_t m = {}; m.identifier = id; m.extd = 1; m.rtr = rtr ? 1 : 0;
  m.data_length_code = len;
  for (uint8_t i = 0; i < len && i < 8; i++) m.data[i] = data[i];
  return twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK;
}

// ====== Запросы ======
bool reqDir(uint8_t target, uint16_t dirIndex) {
  LcHeader h{}; h.source=MY_ADDR; h.target=target; h.msgId=MSG_PARAM_REQUEST;
  h.eom=1; h.parity=1; h.rts=1; h.prio=0;
  uint8_t pl[4] = { (uint8_t)(LCP_REQ_DIRECTORY_INFO & 0xFF), (uint8_t)(LCP_REQ_DIRECTORY_INFO >> 8),
                    (uint8_t)(dirIndex & 0xFF), (uint8_t)(dirIndex >> 8) };
  return sendFrame(packId(h), pl, 4);
}
bool reqEntryCmd(uint8_t target, uint16_t dirIndex, uint16_t entryIndex, uint16_t cmdMask) {
  LcHeader h{}; h.source=MY_ADDR; h.target=target; h.msgId=MSG_PARAM_REQUEST;
  h.eom=1; h.parity=1; h.rts=1; h.prio=0;
  uint8_t pl[6] = { (uint8_t)(cmdMask & 0xFF), (uint8_t)(cmdMask >> 8),
                    (uint8_t)(dirIndex & 0xFF), (uint8_t)(dirIndex >> 8),
                    (uint8_t)(entryIndex & 0xFF), (uint8_t)(entryIndex >> 8) };
  return sendFrame(packId(h), pl, 6);
}
bool reqEntry         (uint8_t target, uint16_t dir, uint16_t e) { return reqEntryCmd(target, dir, e, LCP_REQ_DATA_NAME); }
bool reqEntryVariable (uint8_t target, uint16_t dir, uint16_t e) { return reqEntryCmd(target, dir, e, LCP_REQ_VARIABLE);  }
bool reqSysName(uint8_t target, uint16_t msgId) {
  // RTR-запрос (Request flag) на системное msgId. Если target=BROADCAST — отвечают все.
  LcHeader h{}; h.source=MY_ADDR; h.target=target; h.msgId=msgId;
  h.eom=1; h.parity=1; h.rts=1; h.prio=0;
  return sendFrame(packId(h), nullptr, 0, /*rtr=*/true);
}
bool reqDeviceName(uint8_t target) { return reqSysName(target, MSG_DEVICE_NAME); }
bool reqNodeName  (uint8_t target) { return reqSysName(target, MSG_NODE_NAME);   }
bool broadcastDeviceNameQuery() { return reqDeviceName(BROADCAST); }
bool broadcastNodeNameQuery()   { return reqNodeName(BROADCAST);   }

// ====== Имена типов ======
const char* typeName(uint8_t t) {
  switch (t) {
    case LCP_Folder:return "Fold"; case LCP_Label:return "Lbl";
    case LCP_Bool:return "Bool"; case LCP_Enum:return "Enum";
    case LCP_Bitfield32:return "Bits"; case LCP_Int32:return "I32";
    case LCP_Uint32:return "U32"; case LCP_Int64:return "I64";
    case LCP_Uint64:return "U64"; case LCP_Float:return "F32";
    case LCP_Double:return "F64"; case LCP_Decimal32:return "Dec";
    case LCP_String:return "Str"; default:return "?";
  }
}

// ====== CP1251 → ASCII транслит ======
const char* trL[] = { "a","b","v","g","d","e","zh","z","i","y","k","l","m","n","o","p",
                      "r","s","t","u","f","h","ts","ch","sh","sch","'","y","'","e","yu","ya" };
const char* trU[] = { "A","B","V","G","D","E","Zh","Z","I","Y","K","L","M","N","O","P",
                      "R","S","T","U","F","H","Ts","Ch","Sh","Sch","'","Y","'","E","Yu","Ya" };
void cp1251ToAscii(const uint8_t* src, uint16_t srcLen, char* dst, size_t dstSize) {
  size_t di = 0;
  for (uint16_t si = 0; si < srcLen && di + 1 < dstSize; si++) {
    uint8_t c = src[si]; if (c == 0) break;
    if (c == 0xB0) { if (di + 4 < dstSize) { dst[di++]=' '; dst[di++]='d'; dst[di++]='e'; dst[di++]='g'; } }
    else if (c < 0x80) dst[di++] = (char)c;
    else if (c == 0xA8) { if (di + 2 < dstSize) { dst[di++]='Y'; dst[di++]='o'; } }
    else if (c == 0xB8) { if (di + 2 < dstSize) { dst[di++]='y'; dst[di++]='o'; } }
    else if (c >= 0xC0 && c <= 0xDF) { const char* t = trU[c-0xC0]; while (*t && di+1 < dstSize) dst[di++] = *t++; }
    else if (c >= 0xE0 && c <= 0xFF) { const char* t = trL[c-0xE0]; while (*t && di+1 < dstSize) dst[di++] = *t++; }
    else dst[di++] = '?';
  }
  dst[di] = 0;
}

// ====== Работа со списком узлов ======
int findOrAddNode(uint8_t addr) {
  for (int i = 0; i < nodeCount; i++) if (nodes[i].addr == addr) return i;
  if (nodeCount >= MAX_NODES) return -1;
  int idx = nodeCount++;
  nodes[idx] = {};
  nodes[idx].present = true; nodes[idx].addr = addr;
  snprintf(nodes[idx].deviceName, NAME_BUF, "node%u", addr);
  snprintf(nodes[idx].nodeName,   NAME_BUF, "");
  nodes[idx].lastSeenMs = millis();
  return idx;
}

// Подобрать наиболее «читабельное» имя устройства для UI:
// предпочитаем NodeName (заданный пользователем), затем DeviceName, иначе addr.
const char* nodeDisplayName(const Node& n) {
  if (n.hasNodeName   && n.nodeName[0]   != 0) return n.nodeName;
  if (n.hasDeviceName && n.deviceName[0] != 0) return n.deviceName;
  return n.deviceName; // там лежит "node%u"
}

// ====== Сборщик multi-frame ======
struct Channel { bool active; uint16_t msgId; uint16_t pos; uint8_t buf[ASM_BUF]; uint8_t srcAddr; };
const uint16_t CHAN_IDS[] = { MSG_PARAM_DATA, MSG_PARAM_DESCRIPTOR, MSG_PARAM_NAME, MSG_PARAM_TEXT, MSG_PARAM_VALUE,
                              MSG_NODE_NAME, MSG_DEVICE_NAME };
#define CHAN_COUNT (sizeof(CHAN_IDS)/sizeof(CHAN_IDS[0]))
Channel channels[CHAN_COUNT];

int channelIndexFor(uint16_t msgId) {
  for (uint8_t i = 0; i < CHAN_COUNT; i++) if (CHAN_IDS[i] == msgId) return i;
  return -1;
}
void resetChannel(int i) { channels[i] = {}; channels[i].msgId = CHAN_IDS[i]; }
void resetAllChannels() { for (uint8_t i = 0; i < CHAN_COUNT; i++) resetChannel(i); }

bool sendCTS(uint8_t target, uint16_t msgId, uint8_t parity, uint8_t rts, uint8_t eom) {
  LcHeader h{}; h.source=MY_ADDR; h.target=target; h.msgId=msgId;
  h.parity=parity; h.rts=rts; h.eom=eom; h.prio=0;
  if (VERBOSE_CTS) Serial.printf("    <- CTS to %u msg=0x%X par=%u rts=%u eom=%u\n", target, msgId, parity, rts, eom);
  return sendFrame(packId(h), nullptr, 0, /*rtr=*/true);
}

// ====== Состояние ожидания ======
enum WaitKind { WK_NONE, WK_DIR, WK_ENTRY, WK_VALUE };
struct {
  WaitKind kind;
  uint8_t  target;
  uint16_t dirIndex;
  uint16_t entryIndex;
  uint32_t startMs;
} pending;

// ====== Парсинг ======
void onDeviceName(uint8_t srcAddr, const uint8_t* d, uint16_t len) {
  int i = findOrAddNode(srcAddr);
  if (i < 0) return;
  uint16_t n = len; for (uint16_t k=0; k<n; k++) if (d[k]==0) { n=k; break; }
  cp1251ToAscii(d, n, nodes[i].deviceName, NAME_BUF);
  nodes[i].hasDeviceName = true;
  nodes[i].lastSeenMs = millis();
  Serial.printf("  Node %u DeviceName=\"%s\"\n", srcAddr, nodes[i].deviceName);
  // Сразу поинтересуемся NodeName, если ещё не знаем.
  if (!nodes[i].hasNodeName) reqNodeName(srcAddr);
}
void onNodeName(uint8_t srcAddr, const uint8_t* d, uint16_t len) {
  int i = findOrAddNode(srcAddr);
  if (i < 0) return;
  uint16_t n = len; for (uint16_t k=0; k<n; k++) if (d[k]==0) { n=k; break; }
  cp1251ToAscii(d, n, nodes[i].nodeName, NAME_BUF);
  nodes[i].hasNodeName = true;
  nodes[i].lastSeenMs = millis();
  Serial.printf("  Node %u NodeName=\"%s\"\n", srcAddr, nodes[i].nodeName);
}

void onDirData(uint16_t dirIdx, const uint8_t* d, uint16_t len) {
  if (len < 6) return;
  uint16_t entrySize = d[0] | (d[1] << 8);
  uint16_t nameSize  = d[2] | (d[3] << 8);
  uint16_t actualDir = d[4] | (d[5] << 8);
  if (actualDir != dirIdx) { Serial.printf("  DirData dir mismatch %u/%u\n", actualDir, dirIdx); return; }
  if (entrySize > MAX_ENTRIES_DIR || nameSize == 0 || nameSize > NAME_BUF) {
    Serial.printf("  DirData bogus size es=%u ns=%u\n", entrySize, nameSize); return;
  }
  curDir.known = true;
  curDir.entrySize = entrySize;
  curDir.nameSize  = nameSize;
  Serial.printf("  DirData[%u]: entries=%u\n", dirIdx, entrySize);
}

void onEntryData(uint16_t expectedEntry, const uint8_t* d, uint16_t len) {
  if (len < 4 || expectedEntry >= MAX_ENTRIES_DIR) return;
  uint16_t respEntry = d[2] | (d[3] << 8);
  if (respEntry != expectedEntry) { Serial.printf("  Entry idx mismatch %u/%u\n", respEntry, expectedEntry); return; }
  EntryInfo& e = curDir.entries[expectedEntry];
  e.type = d[0]; e.mode = d[1]; e.entryIdx = respEntry;
  if (len >= 6)  e.varSize  = d[4] | (d[5] << 8);
  if (len >= 8)  e.descSize = d[6] | (d[7] << 8);
  if (len >= 10) e.textSize = d[8] | (d[9] << 8);
  e.hasData = true;
  Serial.printf("  EntryData[%u]: type=%s mode=0x%02X varSz=%u\n",
                expectedEntry, typeName(e.type), e.mode, e.varSize);
}

void onName(int16_t entryIdx, const uint8_t* d, uint16_t len) {
  uint16_t n = len; for (uint16_t i=0;i<n;i++) if (d[i]==0) { n=i; break; }
  char buf[NAME_BUF]; cp1251ToAscii(d, n, buf, sizeof(buf));
  if (entryIdx < 0) {
    strncpy(curDir.name, buf, NAME_BUF-1); curDir.name[NAME_BUF-1]=0;
    Serial.printf("  DirName: \"%s\"\n", curDir.name);
  }
  else if (entryIdx < MAX_ENTRIES_DIR) {
    EntryInfo& e = curDir.entries[entryIdx];
    strncpy(e.name, buf, NAME_BUF-1); e.name[NAME_BUF-1]=0; e.hasName = true;
    Serial.printf("  EntryName[%u]: \"%s\"\n", (unsigned)entryIdx, e.name);
  }
}

void onText(uint16_t entryIdx, const uint8_t* d, uint16_t len) {
  if (entryIdx >= MAX_ENTRIES_DIR) return;
  EntryInfo& e = curDir.entries[entryIdx];
  uint16_t n = len; for (uint16_t i=0;i<n;i++) if (d[i]==0) { n=i; break; }
  cp1251ToAscii(d, n, e.text, TEXT_BUF);
  e.hasText = true;
}
void onDesc(uint16_t entryIdx, const uint8_t* d, uint16_t len) {
  if (entryIdx >= MAX_ENTRIES_DIR) return;
  EntryInfo& e = curDir.entries[entryIdx];
  uint16_t n = (len < DESC_BUF) ? len : DESC_BUF;
  memcpy(e.desc, d, n); e.descLen = n; e.hasDesc = true;
}
void onValue(uint16_t entryIdx, const uint8_t* d, uint16_t len) {
  Serial.printf("  EntryValue[%u]: len=%u\n", (unsigned)entryIdx, len);
  if (entryIdx >= MAX_ENTRIES_DIR) return;
  EntryInfo& e = curDir.entries[entryIdx];
  uint16_t n = (len < sizeof(e.value)) ? len : sizeof(e.value);
  memcpy(e.value, d, n); e.valueLen = n; e.hasValue = true;
}

// ====== Диспатч собранных каналов ======
void dispatchChannel(int idx) {
  Channel& c = channels[idx];
  uint16_t msgId = c.msgId;

  if (msgId == MSG_DEVICE_NAME) { onDeviceName(c.srcAddr, c.buf, c.pos); resetChannel(idx); return; }
  if (msgId == MSG_NODE_NAME)   { onNodeName  (c.srcAddr, c.buf, c.pos); resetChannel(idx); return; }

  // Все остальные относятся к pending параметр-запросу
  if (pending.kind == WK_NONE) { resetChannel(idx); return; }
  if (c.srcAddr != pending.target) { resetChannel(idx); return; }

  if (pending.kind == WK_DIR) {
    switch (msgId) {
      case MSG_PARAM_DATA: onDirData(pending.dirIndex, c.buf, c.pos); break;
      case MSG_PARAM_NAME: onName(-1, c.buf, c.pos); break;
    }
  } else if (pending.kind == WK_ENTRY) {
    switch (msgId) {
      case MSG_PARAM_DATA: onEntryData(pending.entryIndex, c.buf, c.pos); break;
      case MSG_PARAM_NAME: onName((int16_t)pending.entryIndex, c.buf, c.pos); break;
      case MSG_PARAM_DESCRIPTOR: onDesc(pending.entryIndex, c.buf, c.pos); break;
      case MSG_PARAM_TEXT: onText(pending.entryIndex, c.buf, c.pos); break;
      case MSG_PARAM_VALUE: onValue(pending.entryIndex, c.buf, c.pos); break;
    }
  } else if (pending.kind == WK_VALUE) {
    if (msgId == MSG_PARAM_VALUE) onValue(pending.entryIndex, c.buf, c.pos);
  }
  resetChannel(idx);
}

// ====== Обработка кадра ======
void handleFrame(const twai_message_t& m) {
  if (!m.extd) return;
  LcHeader h = unpackId(m.identifier);
  // Игнорируем кадры ОТ нас или НЕ нам и не broadcast
  if (h.source == MY_ADDR) return;
  if (h.target != MY_ADDR && h.target != BROADCAST) return;

  // Узел напомнил о себе через AddressClaimed → создать запись
  if (h.msgId == MSG_ADDRESS_CLAIMED) {
    int i = findOrAddNode(h.source);
    if (i >= 0) {
      nodes[i].lastSeenMs = millis();
      // спросим у него оба имени, если ещё не знаем
      if (!nodes[i].hasDeviceName) reqDeviceName(h.source);
      if (!nodes[i].hasNodeName)   reqNodeName  (h.source);
    }
    return;
  }

  if (m.rtr) return;  // дальше работаем только с data-кадрами

  if (VERBOSE_RX) {
    Serial.printf("    RX 0x%X src=%u msg=0x%X eom=%u par=%u rts=%u dlc=%u\n",
                  m.identifier, h.source, h.msgId, h.eom, h.parity, h.rts, m.data_length_code);
  }

  int chIdx = channelIndexFor(h.msgId);
  if (chIdx < 0) return;
  Channel& c = channels[chIdx];

  // если канал уже занят другим src — сброс (не мешаем)
  if (c.active && c.srcAddr != h.source) resetChannel(chIdx);

  // single-frame: rts=1, eom=1
  if (h.rts == 1 && h.eom == 1) {
    resetChannel(chIdx); c.active = true; c.srcAddr = h.source;
    if (m.data_length_code <= ASM_BUF) {
      memcpy(c.buf, m.data, m.data_length_code); c.pos = m.data_length_code;
    }
    uint8_t ackParity = (~(((c.pos + 7) / 8))) & 1;
    sendCTS(h.source, h.msgId, ackParity, 0, 1);
    dispatchChannel(chIdx);
    return;
  }
  // начало multi-frame: rts=1, eom=0
  if (h.rts == 1 && h.eom == 0) {
    resetChannel(chIdx); c.active = true; c.srcAddr = h.source;
    if (m.data_length_code <= ASM_BUF) {
      memcpy(c.buf, m.data, m.data_length_code); c.pos = m.data_length_code;
    }
    uint8_t nextParity = (~(((c.pos + 7) / 8))) & 1;
    sendCTS(h.source, h.msgId, nextParity, 1, 0);
    return;
  }
  // продолжение: rts=0
  if (h.rts == 0) {
    if (!c.active) return;
    uint16_t take = m.data_length_code;
    if (c.pos + take > ASM_BUF) take = ASM_BUF - c.pos;
    memcpy(c.buf + c.pos, m.data, take); c.pos += take;
    if (h.eom == 1) {
      uint8_t lastParity = (~(((c.pos + 7) / 8))) & 1;
      sendCTS(h.source, h.msgId, lastParity, 0, 1);
      dispatchChannel(chIdx);
    } else {
      uint8_t nextParity = (~(((c.pos + 7) / 8))) & 1;
      sendCTS(h.source, h.msgId, nextParity, 1, 0);
    }
    return;
  }
}

// ====== Загрузка одной директории (имена + типы entries, без text/desc/value) ======
void clearDir() {
  memset(&curDir, 0, sizeof(curDir));
}
void clearEntryFlags(uint8_t i) {
  if (i >= MAX_ENTRIES_DIR) return;
  EntryInfo& e = curDir.entries[i];
  e.hasData = e.hasName = e.hasDesc = e.hasText = e.hasValue = false;
}

bool waitFor(uint32_t timeoutMs, bool (*cond)()) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);
    if (cond()) return true;
    delay(2);
  }
  return false;
}

bool dirNameReady() { return curDir.known && curDir.name[0] != 0; }
bool entryDataReady() {
  EntryInfo& e = curDir.entries[pending.entryIndex];
  return e.hasData;
}
bool entryNameReady() {
  EntryInfo& e = curDir.entries[pending.entryIndex];
  return e.hasData && e.hasName;
}
bool entryFullReady() {
  EntryInfo& e = curDir.entries[pending.entryIndex];
  // Для Folder/Label валю не ждём
  if (e.hasData && (e.type == LCP_Folder || e.type == LCP_Label)) return e.hasName;
  // Для WriteOnly value не придёт
  if (e.hasData && (e.mode & 0x02 /*WriteOnly*/)) return e.hasName;
  // Для всех остальных звеньев ждём и имя, и значение.
  return e.hasData && e.hasName && e.hasValue;
}
bool entryValueReady() {
  EntryInfo& e = curDir.entries[pending.entryIndex];
  return e.hasValue;
}

bool loadDir(uint8_t target, uint16_t dirIndex) {
  Serial.printf("[loadDir target=%u dir=%u]\n", target, dirIndex);
  resetAllChannels();
  clearDir();
  // 1) Запрос DirInfo (data + name)
  pending = { WK_DIR, target, dirIndex, 0, millis() };
  if (!reqDir(target, dirIndex)) { Serial.println("  reqDir TX fail"); return false; }
  if (!waitFor(1500, dirNameReady)) { Serial.println("  dir timeout"); pending.kind=WK_NONE; return false; }
  pending.kind = WK_NONE;

  // 2) Для каждой entry — ДВА отдельных запроса:
  //   а) Data + Name (для всех)
  //   б) Variable    (опционально — только для читаемых параметров, без Folder/Label/WriteOnly).
  // Разделяем, чтобы не перегружать TX-очередь alight (TCP-send отбрасывается при переполнении).
  for (uint16_t i = 0; i < curDir.entrySize; i++) {
    clearEntryFlags(i);
    pending = { WK_ENTRY, target, dirIndex, i, millis() };
    bool ok = false;
    // Шаг А: Data + Name
    for (int retry = 0; retry < 3 && !ok; retry++) {
      if (retry > 0) resetAllChannels();
      reqEntry(target, dirIndex, i);
      ok = waitFor(1500, entryNameReady);
      if (!ok) Serial.printf("    retry entry %u data/name (hasData=%u hasName=%u)\n",
                             i, curDir.entries[i].hasData, curDir.entries[i].hasName);
    }
    if (!ok) {
      Serial.printf("  entry %u: skip data/name after retries\n", i);
      pending.kind = WK_NONE; delay(20); continue;
    }

    // Шаг Б: Value (если имеет смысл)
    EntryInfo& e = curDir.entries[i];
    bool needValue = (e.type != LCP_Folder) && (e.type != LCP_Label)
                  && ((e.mode & 0x02) == 0)  // не WriteOnly
                  && (e.varSize > 0);
    if (needValue) {
      delay(20);  // лёгкая пауза, чтобы alight разгрузил TX-очередь
      bool vok = false;
      for (int retry = 0; retry < 3 && !vok; retry++) {
        if (retry > 0) resetAllChannels();
        reqEntryVariable(target, dirIndex, i);
        vok = waitFor(1500, entryValueReady);
        if (!vok) Serial.printf("    retry entry %u value\n", i);
      }
      if (!vok) Serial.printf("  entry %u: skip value after retries\n", i);
    }
    pending.kind = WK_NONE;
    delay(20);
  }
  return true;
}

// ====== Live-обновление: только Value для entries с LCP_LiveUpdate ======
bool refreshLiveValues(uint8_t target, uint16_t dirIndex) {
  for (uint16_t i = 0; i < curDir.entrySize; i++) {
    EntryInfo& e = curDir.entries[i];
    if (!e.hasData) continue;
    if (!(e.mode & LCP_MODE_LIVE_UPD)) continue;
    if (e.type == LCP_Folder || e.type == LCP_Label) continue;
    // Запрашиваем полный entry (это вернёт обновлённый Value)
    e.hasValue = false;
    pending = { WK_VALUE, target, dirIndex, i, millis() };
    reqEntryVariable(target, dirIndex, i);
    waitFor(400, entryValueReady);
    pending.kind = WK_NONE;
  }
  return true;
}

// ====== Кнопки (debounce) ======
struct Button { uint8_t pin; uint8_t state; uint32_t lastChangeMs; bool pressed; };
Button btns[4] = { {BTN_BACK,1,0,false}, {BTN_UP,1,0,false}, {BTN_DOWN,1,0,false}, {BTN_SELECT,1,0,false} };
enum BtnIdx { BI_BACK=0, BI_UP=1, BI_DOWN=2, BI_SEL=3 };

void initButtons() {
  for (int i = 0; i < 4; i++) pinMode(btns[i].pin, INPUT_PULLUP);
}
// возвращает true один раз на нажатие (фронт)
bool pollButton(int i) {
  uint8_t s = digitalRead(btns[i].pin);
  uint32_t now = millis();
  if (s != btns[i].state && (now - btns[i].lastChangeMs) > 30) {
    btns[i].lastChangeMs = now; btns[i].state = s;
    if (s == 0) { btns[i].pressed = true; return true; }
  }
  return false;
}

// ====== UI ======
enum AppState { S_DISCOVER, S_DEVICES, S_LOAD_DIR, S_BROWSE };
AppState appState = S_DISCOVER;
uint32_t lastBroadcastMs = 0;
uint32_t lastLiveUpdMs   = 0;
uint32_t discoverStartMs = 0;
const uint32_t BROADCAST_PERIOD = 5000;
const uint32_t LIVE_PERIOD      = 700;
const uint32_t DISCOVER_MIN_MS  = 2500;

void drawHeader(const char* line) {
  display.fillRect(0, 0, SCREEN_WIDTH, 9, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(2, 1);
  display.print(line);
  display.setTextColor(SSD1306_WHITE);
}

void drawFooter(const char* line) {
  display.setCursor(0, SCREEN_HEIGHT - 8);
  display.setTextColor(SSD1306_WHITE);
  display.print(line);
}

void drawDiscover() {
  display.clearDisplay();
  drawHeader("Searching CAN...");
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 12);
  display.printf("Found: %u\n", nodeCount);
  for (uint8_t i = 0; i < nodeCount && i < 4; i++) {
    display.printf(" %u %.16s\n", nodes[i].addr, nodeDisplayName(nodes[i]));
  }
  if (millis() - discoverStartMs > DISCOVER_MIN_MS && nodeCount > 0)
    drawFooter("Sel=Continue");
  display.display();
}

void drawDevices() {
  display.clearDisplay();
  drawHeader("Devices");
  for (uint8_t i = 0; i < nodeCount && i < 4; i++) {
    bool sel = (int)i == selectedNode;
    if (sel) {
      display.fillRect(0, 12 + i*10, SCREEN_WIDTH, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else display.setTextColor(SSD1306_WHITE);
    display.setCursor(2, 13 + i*10);
    display.printf("%u %.18s", nodes[i].addr, nodeDisplayName(nodes[i]));
  }
  display.setTextColor(SSD1306_WHITE);
  drawFooter("U/D Sel=Open B=Scan");
  display.display();
}

void drawLoading(const char* what) {
  display.clearDisplay();
  drawHeader("Loading...");
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 24); display.print(what);
  display.display();
}

// Форматирование значения параметра в строку
void formatValue(const EntryInfo& e, char* out, size_t outSize) {
  if (!e.hasValue) { out[0] = '?'; out[1] = 0; return; }
  switch (e.type) {
    case LCP_Bool: snprintf(out, outSize, "%s", e.value[0] ? "ON" : "off"); break;
    case LCP_Int32: {
      int32_t v = (int32_t)(e.value[0] | (e.value[1]<<8) | (e.value[2]<<16) | (e.value[3]<<24));
      snprintf(out, outSize, "%ld", (long)v); break;
    }
    case LCP_Uint32: {
      uint32_t v = (uint32_t)(e.value[0] | (e.value[1]<<8) | (e.value[2]<<16) | (e.value[3]<<24));
      snprintf(out, outSize, "%lu", (unsigned long)v); break;
    }
    case LCP_Decimal32: {
      // Decimal32: 4 байта integer + descriptor (decimals) известен в descSize/desc[]
      // на упрощение: трактуем как int16 со знаком и подразумеваем 1 знак (как у T-sensors)
      int16_t v = (int16_t)(e.value[0] | (e.value[1]<<8));
      snprintf(out, outSize, "%d.%d", v/10, abs(v%10));
      break;
    }
    case LCP_Float: {
      float f; memcpy(&f, e.value, 4);
      snprintf(out, outSize, "%.2f", f); break;
    }
    case LCP_Enum: snprintf(out, outSize, "%u", e.value[0]); break;
    default:
      out[0] = 0;
      for (uint8_t k = 0; k < e.valueLen && k < 4; k++) {
        char tmp[4]; snprintf(tmp, sizeof(tmp), "%02X", e.value[k]);
        if (strlen(out) + 2 < outSize) strcat(out, tmp);
      }
  }
}

void drawBrowse() {
  display.clearDisplay();
  drawHeader(curDir.name);

  // 4 строки по 10px начиная с y=12 (12+4*10=52, footer с y=55).
  const uint8_t VIS_ROWS = 4;
  if (cursor < scrollTop) scrollTop = cursor;
  if (cursor >= scrollTop + VIS_ROWS) scrollTop = cursor - VIS_ROWS + 1;

  for (uint8_t row = 0; row < VIS_ROWS; row++) {
    uint8_t i = scrollTop + row;
    if (i >= curDir.entrySize) break;
    EntryInfo& e = curDir.entries[i];
    bool sel = (i == cursor);
    int y = 12 + row * 10;
    if (sel) { display.fillRect(0, y, SCREEN_WIDTH, 10, SSD1306_WHITE); display.setTextColor(SSD1306_BLACK); }
    else display.setTextColor(SSD1306_WHITE);

    display.setCursor(2, y + 1);
    char prefix = ' ';
    if (e.type == LCP_Folder) prefix = '>';
    else if (e.type == LCP_Label) prefix = '#';
    char val[12] = "";
    if (e.hasValue) formatValue(e, val, sizeof(val));
    // Окно 21 символ (шрифт 6x8): prefix(1) + name(14) + value(6).
    // ~70% под имя, ~30% под значение — обрезка с '~' в конце.
    char nm[15];
    {
      size_t L = strlen(e.name);
      if (L <= 14) { strcpy(nm, e.name); }
      else         { memcpy(nm, e.name, 13); nm[13] = '~'; nm[14] = 0; }
    }
    char vs[7];
    {
      size_t L = strlen(val);
      if (L <= 6) { strcpy(vs, val); }
      else        { memcpy(vs, val, 5); vs[5] = '~'; vs[6] = 0; }
    }
    display.printf("%c%-14s%6s", prefix, nm, vs);
  }
  display.setTextColor(SSD1306_WHITE);
  display.fillRect(0, SCREEN_HEIGHT-9, SCREEN_WIDTH, 9, SSD1306_BLACK);
  char foot[24]; snprintf(foot, sizeof(foot), "%u/%u %s", cursor+1, curDir.entrySize,
                          dirStackPos > 0 ? "B=back" : "B=devs");
  drawFooter(foot);
  display.display();
}

// ====== Action handlers ======
void enterDevice(int nodeIdx) {
  selectedNode = nodeIdx; targetAddr = nodes[nodeIdx].addr;
  dirStackPos = 0; curDirIndex = 0; cursor = 0; scrollTop = 0;
  appState = S_LOAD_DIR;
}
void enterFolder(uint16_t newDirIndex) {
  if (dirStackPos < DEPTH_STACK) {
    dirStack[dirStackPos++] = { curDirIndex, cursor, scrollTop };
  }
  curDirIndex = newDirIndex; cursor = 0; scrollTop = 0;
  appState = S_LOAD_DIR;
}
void goBack() {
  if (dirStackPos == 0) {
    // вернуться к списку устройств
    appState = S_DEVICES; return;
  }
  StackFrame f = dirStack[--dirStackPos];
  curDirIndex = f.dirIndex; cursor = f.cursor; scrollTop = f.scrollTop;
  appState = S_LOAD_DIR;
}

// При Select на entry-Folder — проваливаемся.
// Чтобы узнать индекс дочерней директории, надо запросить FullEntry с descriptor.
// Но у нас childDir — последовательный (alight нумерует Dir_X enum'ом). Пока: в alight
// порядок papok в Settings совпадает с enum Dir_*. Для общего случая нужен parser desc.
//
// Упрощение: каждый Folder получает curDirIndex+offset через дозапрос descriptor'а.
// Пока что — просто пробуем dirIndex+1 если в root, иначе по порядку из descriptor'а.
//
// Для корректности добавлю отдельный запрос descriptor для folder и распарсю его.
// LEVCAN folder descriptor — uint16_t DirIndex (см. levcan_paraminternal.h).

bool fetchFolderDirIndex(uint8_t target, uint16_t dirIndex, uint16_t entryIdx, uint16_t* outDir) {
  // запросим entry заново с целью получить descriptor
  pending = { WK_ENTRY, target, dirIndex, entryIdx, millis() };
  EntryInfo& e = curDir.entries[entryIdx];
  e.hasDesc = false; e.descLen = 0;
  resetAllChannels();
  reqEntry(target, dirIndex, entryIdx);
  uint32_t start = millis();
  while (millis() - start < 800) {
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);
    if (e.hasDesc && e.descLen >= 2) break;
    delay(2);
  }
  pending.kind = WK_NONE;
  if (!e.hasDesc || e.descLen < 2) return false;
  *outDir = e.desc[0] | (e.desc[1] << 8);
  return true;
}

void handleSelect() {
  if (cursor >= curDir.entrySize) return;
  EntryInfo& e = curDir.entries[cursor];
  if (e.type == LCP_Folder) {
    drawLoading("descriptor");
    uint16_t childDir = 0xFFFF;
    if (!fetchFolderDirIndex(targetAddr, curDirIndex, cursor, &childDir)) {
      Serial.println("  failed to fetch child dir");
      return;
    }
    Serial.printf("  → folder %u\n", childDir);
    enterFolder(childDir);
  } else {
    // value-параметр — пока только показываем подробности; редактирование добавим позже
    Serial.printf("  Entry select: %s mode=0x%02X\n", e.name, e.mode);
  }
}

// ====== setup / loop ======
void setup() {
  Serial.begin(115200); delay(300);
  Serial.println("\n=== ESP32 LEVCAN browser v4 ===");
  initButtons();
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false);
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); display.println("LEVCAN browser"); display.println("init CAN..."); display.display();
  if (!initCAN()) { display.println("CAN FAIL"); display.display(); while(1) delay(1000); }
  Serial.printf("Buttons: BACK=%d UP=%d DOWN=%d SEL=%d\n", BTN_BACK, BTN_UP, BTN_DOWN, BTN_SELECT);
  appState = S_DISCOVER;
  discoverStartMs = millis();
  resetAllChannels();
  // Широковещательный запрос DeviceName выявит всех на шине.
  // NodeName запросим адресно на каждый ответивший узел (не все серверы отвечают на broadcast 0x388).
  broadcastDeviceNameQuery();
  lastBroadcastMs = millis();
}

void runLoopFrame() {
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);
}

void loop() {
  runLoopFrame();
  uint32_t now = millis();

  switch (appState) {
    case S_DISCOVER: {
      if (now - lastBroadcastMs > BROADCAST_PERIOD) {
        broadcastDeviceNameQuery();
        // Для уже найденных узлов, у кого нет NodeName, не ждём больше — попытаемся адресно.
        for (uint8_t k = 0; k < nodeCount; k++) {
          if (!nodes[k].hasNodeName) reqNodeName(nodes[k].addr);
        }
        lastBroadcastMs = now;
      }
      bool back = pollButton(BI_BACK), up = pollButton(BI_UP), down = pollButton(BI_DOWN), sel = pollButton(BI_SEL);
      (void)back; (void)up; (void)down;
      if (sel && nodeCount > 0 && (now - discoverStartMs) > DISCOVER_MIN_MS) {
        selectedNode = 0; appState = S_DEVICES;
      }
      drawDiscover();
      break;
    }
    case S_DEVICES: {
      if (pollButton(BI_UP) && selectedNode > 0) selectedNode--;
      if (pollButton(BI_DOWN) && selectedNode + 1 < (int)nodeCount) selectedNode++;
      if (pollButton(BI_BACK)) {
        // повторное обнаружение
        nodeCount = 0; selectedNode = -1;
        appState = S_DISCOVER; discoverStartMs = now;
        broadcastDeviceNameQuery(); lastBroadcastMs = now;
      }
      if (pollButton(BI_SEL) && selectedNode >= 0) enterDevice(selectedNode);
      drawDevices();
      break;
    }
    case S_LOAD_DIR: {
      drawLoading("menu...");
      bool ok = loadDir(targetAddr, curDirIndex);
      if (ok) { appState = S_BROWSE; lastLiveUpdMs = now; }
      else    { appState = S_DEVICES; }
      break;
    }
    case S_BROWSE: {
      if (pollButton(BI_UP)   && cursor > 0) cursor--;
      if (pollButton(BI_DOWN) && cursor + 1 < curDir.entrySize) cursor++;
      if (pollButton(BI_BACK)) goBack();
      if (pollButton(BI_SEL))  handleSelect();
      // live-обновление RO live-параметров
      if (now - lastLiveUpdMs > LIVE_PERIOD) {
        lastLiveUpdMs = now;
        refreshLiveValues(targetAddr, curDirIndex);
      }
      drawBrowse();
      break;
    }
  }
  delay(5);
}
