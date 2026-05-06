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
// OLED SSD1306 — I²C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_SDA 5
#define OLED_SCL 4
// CAN-трансивер TJA1050
#define CAN_TX GPIO_NUM_25
#define CAN_RX GPIO_NUM_26
// Кнопки (active-low, внутренние pull-up)
#define BTN_BACK   13
#define BTN_UP     14
#define BTN_DOWN   27
#define BTN_SELECT 33
// Внешний USB→UART конвертер для отладочного лога.
// Основной USB-порт (Serial = UART0, GPIO 1/3, через встроенный CP210x виден как /dev/ttyUSB0)
// в rc2.1 отведён под VT100-интерфейс меню, а отладка уходит на UART1:
//   • ESP32 GPIO 10 (TX1) → RX внешнего конвертера
//   • ESP32 GPIO  9 (RX1) ← TX внешнего конвертера
//   • GND общий с ESP32
// 115200-N-1, принимать любым терминалом (minicom/screen/picocom).
// ПРИМЕЧАНИЕ: на ESP32 GPIO 9/10 внутренне используются flash в QIO/QOUT режиме.
// На LOLIN32 flash подключен по DIO, поэтому эти пины свободны. UART1 перемапим на 9/10 явно.
#define DBG_UART_NUM    1
#define DBG_UART_TX     10
#define DBG_UART_RX     9
#define DBG_UART_BAUD   115200

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
static const uint16_t LCP_REQ_VALUE_SET      = 0x40;  // запись значения

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
#define DESC_BUF         48  // Decimal32_t=13, Int64_t/Uint64_t=24, запас на будущее
#define ASM_BUF          128
#define DEPTH_STACK      8

// ====== Опции ======
static const bool VERBOSE_RX  = true;
static const bool VERBOSE_CTS = false;
// #define ENABLE_FULL_WALKER 1   // раскомментируй чтобы по B+S при загрузке снять полный дамп

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Отладочный UART (на внешний USB→UART конвертер).
// На этот порт идёт весь отладочный вывод (Dbg.print*); штатный Serial резервируется за VT100-меню.
HardwareSerial Dbg(DBG_UART_NUM);

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

// ====== Ожидание CTS/ACK от сервера при multi-frame TX ======
// (определено заранее, до sendTcpToServer, который её использует)
struct TxWaitState {
  bool     active;
  uint8_t  src;
  uint16_t msgId;
  bool     gotCts;       // пришло RTR с rts=1, eom=0
  uint8_t  ctsParity;
  bool     gotFinalAck;  // пришло RTR с rts=0, eom=1
} txWait;

// ====== Forward declarations (используются в drawBrowse до своего определения) ======
bool isEditable(const EntryInfo& e);

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
bool reqEntry           (uint8_t target, uint16_t dir, uint16_t e) { return reqEntryCmd(target, dir, e, LCP_REQ_DATA_NAME);  }
bool reqEntryVariable   (uint8_t target, uint16_t dir, uint16_t e) { return reqEntryCmd(target, dir, e, LCP_REQ_VARIABLE);   }
bool reqEntryDescriptor (uint8_t target, uint16_t dir, uint16_t e) { return reqEntryCmd(target, dir, e, LCP_REQ_DESCRIPTOR); }
bool reqEntryText       (uint8_t target, uint16_t dir, uint16_t e) { return reqEntryCmd(target, dir, e, LCP_REQ_TEXT);       }
bool reqSysName(uint8_t target, uint16_t msgId) {
  // RTR-запрос (Request flag) на системное msgId. Если target=BROADCAST — отвечают все.
  // ВАЖНО: для нового запроса нужны RTS_CTS=0, EoM=0, Parity=0 (см. levcan.c строка 951:
  //   if (rxBuffered.header.Request) { if (RTS_CTS==0 && EoM==0) /*new request*/ ...
  // иначе наш запрос трактуется как CTS/EoM-ACK для несуществующего TX-объекта).
  LcHeader h{}; h.source=MY_ADDR; h.target=target; h.msgId=msgId;
  h.eom=0; h.parity=0; h.rts=0; h.prio=0;
  return sendFrame(packId(h), nullptr, 0, /*rtr=*/true);
}
bool reqDeviceName(uint8_t target) { return reqSysName(target, MSG_DEVICE_NAME); }
bool reqNodeName  (uint8_t target) { return reqSysName(target, MSG_NODE_NAME);   }
bool broadcastDeviceNameQuery() { return reqDeviceName(BROADCAST); }
bool broadcastNodeNameQuery()   { return reqNodeName(BROADCAST);   }

// Отправка пакета на 0x399 (ParametersRequest). Длина любая, multi-frame TCP если >8.
// Ответы CTS/ACK приходят RTR-кадрами от target на том же msgId.
bool sendTcpToServer(uint8_t target, uint16_t msgId, const uint8_t* data, uint16_t len) {
  // Подготовим слот ожидания CTS/ACK
  txWait = { true, target, msgId, false, 0, false };

  if (len <= 8) {
    // single-frame: rts=1, eom=1, parity=1
    LcHeader h{}; h.source=MY_ADDR; h.target=target; h.msgId=msgId;
    h.rts=1; h.eom=1; h.parity=1; h.prio=0;
    if (!sendFrame(packId(h), data, (uint8_t)len, false)) { txWait.active=false; return false; }
    // ждём final ACK (RTR rts=0 eom=1)
    uint32_t t0 = millis();
    while (millis() - t0 < 800 && !txWait.gotFinalAck) {
      twai_message_t m; while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);
      delay(2);
    }
    bool ok = txWait.gotFinalAck;
    txWait.active = false;
    return ok;
  }

  // multi-frame TCP. Протокол:
  //   frame0: rts=1, eom=0, parity=1, data[0..7]
  //   <- CTS RTR rts=1, eom=0, parity=ожидаемый паритет след.фрейма
  //   frameN: rts=0, eom=(последний?1:0), parity=ожидаемый
  //   <- final ACK RTR rts=0, eom=1
  uint16_t pos = 0;
  uint8_t  parity = 1;  // первый фрейм
  // frame 0
  {
    uint16_t take = 8;
    LcHeader h{}; h.source=MY_ADDR; h.target=target; h.msgId=msgId;
    h.rts=1; h.eom=0; h.parity=parity; h.prio=0;
    if (!sendFrame(packId(h), data + pos, (uint8_t)take, false)) { txWait.active=false; return false; }
    pos += take;
  }
  // ждём CTS
  {
    uint32_t t0 = millis();
    while (millis() - t0 < 800 && !txWait.gotCts) {
      twai_message_t m; while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);
      delay(2);
    }
    if (!txWait.gotCts) { txWait.active=false; Dbg.println("  TX: CTS timeout"); return false; }
  }
  // оставшиеся фреймы
  while (pos < len) {
    uint16_t take = (len - pos > 8) ? 8 : (len - pos);
    bool last = (pos + take >= len);
    parity = (~(((pos + 7) / 8))) & 1;  // паритет для текущего position
    LcHeader h{}; h.source=MY_ADDR; h.target=target; h.msgId=msgId;
    h.rts=0; h.eom=last?1:0; h.parity=parity; h.prio=0;
    if (!sendFrame(packId(h), data + pos, (uint8_t)take, false)) { txWait.active=false; return false; }
    pos += take;
    if (!last) {
      // ждём следующий CTS перед очередным блоком (сервер обычно разрешает сразу после первого CTS).
      // На практике для payload <=16 байт этот блок не выполняется.
      txWait.gotCts = false;
      uint32_t t0 = millis();
      while (millis() - t0 < 400 && !txWait.gotCts) {
        twai_message_t m; while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);
        delay(2);
      }
      if (!txWait.gotCts) { txWait.active=false; Dbg.println("  TX: mid CTS timeout"); return false; }
    }
  }
  // ждём final ACK
  {
    uint32_t t0 = millis();
    while (millis() - t0 < 800 && !txWait.gotFinalAck) {
      twai_message_t m; while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);
      delay(2);
    }
    bool ok = txWait.gotFinalAck;
    txWait.active = false;
    if (!ok) Dbg.println("  TX: final ACK timeout");
    return ok;
  }
}

// Собирает lc_value_set_t и шлёт его на сервер. Ответ (1 байт ErrorCode на 0x39A) ждём
// в вызывающем коде через setResult.received.
bool sendValueSet(uint8_t target, uint16_t dirIndex, uint16_t entryIndex,
                  const uint8_t* value, uint16_t valueSize) {
  uint16_t total = 6 + valueSize;
  if (total > 64) return false;
  uint8_t buf[64];
  buf[0] = (uint8_t)(LCP_REQ_VALUE_SET & 0xFF);
  buf[1] = (uint8_t)(LCP_REQ_VALUE_SET >> 8);
  buf[2] = (uint8_t)(dirIndex & 0xFF);
  buf[3] = (uint8_t)(dirIndex >> 8);
  buf[4] = (uint8_t)(entryIndex & 0xFF);
  buf[5] = (uint8_t)(entryIndex >> 8);
  for (uint16_t i = 0; i < valueSize; i++) buf[6+i] = value[i];
  return sendTcpToServer(target, MSG_PARAM_REQUEST, buf, total);
}

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
    if (c == 0xB0) { /* ° — выкидываем символ: в этом шрифте "*" слишком выделяется. Остаются "42C" / "42 C". */ }
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
  if (VERBOSE_CTS) Dbg.printf("    <- CTS to %u msg=0x%X par=%u rts=%u eom=%u\n", target, msgId, parity, rts, eom);
  return sendFrame(packId(h), nullptr, 0, /*rtr=*/true);
}

// ====== Состояние ожидания ======
enum WaitKind { WK_NONE, WK_DIR, WK_ENTRY, WK_VALUE, WK_SET };
struct {
  WaitKind kind;
  uint8_t  target;
  uint16_t dirIndex;
  uint16_t entryIndex;
  uint32_t startMs;
} pending;

// ====== Результат ValueSet (приходит в 0x39A как 1 байт ErrorCode) ======
struct {
  bool     received;
  uint8_t  errorCode;
} setResult;

// ====== Парсинг ======
void onDeviceName(uint8_t srcAddr, const uint8_t* d, uint16_t len) {
  int i = findOrAddNode(srcAddr);
  if (i < 0) return;
  uint16_t n = len; for (uint16_t k=0; k<n; k++) if (d[k]==0) { n=k; break; }
  cp1251ToAscii(d, n, nodes[i].deviceName, NAME_BUF);
  nodes[i].hasDeviceName = true;
  nodes[i].lastSeenMs = millis();
  Dbg.printf("  Node %u DeviceName=\"%s\"\n", srcAddr, nodes[i].deviceName);
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
  Dbg.printf("  Node %u NodeName=\"%s\"\n", srcAddr, nodes[i].nodeName);
}

void onDirData(uint16_t dirIdx, const uint8_t* d, uint16_t len) {
  if (len < 6) return;
  uint16_t entrySize = d[0] | (d[1] << 8);
  uint16_t nameSize  = d[2] | (d[3] << 8);
  uint16_t actualDir = d[4] | (d[5] << 8);
  if (actualDir != dirIdx) { Dbg.printf("  DirData dir mismatch %u/%u\n", actualDir, dirIdx); return; }
  if (entrySize > MAX_ENTRIES_DIR || nameSize == 0 || nameSize > NAME_BUF) {
    Dbg.printf("  DirData bogus size es=%u ns=%u\n", entrySize, nameSize); return;
  }
  curDir.known = true;
  curDir.entrySize = entrySize;
  curDir.nameSize  = nameSize;
  Dbg.printf("  DirData[%u]: entries=%u\n", dirIdx, entrySize);
}

void onEntryData(uint16_t expectedEntry, const uint8_t* d, uint16_t len) {
  if (len < 4 || expectedEntry >= MAX_ENTRIES_DIR) return;
  uint16_t respEntry = d[2] | (d[3] << 8);
  if (respEntry != expectedEntry) { Dbg.printf("  Entry idx mismatch %u/%u\n", respEntry, expectedEntry); return; }
  EntryInfo& e = curDir.entries[expectedEntry];
  e.type = d[0]; e.mode = d[1]; e.entryIdx = respEntry;
  if (len >= 6)  e.varSize  = d[4] | (d[5] << 8);
  if (len >= 8)  e.descSize = d[6] | (d[7] << 8);
  if (len >= 10) e.textSize = d[8] | (d[9] << 8);
  e.hasData = true;
  Dbg.printf("  EntryData[%u]: type=%s mode=0x%02X varSz=%u\n",
                expectedEntry, typeName(e.type), e.mode, e.varSize);
}

void onName(int16_t entryIdx, const uint8_t* d, uint16_t len) {
  uint16_t n = len; for (uint16_t i=0;i<n;i++) if (d[i]==0) { n=i; break; }
  char buf[NAME_BUF]; cp1251ToAscii(d, n, buf, sizeof(buf));
  if (entryIdx < 0) {
    strncpy(curDir.name, buf, NAME_BUF-1); curDir.name[NAME_BUF-1]=0;
    Dbg.printf("  DirName: \"%s\"\n", curDir.name);
  }
  else if (entryIdx < MAX_ENTRIES_DIR) {
    EntryInfo& e = curDir.entries[entryIdx];
    strncpy(e.name, buf, NAME_BUF-1); e.name[NAME_BUF-1]=0; e.hasName = true;
    Dbg.printf("  EntryName[%u]: \"%s\"\n", (unsigned)entryIdx, e.name);
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
  Dbg.printf("  EntryValue[%u]: len=%u\n", (unsigned)entryIdx, len);
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
  } else if (pending.kind == WK_SET) {
    // Ответ на ValueSet приходит в 0x39A (MSG_PARAM_DATA = LC_SYS_ParametersData)
    // и содержит 1 байт ErrorCode (lc_request_error_t)
    if (msgId == MSG_PARAM_DATA && c.pos >= 1) {
      setResult.errorCode = c.buf[0];
      setResult.received  = true;
      Dbg.printf("  ValueSet response: errorCode=%u\n", c.buf[0]);
    }
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

  // RTR-кадры: это CTS/ACK от сервера. Если мы сейчас ведём multi-frame TX, пробуем поймать.
  if (m.rtr) {
    if (txWait.active && h.source == txWait.src && h.msgId == txWait.msgId) {
      if (h.rts == 1 && h.eom == 0) { txWait.gotCts = true; txWait.ctsParity = h.parity; }
      else if (h.rts == 0 && h.eom == 1) { txWait.gotFinalAck = true; }
      if (VERBOSE_RX) Dbg.printf("    RX RTR src=%u msg=0x%X rts=%u eom=%u par=%u\n",
                                    h.source, h.msgId, h.rts, h.eom, h.parity);
    }
    return;
  }

  if (VERBOSE_RX) {
    Dbg.printf("    RX 0x%X src=%u msg=0x%X eom=%u par=%u rts=%u dlc=%u\n",
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
bool entryDescReady() {
  EntryInfo& e = curDir.entries[pending.entryIndex];
  return e.hasDesc;
}
bool entryTextReady() {
  EntryInfo& e = curDir.entries[pending.entryIndex];
  return e.hasText;
}

bool loadDir(uint8_t target, uint16_t dirIndex) {
  Dbg.printf("[loadDir target=%u dir=%u]\n", target, dirIndex);
  resetAllChannels();
  clearDir();
  // 1) Запрос DirInfo (data + name)
  pending = { WK_DIR, target, dirIndex, 0, millis() };
  if (!reqDir(target, dirIndex)) { Dbg.println("  reqDir TX fail"); return false; }
  if (!waitFor(1500, dirNameReady)) { Dbg.println("  dir timeout"); pending.kind=WK_NONE; return false; }
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
    for (int retry = 0; retry < 2 && !ok; retry++) {
      if (retry > 0) resetAllChannels();
      reqEntry(target, dirIndex, i);
      ok = waitFor(800, entryNameReady);
      if (!ok) Dbg.printf("    retry entry %u data/name (hasData=%u hasName=%u)\n",
                             i, curDir.entries[i].hasData, curDir.entries[i].hasName);
    }
    if (!ok) {
      Dbg.printf("  entry %u: skip data/name after retries\n", i);
      pending.kind = WK_NONE; delay(20); continue;
    }

    EntryInfo& e = curDir.entries[i];

    // Шаг А': Descriptor.
    //   Decimal32 — забираем Decimals (1 байт по смещению 12).
    //   Enum     — забираем Min (uint32, смещение 0 в LCP_Enum_t).
    //   Int/Uint/Int64/Uint64 — забираем {Min, Max, Step} для редактора.
    //   Bool дескриптора не имеет (DescSize=0 в pbool macro).
    bool needDesc = (e.type == LCP_Decimal32) || (e.type == LCP_Enum)
                 || (e.type == LCP_Int32)     || (e.type == LCP_Uint32)
                 || (e.type == LCP_Int64)     || (e.type == LCP_Uint64);
    if (needDesc) {
      delay(5);
      bool dok = false;
      for (int retry = 0; retry < 2 && !dok; retry++) {
        if (retry > 0) resetAllChannels();
        reqEntryDescriptor(target, dirIndex, i);
        dok = waitFor(800, entryDescReady);
        if (!dok) Dbg.printf("    retry entry %u desc\n", i);
      }
      if (!dok) Dbg.printf("  entry %u: skip desc after retries\n", i);
    }

    // Шаг А'': Text.
    //   Enum/Bool — список вариантов через '\n'.
    //   Int32/Uint32/Int64/Uint64/Decimal32 — format-строка ("%d%%", "%u sec", "%s*C" и т.п.).
    //
    // ВАЖНО: alight server считает TextSize = strlen(name) + strlen(textData).
    // Если у entry textData = NULL (например, у большинства Bool — встроенные off/ON),
    // сервер всё равно вернёт textSize = len(name), но на REQ_TEXT не пришлёт ничего.
    // Поэтому реальное наличие Text определяем как textSize > strlen(name).
    uint16_t nameLen = strlen(e.name);
    bool hasRealText = (e.textSize > nameLen);
    bool needText = hasRealText && (
        e.type == LCP_Enum   || e.type == LCP_Bool   ||
        e.type == LCP_Int32  || e.type == LCP_Uint32 ||
        e.type == LCP_Int64  || e.type == LCP_Uint64 ||
        e.type == LCP_Decimal32);
    if (needText) {
      delay(5);
      bool tok = false;
      // 1 ретрай при 600 мс — text опционален, fallback есть.
      for (int retry = 0; retry < 1 && !tok; retry++) {
        if (retry > 0) resetAllChannels();
        reqEntryText(target, dirIndex, i);
        tok = waitFor(600, entryTextReady);
        if (!tok) Dbg.printf("    skip entry %u text (no response)\n", i);
      }
    }

    // Шаг Б: Value (если имеет смысл)
    bool needValue = (e.type != LCP_Folder) && (e.type != LCP_Label)
                  && ((e.mode & 0x02) == 0)  // не WriteOnly
                  && (e.varSize > 0);
    if (needValue) {
      delay(5);  // лёгкая пауза, чтобы alight разгрузил TX-очередь
      bool vok = false;
      for (int retry = 0; retry < 2 && !vok; retry++) {
        if (retry > 0) resetAllChannels();
        reqEntryVariable(target, dirIndex, i);
        vok = waitFor(800, entryValueReady);
        if (!vok) Dbg.printf("    retry entry %u value\n", i);
      }
      if (!vok) Dbg.printf("  entry %u: skip value after retries\n", i);
    }
    pending.kind = WK_NONE;
    delay(5);
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
// Виртуальные нажатия от VT100-терминала. kbPump() выставляет флаг,
// pollButton() подхватывает и отдаёт однократно — так вся логика UI остаётся без изменений.
static volatile bool kbInjected[4] = {false, false, false, false};

// возвращает true один раз на нажатие (фронт)
bool pollButton(int i) {
  // Клавиатура VT100 — обрабатывается в приоритете.
  if (kbInjected[i]) { kbInjected[i] = false; return true; }

  uint8_t s = digitalRead(btns[i].pin);
  uint32_t now = millis();
  if (s != btns[i].state && (now - btns[i].lastChangeMs) > 30) {
    btns[i].lastChangeMs = now; btns[i].state = s;
    if (s == 0) { btns[i].pressed = true; return true; }
  }
  return false;
}

// ====== Клавиатура VT100 (Serial → виртуальные кнопки) ======
// Читаем из штатного Serial. Распознаём:
//   ENTER (CR или LF) → SEL
//   ESC без продолжения в течение 50 мс → BACK
//   ESC [ A → UP, ESC [ B → DOWN
//   ESC [ C / D — пропускаем (нет горизонтальных движений)
//   k/j (как vim) → UP/DOWN — удобно на некоторых терминалах
static int       kbEscState   = 0;        // 0=normal, 1=получен ESC, 2=получен ESC+[
static uint32_t  kbEscStartMs = 0;

void kbPump() {
  // Тайм-аут одиночного ESC: если прошло >50мс и продолжения нет — это был ESC (BACK).
  if (kbEscState == 1 && (millis() - kbEscStartMs) > 50) {
    kbInjected[BI_BACK] = true;
    kbEscState = 0;
  }

  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch < 0) break;

    if (kbEscState == 0) {
      switch (ch) {
        case 0x1B:                                  // ESC
          kbEscState   = 1;
          kbEscStartMs = millis();
          break;
        case '\r': case '\n':
          kbInjected[BI_SEL] = true;
          break;
        case 'k': case 'K':
          kbInjected[BI_UP] = true;
          break;
        case 'j': case 'J':
          kbInjected[BI_DOWN] = true;
          break;
        default: break;                             // всё остальное игнорируем
      }
    } else if (kbEscState == 1) {
      if (ch == '[') {
        kbEscState = 2;
      } else {
        // ESC + что-то осмысленное, но не [ — трактуем как BACK и обработаем символ заново.
        kbInjected[BI_BACK] = true;
        kbEscState = 0;
        // Одновременность редкая, поэтому просто теряем символ ch — это приемлемо.
      }
    } else if (kbEscState == 2) {
      switch (ch) {
        case 'A': kbInjected[BI_UP]   = true; break;
        case 'B': kbInjected[BI_DOWN] = true; break;
        default:  break;                            // C/D/H/F и др. пропускаем
      }
      kbEscState = 0;
    }
  }
}

// ====== UI ======
enum AppState { S_DISCOVER, S_DEVICES, S_LOAD_DIR, S_BROWSE, S_EDIT, S_EDIT_APPLY };
AppState appState = S_DISCOVER;

// ====== Состояние редактора значения ======
#define EDIT_MAX_OPTIONS 256
struct {
  uint16_t entryIdx;       // какой entry редактируем (в curDir)
  uint8_t  type;           // LCP_Type
  uint16_t varSize;        // 1/2/4 байт
  bool     isSigned;       // для численных типов
  // диапазон и шаг (signed-объём покрывает и unsigned до 2^31-1)
  int32_t  vmin, vmax, vstep;
  uint16_t count;          // сколько вариантов (1..EDIT_MAX_OPTIONS)
  uint16_t cursor;
  uint16_t scrollTop;
  // для Decimal32: Decimals
  uint8_t  decimals;
  // каждый вариант выражается через (vmin + i*vstep) — это лениво вычисляемый список, памяти не храним.
  // Опция “Отмена” обрабатывается кнопкой Back, отдельный пункт не нужен.
  // format-строка из e.text (для численных), или список лейблов (Enum/Bool) — берём
  // прямо из curDir.entries[entryIdx].text по необходимости.
} editor;
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
// Скопировать в out подстроку с индексом idx из src, где разделитель '\n'.
// Возвращает true если подстрока найдена.
static bool pickEnumLabel(const char* src, uint32_t idx, char* out, size_t outSize) {
  if (!src || !src[0]) return false;
  const char* p = src;
  for (uint32_t k = 0; k < idx; k++) {
    const char* nl = strchr(p, '\n');
    if (!nl) return false;
    p = nl + 1;
  }
  const char* end = strchr(p, '\n');
  size_t len = end ? (size_t)(end - p) : strlen(p);
  if (len >= outSize) len = outSize - 1;
  memcpy(out, p, len);
  out[len] = 0;
  return true;
}

// Проверяет безопасность format-строки из alight (из entry.Text для LCP_Decimal/LCP_Int/LCP_Uint).
// Разрешаем ровно один конвертер — буква из mode (одна из 'd','i','u','s'),
// флаги/ширину/длину игнорируем. %% разрешён.
// Возвращает true, если безопасна и в *modeOut ложит 'd' / 'u' / 's'.
static bool checkSafeFormat(const char* fmt, char* modeOut) {
  if (!fmt || !fmt[0]) return false;
  int converters = 0; char mode = 0;
  for (const char* p = fmt; *p; p++) {
    if (*p != '%' || !p[1]) continue;
    if (p[1] == '%') { p++; continue; }
    const char* q = p + 1;
    while (*q == '-' || *q == '+' || *q == ' ' || *q == '0' || *q == '#') q++;
    while (*q >= '0' && *q <= '9') q++;
    if (*q == '.') { q++; while (*q >= '0' && *q <= '9') q++; }
    while (*q == 'l' || *q == 'h') q++;
    char c = *q;
    if (c == 'd' || c == 'i')      mode = 'd';
    else if (c == 'u')             mode = 'u';
    else if (c == 's')             mode = 's';
    else                            return false;
    converters++;
    p = q;
  }
  if (converters != 1) return false;
  if (modeOut) *modeOut = mode;
  return true;
}

// Применить format-строку из alight к уже отформатированному числу/строке.
// Если fmt безопасен — пишем в out и возвращаем true. Иначе false.
static bool applyTextFormat(const char* fmt, long ival, const char* sval, char* out, size_t outSize) {
  char mode = 0;
  if (!checkSafeFormat(fmt, &mode)) return false;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
  if (mode == 's') snprintf(out, outSize, fmt, sval ? sval : "");
  else             snprintf(out, outSize, fmt, ival);
#pragma GCC diagnostic pop
  return true;
}

void formatValue(const EntryInfo& e, char* out, size_t outSize) {
  if (!e.hasValue) { out[0] = '?'; out[1] = 0; return; }
  switch (e.type) {
    case LCP_Bool: {
      // По протоколу: в TextData две строки через '\n' (первая — false, вторая — true).
      uint32_t idx = e.value[0] ? 1 : 0;
      if (e.hasText && pickEnumLabel(e.text, idx, out, outSize)) break;
      snprintf(out, outSize, "%s", idx ? "ON" : "off");
      break;
    }
    case LCP_Int32: {
      int32_t v = (int32_t)(e.value[0] | (e.value[1]<<8) | (e.value[2]<<16) | (e.value[3]<<24));
      char raw[16]; snprintf(raw, sizeof(raw), "%ld", (long)v);
      if (e.hasText && applyTextFormat(e.text, (long)v, raw, out, outSize)) break;
      snprintf(out, outSize, "%s", raw);
      break;
    }
    case LCP_Uint32: {
      uint32_t v = (uint32_t)(e.value[0] | (e.value[1]<<8) | (e.value[2]<<16) | (e.value[3]<<24));
      char raw[16]; snprintf(raw, sizeof(raw), "%lu", (unsigned long)v);
      if (e.hasText && applyTextFormat(e.text, (long)v, raw, out, outSize)) break;
      snprintf(out, outSize, "%s", raw);
      break;
    }
    case LCP_Decimal32: {
      // Variable: int8/int16/int32 со знаком (судим по e.varSize).
      // Descriptor: LCP_Decimal32_t = int32 Min, int32 Max, int32 Step, uint8 Decimals (всего 13 байт).
      // Decimals = кол-во знаков после запятой, делитель = 10^Decimals.
      int32_t v = 0;
      if      (e.valueLen >= 4) { v = (int32_t)((uint32_t)e.value[0] | ((uint32_t)e.value[1]<<8) |
                                                 ((uint32_t)e.value[2]<<16) | ((uint32_t)e.value[3]<<24)); }
      else if (e.valueLen >= 2) { int16_t s = (int16_t)((uint16_t)e.value[0] | ((uint16_t)e.value[1]<<8)); v = s; }
      else if (e.valueLen >= 1) { v = (int8_t)e.value[0]; }
      uint8_t decimals = 0;
      if (e.hasDesc && e.descLen >= 13) decimals = e.desc[12];
      // Сначала формируем raw-строку без единиц (она же пойдёт в %s).
      char raw[20];
      if (decimals == 0) {
        snprintf(raw, sizeof(raw), "%ld", (long)v);
      } else {
        int32_t div = 1; for (uint8_t k = 0; k < decimals; k++) div *= 10;
        int32_t whole = v / div;
        int32_t frac  = v % div; if (frac < 0) frac = -frac;
        if (decimals > 6) decimals = 6;
        if (v < 0 && whole == 0) snprintf(raw, sizeof(raw), "-0.%0*ld", (int)decimals, (long)frac);
        else                     snprintf(raw, sizeof(raw), "%ld.%0*ld", (long)whole, (int)decimals, (long)frac);
      }
      // Если в параметре есть format-строка (напр. "%s*C") — применим её к raw.
      if (e.hasText && applyTextFormat(e.text, (long)v, raw, out, outSize)) break;
      snprintf(out, outSize, "%s", raw);
      break;
    }
    case LCP_Float: {
      float f; memcpy(&f, e.value, 4);
      snprintf(out, outSize, "%.2f", f); break;
    }
    case LCP_Enum: {
      // Целое без знака по размеру varSize, индекс = (val - Min). Min лежит в дескрипторе (offset 0, uint32 LE).
      uint32_t v = 0;
      if      (e.valueLen >= 4) v = (uint32_t)e.value[0] | ((uint32_t)e.value[1]<<8) | ((uint32_t)e.value[2]<<16) | ((uint32_t)e.value[3]<<24);
      else if (e.valueLen >= 2) v = (uint32_t)e.value[0] | ((uint32_t)e.value[1]<<8);
      else if (e.valueLen >= 1) v = e.value[0];
      uint32_t minv = 0;
      if (e.hasDesc && e.descLen >= 4) {
        minv = (uint32_t)e.desc[0] | ((uint32_t)e.desc[1]<<8) | ((uint32_t)e.desc[2]<<16) | ((uint32_t)e.desc[3]<<24);
      }
      uint32_t idx = (v >= minv) ? (v - minv) : 0;
      if (e.hasText && pickEnumLabel(e.text, idx, out, outSize)) break;
      snprintf(out, outSize, "%lu", (unsigned long)v);
      break;
    }
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
    // Префикс рисуем только для Folder/Label.
    // Для обычных пунктов освобождённый столбец отдаём имени.
    bool hasPrefix = (e.type == LCP_Folder) || (e.type == LCP_Label);
    char prefix = (e.type == LCP_Folder) ? '>' : (e.type == LCP_Label ? '#' : ' ');

    // Сначала формируем значение (до 9 символов), имя занимает всё что осталось.
    char val[14] = "";
    if (e.hasValue) formatValue(e, val, sizeof(val));
    char vs[10];
    {
      size_t L = strlen(val);
      const size_t VMAX = 9;
      if (L <= VMAX) { strcpy(vs, val); }
      else           { memcpy(vs, val, VMAX-1); vs[VMAX-1] = '~'; vs[VMAX] = 0; }
    }
    size_t vlen = strlen(vs);

    // Общая ширина окна = 21 символ. Из них вычитаем prefix (если есть)
    // и один разделительный пробел между именем и значением (если в пункте есть значение).
    int total   = 21;
    int prefixW = hasPrefix ? 1 : 0;
    int sepW    = (vlen > 0) ? 1 : 0;
    int nameMax = total - prefixW - sepW - (int)vlen;
    if (nameMax < 6)  nameMax = 6;
    if (nameMax > 20) nameMax = 20;
    char nm[24];
    {
      size_t L = strlen(e.name);
      if ((int)L <= nameMax) { strcpy(nm, e.name); }
      else                   { memcpy(nm, e.name, nameMax - 1); nm[nameMax - 1] = '~'; nm[nameMax] = 0; }
    }
    int valWidth = total - prefixW - nameMax;   // включает сепаратор
    if (valWidth < 0) valWidth = 0;
    if (hasPrefix) display.printf("%c%-*s%*s", prefix, nameMax, nm, valWidth, vs);
    else           display.printf("%-*s%*s",            nameMax, nm, valWidth, vs);
  }
  display.setTextColor(SSD1306_WHITE);
  display.fillRect(0, SCREEN_HEIGHT-9, SCREEN_WIDTH, 9, SSD1306_BLACK);
  // Подсказка по кнопке Select зависит от типа текущего пункта:
  //   Folder → S=open, редактируемый → S=edit, иначе (просто read-only лист) — без подсказки.
  const char* selHint = "";
  if (cursor < curDir.entrySize) {
    const EntryInfo& ec = curDir.entries[cursor];
    if (ec.type == LCP_Folder)   selHint = " S=open";
    else if (isEditable(ec))     selHint = " S=edit";
  }
  char foot[32]; snprintf(foot, sizeof(foot), "%u/%u %s%s", cursor+1, curDir.entrySize,
                          dirStackPos > 0 ? "B=back" : "B=devs", selHint);
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

// Можно ли редактировать entry?
bool isEditable(const EntryInfo& e) {
  if (e.type == LCP_Folder || e.type == LCP_Label || e.type == LCP_String) return false;
  if (e.type == LCP_Float || e.type == LCP_Double || e.type == LCP_Bitfield32) return false; // пока не поддерживаем
  if (e.mode & LCP_MODE_RO) return false;        // ReadOnly
  if (e.mode & LCP_MODE_WO) return false;        // пока не поддерживаем WriteOnly
  if (e.varSize == 0) return false;
  return true;
}

// Разобрать {Min, Max, Step} из дескриптора. Для Decimal32 также берём Decimals.
// Для Bool/Enum выставляем min/max вручную.
bool prepareEditor(uint16_t entryIdx) {
  EntryInfo& e = curDir.entries[entryIdx];
  editor.entryIdx = entryIdx;
  editor.type = e.type;
  editor.varSize = e.varSize;
  editor.cursor = 0;
  editor.scrollTop = 0;
  editor.decimals = 0;
  editor.isSigned = false;

  auto rd_i32 = [&](uint16_t off, int32_t* out) -> bool {
    if (e.descLen < off + 4) return false;
    *out = (int32_t)((uint32_t)e.desc[off] | ((uint32_t)e.desc[off+1] << 8)
                   | ((uint32_t)e.desc[off+2] << 16) | ((uint32_t)e.desc[off+3] << 24));
    return true;
  };

  switch (e.type) {
    case LCP_Bool:
      editor.vmin = 0; editor.vmax = 1; editor.vstep = 1; editor.count = 2; break;
    case LCP_Enum: {
      // в дескрипторе LCP_Enum_t = {Min:u32, Size:u32}. Size = кол-во вариантов.
      int32_t mn=0, sz=0;
      if (!rd_i32(0, &mn) || !rd_i32(4, &sz)) return false;
      if (sz <= 0) sz = 1;
      if (sz > EDIT_MAX_OPTIONS) sz = EDIT_MAX_OPTIONS;
      editor.vmin = mn; editor.vmax = mn + sz - 1; editor.vstep = 1; editor.count = (uint16_t)sz;
      break;
    }
    case LCP_Int32: case LCP_Int64: {
      editor.isSigned = true;
      int32_t mn,mx,st;
      if (!rd_i32(0,&mn) || !rd_i32(4,&mx) || !rd_i32(8,&st)) return false;
      if (st <= 0) st = 1;
      editor.vmin = mn; editor.vmax = mx; editor.vstep = st;
      int64_t cnt = ((int64_t)mx - (int64_t)mn) / st + 1;
      if (cnt < 1) cnt = 1; if (cnt > EDIT_MAX_OPTIONS) cnt = EDIT_MAX_OPTIONS;
      editor.count = (uint16_t)cnt;
      break;
    }
    case LCP_Uint32: case LCP_Uint64: {
      int32_t mn,mx,st;
      if (!rd_i32(0,&mn) || !rd_i32(4,&mx) || !rd_i32(8,&st)) return false;
      if (st <= 0) st = 1;
      editor.vmin = mn; editor.vmax = mx; editor.vstep = st;
      int64_t cnt = ((int64_t)(uint32_t)mx - (int64_t)(uint32_t)mn) / st + 1;
      if (cnt < 1) cnt = 1; if (cnt > EDIT_MAX_OPTIONS) cnt = EDIT_MAX_OPTIONS;
      editor.count = (uint16_t)cnt;
      break;
    }
    case LCP_Decimal32: {
      editor.isSigned = true;
      int32_t mn,mx,st;
      if (!rd_i32(0,&mn) || !rd_i32(4,&mx) || !rd_i32(8,&st)) return false;
      if (st <= 0) st = 1;
      editor.vmin = mn; editor.vmax = mx; editor.vstep = st;
      if (e.descLen >= 13) editor.decimals = e.desc[12];
      int64_t cnt = ((int64_t)mx - (int64_t)mn) / st + 1;
      if (cnt < 1) cnt = 1; if (cnt > EDIT_MAX_OPTIONS) cnt = EDIT_MAX_OPTIONS;
      editor.count = (uint16_t)cnt;
      break;
    }
    default:
      return false;
  }

  // Подвинем курсор на текущее значение, если оно известно.
  if (e.hasValue) {
    int32_t cur = 0;
    if      (e.valueLen >= 4) cur = (int32_t)((uint32_t)e.value[0] | ((uint32_t)e.value[1]<<8) | ((uint32_t)e.value[2]<<16) | ((uint32_t)e.value[3]<<24));
    else if (e.valueLen >= 2) cur = editor.isSigned ? (int32_t)(int16_t)((uint16_t)e.value[0] | ((uint16_t)e.value[1]<<8))
                                                    : (int32_t)((uint16_t)e.value[0] | ((uint16_t)e.value[1]<<8));
    else if (e.valueLen >= 1) cur = editor.isSigned ? (int32_t)(int8_t)e.value[0] : (int32_t)e.value[0];
    if (editor.vstep > 0 && cur >= editor.vmin && cur <= editor.vmax) {
      uint32_t idx = (uint32_t)((cur - editor.vmin) / editor.vstep);
      if (idx < editor.count) editor.cursor = (uint16_t)idx;
    }
  }
  return true;
}

// Сформировать в out текст i-го варианта
void formatEditorOption(uint16_t i, char* out, size_t outSize) {
  EntryInfo& e = curDir.entries[editor.entryIdx];
  int32_t v = editor.vmin + (int32_t)i * editor.vstep;
  switch (editor.type) {
    case LCP_Bool: {
      uint32_t idx = v ? 1 : 0;
      if (e.hasText && pickEnumLabel(e.text, idx, out, outSize)) return;
      snprintf(out, outSize, "%s", idx ? "ON" : "off");
      return;
    }
    case LCP_Enum: {
      uint32_t idx = (uint32_t)((v - editor.vmin) / editor.vstep);
      if (e.hasText && pickEnumLabel(e.text, idx, out, outSize)) return;
      snprintf(out, outSize, "%ld", (long)v); return;
    }
    case LCP_Decimal32: {
      uint8_t d = editor.decimals; if (d > 6) d = 6;
      char raw[20];
      if (d == 0) { snprintf(raw, sizeof(raw), "%ld", (long)v); }
      else {
        int32_t div = 1; for (uint8_t k=0;k<d;k++) div *= 10;
        int32_t whole = v / div, frac = v % div; if (frac < 0) frac = -frac;
        if (v < 0 && whole == 0) snprintf(raw, sizeof(raw), "-0.%0*ld", (int)d, (long)frac);
        else                     snprintf(raw, sizeof(raw), "%ld.%0*ld", (long)whole, (int)d, (long)frac);
      }
      if (e.hasText && applyTextFormat(e.text, (long)v, raw, out, outSize)) return;
      snprintf(out, outSize, "%s", raw);
      return;
    }
    default: {
      // Численные (Int32/Uint32/Int64/Uint64): сначала raw, затем format-строка из e.text.
      char raw[20];
      if (editor.isSigned) snprintf(raw, sizeof(raw), "%ld", (long)v);
      else                 snprintf(raw, sizeof(raw), "%lu", (unsigned long)(uint32_t)v);
      if (e.hasText && applyTextFormat(e.text, (long)v, raw, out, outSize)) return;
      snprintf(out, outSize, "%s", raw);
      return;
    }
  }
}

// Применение выбранного значения.
bool applyEditorValue() {
  EntryInfo& e = curDir.entries[editor.entryIdx];
  int32_t v = editor.vmin + (int32_t)editor.cursor * editor.vstep;
  uint8_t bytes[8] = {0};
  uint16_t sz = editor.varSize;
  if (sz > 8) sz = 8;
  // выполним LEупаковку в нужный размер (сигнал/без) — сервер сам правильно разопьёт по типу.
  uint32_t u = (uint32_t)v;
  for (uint16_t k = 0; k < sz; k++) bytes[k] = (uint8_t)(u >> (8*k));
  // для sz>4 расширим знаком (Int64/Uint64): повторяем знаковый октет
  if (sz > 4) {
    uint8_t pad = (editor.isSigned && (v < 0)) ? 0xFF : 0x00;
    for (uint16_t k = 4; k < sz; k++) bytes[k] = pad;
  }

  setResult.received = false;
  pending = { WK_SET, targetAddr, curDirIndex, editor.entryIdx, millis() };
  bool sent = sendValueSet(targetAddr, curDirIndex, editor.entryIdx, bytes, sz);
  if (!sent) { Dbg.println("  sendValueSet failed"); pending.kind = WK_NONE; return false; }
  // ждём ответ ErrorCode (1 байт на 0x39A)
  uint32_t t0 = millis();
  while (millis() - t0 < 1000 && !setResult.received) {
    twai_message_t m; while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);
    delay(2);
  }
  pending.kind = WK_NONE;
  if (!setResult.received) { Dbg.println("  ValueSet: response timeout"); return false; }
  Dbg.printf("  ValueSet ok=%u err=%u\n", setResult.errorCode == 0, setResult.errorCode);
  // Обновим локально отображаемое значение, если сервер принял.
  if (setResult.errorCode == 0) {
    e.valueLen = (uint8_t)sz;
    for (uint16_t k = 0; k < sz; k++) e.value[k] = bytes[k];
    e.hasValue = true;
    // Перечитаем с сервера — сервер мог обрезать значение по Min/Max,
    // и для обычных (не LiveUpdate) параметров без этого UI покажет старое.
    e.hasValue = false;
    pending = { WK_VALUE, targetAddr, curDirIndex, editor.entryIdx, millis() };
    reqEntryVariable(targetAddr, curDirIndex, editor.entryIdx);
    waitFor(500, [](){ return curDir.entries[editor.entryIdx].hasValue; });
    pending.kind = WK_NONE;
  }
  return setResult.errorCode == 0;
}

void handleSelect() {
  if (cursor >= curDir.entrySize) return;
  EntryInfo& e = curDir.entries[cursor];
  if (e.type == LCP_Folder) {
    // Для Folder дочерний dirIndex уже лежит в entry->VarSize и приходит в lc_entry_data_t
    // (см. levcan_paramserver.c:264 «dindex = entry->VarSize»). Никакой descriptor у folder нет.
    uint16_t childDir = e.varSize;
    Dbg.printf("  → folder %u\n", childDir);
    enterFolder(childDir);
  } else if (isEditable(e)) {
    if (prepareEditor(cursor)) {
      Dbg.printf("  ✎ edit %s: type=%u min=%ld max=%ld step=%ld count=%u\n",
                    e.name, e.type, (long)editor.vmin, (long)editor.vmax, (long)editor.vstep, editor.count);
      appState = S_EDIT;
    } else {
      Dbg.printf("  edit prepare failed for %s\n", e.name);
    }
  } else {
    Dbg.printf("  Entry select: %s mode=0x%02X (read-only)\n", e.name, e.mode);
  }
}

// ====== Отрисовка редактора ======
void drawEditor() {
  display.clearDisplay();
  EntryInfo& e = curDir.entries[editor.entryIdx];
  drawHeader(e.name);

  const uint8_t VIS_ROWS = 4;
  if (editor.cursor < editor.scrollTop) editor.scrollTop = editor.cursor;
  if (editor.cursor >= editor.scrollTop + VIS_ROWS) editor.scrollTop = editor.cursor - VIS_ROWS + 1;

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  for (uint8_t r = 0; r < VIS_ROWS; r++) {
    uint16_t i = editor.scrollTop + r;
    if (i >= editor.count) break;
    int16_t y = 12 + r * 10;
    bool sel = (i == editor.cursor);
    if (sel) display.fillRect(0, y - 1, SCREEN_WIDTH, 9, SSD1306_WHITE);
    display.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE);
    char buf[28]; formatEditorOption(i, buf, sizeof(buf));
    display.setCursor(2, y);
    display.print(buf);
  }

  char foot[32];
  snprintf(foot, sizeof(foot), "%u/%u  S=apply B=cancel", editor.cursor + 1, editor.count);
  drawFooter(foot);
  display.display();
}

// ====== VT100 панель (дублирует OLED в терминал 80×24) ======
// Отображение одновременное с OLED. Шапка содержит имя узла (DeviceName/NodeName)
// и вторым рядом — контекст (название директории или имя редактируемого параметра).
// Строки меню подрезаются/раскладываются ровно так же как на OLED, только окно шире — 78 символов.
// Выделение выбранной строки — инверсия (ESC[7m).
//
// Для экономии пропускной способности и мерцания экран перерисовывается только при изменении signature
// (хэш от всех видимых данных).

static const uint8_t  VT_COLS = 80;
static const uint8_t  VT_ROWS = 24;
static const uint8_t  VT_LIST_TOP    = 4;            // первая строка меню (строки 1..3 — шапка + разделитель)
static const uint8_t  VT_LIST_HEIGHT = 18;           // 18 видимых строк (ряды 4..21)
static const uint8_t  VT_FOOTER_ROW   = 23;          // последняя строка
static const uint8_t  VT_INNER_W      = VT_COLS - 2; // боковые вертикальные рамки "|"

static uint32_t       vtLastSig    = 0;
static bool           vtInitDone   = false;
static AppState       vtLastState  = (AppState)-1;

// Базовые ESC-последовательности
static inline void vtClear()       { Serial.print("\x1B[2J"); }
static inline void vtHome()        { Serial.print("\x1B[H");  }
static inline void vtCursorOff()   { Serial.print("\x1B[?25l"); }
static inline void vtReset()       { Serial.print("\x1B[0m");  }
static inline void vtInverseOn()   { Serial.print("\x1B[7m");  }
static inline void vtMoveTo(uint8_t row1, uint8_t col1) {  // 1-based
  Serial.printf("\x1B[%u;%uH", (unsigned)row1, (unsigned)col1);
}
static inline void vtClearLine() { Serial.print("\x1B[2K"); }

// FNV-1a 32-bit — дешёвый хэш для signature
static uint32_t vtHashUpdate(uint32_t h, const void* data, size_t n) {
  const uint8_t* p = (const uint8_t*)data;
  for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
  return h;
}
static uint32_t vtHashStr(uint32_t h, const char* s) {
  if (!s) return h;
  return vtHashUpdate(h, s, strlen(s));
}

// Печатает строку в окне ширины w с подрезкой и дополнением пробелами.
static void vtPrintFitted(const char* s, int w, char fill = ' ') {
  if (w <= 0) return;
  int L = (int)strlen(s);
  if (L <= w) {
    Serial.print(s);
    for (int i = L; i < w; i++) Serial.write((uint8_t)fill);
  } else {
    // Аналогично OLED: последний символ — '~', остальные (w-1) из исходной.
    for (int i = 0; i < w - 1; i++) Serial.write((uint8_t)s[i]);
    Serial.write((uint8_t)'~');
  }
}

// Шапка: 2 строки. Первая — имя узла и контекст, вторая — разделитель.
static void vtDrawHeader(const char* nodeName, const char* ctx) {
  vtMoveTo(1, 1);
  vtInverseOn();
  // Имя узла (до 24 символов) | контекст (остаток) | время? Нет — компактно.
  Serial.write((uint8_t)' ');
  vtPrintFitted(nodeName ? nodeName : "-", 24);
  Serial.print(" | ");
  vtPrintFitted(ctx ? ctx : "", VT_COLS - 1 - 24 - 3 - 1);
  Serial.write((uint8_t)' ');
  vtReset();
  // Разделитель
  vtMoveTo(2, 1);
  for (int i = 0; i < VT_COLS; i++) Serial.write((uint8_t)'-');
  // Пустая строка перед меню
  vtMoveTo(3, 1);
  vtClearLine();
}

// Footer: инверсная строка внизу.
static void vtDrawFooter(const char* foot) {
  vtMoveTo(VT_FOOTER_ROW, 1);
  vtClearLine();
  vtMoveTo(VT_FOOTER_ROW, 1);
  vtInverseOn();
  Serial.write((uint8_t)' ');
  vtPrintFitted(foot ? foot : "", VT_COLS - 2);
  Serial.write((uint8_t)' ');
  vtReset();
}

// Очистка строк меню (ряды 3..22)
static void vtClearListArea() {
  for (uint8_t r = 3; r < VT_FOOTER_ROW; r++) {
    vtMoveTo(r, 1);
    vtClearLine();
  }
}

// Получить имя текущего узла (или "-" на стадии discover/devices)
static const char* vtCurrentNodeName() {
  if (selectedNode >= 0 && selectedNode < (int)nodeCount) {
    const Node& n = nodes[selectedNode];
    if (n.deviceName[0]) return n.deviceName;
    if (n.nodeName[0])   return n.nodeName;
  }
  return "-";
}

// ----- Отдельные экраны -----

static void vtDrawDiscover() {
  vtClearListArea();
  vtDrawHeader("LEVCAN", "Discover");
  vtMoveTo(VT_LIST_TOP, 3);
  Serial.printf("Searching CAN... found %u node(s)", (unsigned)nodeCount);
  for (uint8_t i = 0; i < nodeCount; i++) {
    vtMoveTo(VT_LIST_TOP + 2 + i, 3);
    const Node& n = nodes[i];
    const char* nm = n.deviceName[0] ? n.deviceName : (n.nodeName[0] ? n.nodeName : "?");
    Serial.printf("addr=%-3u %s", (unsigned)n.addr, nm);
  }
  vtDrawFooter(nodeCount > 0 ? "ENTER=continue" : "ждём узлы LEVCAN...");
}

static void vtDrawDevices() {
  vtClearListArea();
  vtDrawHeader("LEVCAN", "Devices");
  uint8_t maxRows = VT_LIST_HEIGHT;
  if (maxRows > nodeCount) maxRows = nodeCount;
  for (uint8_t i = 0; i < maxRows; i++) {
    bool sel = ((int)i == selectedNode);
    vtMoveTo(VT_LIST_TOP + i, 1);
    if (sel) vtInverseOn();
    char buf[80];
    const Node& n = nodes[i];
    const char* nm = n.deviceName[0] ? n.deviceName : (n.nodeName[0] ? n.nodeName : "?");
    snprintf(buf, sizeof(buf), " %c addr=%-3u %s", sel ? '>' : ' ', (unsigned)n.addr, nm);
    vtPrintFitted(buf, VT_COLS);
    if (sel) vtReset();
  }
  vtDrawFooter("↑/↓ — выбор | ENTER=open | ESC=re-scan");
}

static void vtDrawLoading(const char* what) {
  vtClearListArea();
  vtDrawHeader(vtCurrentNodeName(), "loading");
  vtMoveTo(VT_LIST_TOP + 2, 4);
  Serial.printf("%s", what ? what : "...");
  vtDrawFooter("");
}

static void vtDrawBrowse() {
  vtClearListArea();
  vtDrawHeader(vtCurrentNodeName(), curDir.name);

  // Окно прокрутки: VT_LIST_HEIGHT видимых строк.
  uint16_t vTop = scrollTop;
  if (cursor < vTop) vTop = cursor;
  if (cursor >= vTop + VT_LIST_HEIGHT) vTop = cursor - VT_LIST_HEIGHT + 1;

  for (uint8_t r = 0; r < VT_LIST_HEIGHT; r++) {
    uint16_t i = vTop + r;
    if (i >= curDir.entrySize) break;
    const EntryInfo& e = curDir.entries[i];
    bool sel = (i == cursor);
    vtMoveTo(VT_LIST_TOP + r, 1);

    bool hasPrefix = (e.type == LCP_Folder) || (e.type == LCP_Label);
    char prefix    = (e.type == LCP_Folder) ? '>' : (e.type == LCP_Label ? '#' : ' ');

    // Значение — до 16 символов (в терминале места больше).
    char val[32] = "";
    if (e.hasValue) formatValue(e, val, sizeof(val));
    char vs[18];
    {
      size_t L = strlen(val);
      const size_t VMAX = 16;
      if (L <= VMAX) { strcpy(vs, val); }
      else           { memcpy(vs, val, VMAX-1); vs[VMAX-1] = '~'; vs[VMAX] = 0; }
    }
    size_t vlen = strlen(vs);

    int total   = (int)VT_INNER_W - 2;     // -2: ведущий пробел + курсорный маркер
    int prefixW = hasPrefix ? 1 : 0;
    int sepW    = (vlen > 0) ? 1 : 0;
    int nameMax = total - prefixW - sepW - (int)vlen;
    if (nameMax < 8)  nameMax = 8;
    if (nameMax > 60) nameMax = 60;
    char nm[64];
    {
      size_t L = strlen(e.name);
      if ((int)L <= nameMax) { strcpy(nm, e.name); }
      else                   { memcpy(nm, e.name, nameMax - 1); nm[nameMax - 1] = '~'; nm[nameMax] = 0; }
    }
    int valWidth = total - prefixW - nameMax;
    if (valWidth < 0) valWidth = 0;

    if (sel) vtInverseOn();
    Serial.printf(" %c", sel ? '>' : ' ');
    if (hasPrefix) Serial.printf("%c%-*s%*s", prefix, nameMax, nm, valWidth, vs);
    else           Serial.printf( "%-*s%*s",          nameMax, nm, valWidth, vs);
    if (sel) vtReset();
  }

  // Footer
  const char* selHint = "";
  if (cursor < curDir.entrySize) {
    const EntryInfo& ec = curDir.entries[cursor];
    if (ec.type == LCP_Folder)   selHint = " ENTER=open";
    else if (isEditable(ec))     selHint = " ENTER=edit";
  }
  char foot[80];
  snprintf(foot, sizeof(foot), "%u/%u  ↑/↓  ESC=%s%s",
           (unsigned)cursor + 1, (unsigned)curDir.entrySize,
           dirStackPos > 0 ? "back" : "devices", selHint);
  vtDrawFooter(foot);
}

static void vtDrawEditor() {
  vtClearListArea();
  const EntryInfo& e = curDir.entries[editor.entryIdx];
  vtDrawHeader(vtCurrentNodeName(), e.name);

  uint16_t vTop = editor.scrollTop;
  if (editor.cursor < vTop) vTop = editor.cursor;
  if (editor.cursor >= vTop + VT_LIST_HEIGHT) vTop = editor.cursor - VT_LIST_HEIGHT + 1;

  for (uint8_t r = 0; r < VT_LIST_HEIGHT; r++) {
    uint16_t i = vTop + r;
    if (i >= editor.count) break;
    bool sel = (i == editor.cursor);
    vtMoveTo(VT_LIST_TOP + r, 1);
    if (sel) vtInverseOn();
    char buf[64]; formatEditorOption(i, buf, sizeof(buf));
    char line[VT_COLS + 1];
    snprintf(line, sizeof(line), " %c %s", sel ? '>' : ' ', buf);
    vtPrintFitted(line, VT_COLS);
    if (sel) vtReset();
  }

  char foot[80];
  snprintf(foot, sizeof(foot), "%u/%u  ↑/↓  ENTER=apply  ESC=cancel",
           (unsigned)editor.cursor + 1, (unsigned)editor.count);
  vtDrawFooter(foot);
}

// Вычисляем signature текущего экрана — если не изменился, не перерисовываем (бережём пропускную способность).
static uint32_t vtComputeSignature() {
  uint32_t h = 2166136261u;
  uint8_t st = (uint8_t)appState;
  h = vtHashUpdate(h, &st, 1);
  switch (appState) {
    case S_DISCOVER: {
      h = vtHashUpdate(h, &nodeCount, sizeof(nodeCount));
      for (uint8_t i = 0; i < nodeCount; i++) {
        h = vtHashUpdate(h, &nodes[i].addr, 1);
        h = vtHashStr(h, nodes[i].deviceName);
        h = vtHashStr(h, nodes[i].nodeName);
      }
      break;
    }
    case S_DEVICES: {
      h = vtHashUpdate(h, &nodeCount, sizeof(nodeCount));
      h = vtHashUpdate(h, &selectedNode, sizeof(selectedNode));
      for (uint8_t i = 0; i < nodeCount; i++) {
        h = vtHashUpdate(h, &nodes[i].addr, 1);
        h = vtHashStr(h, nodes[i].deviceName);
        h = vtHashStr(h, nodes[i].nodeName);
      }
      break;
    }
    case S_LOAD_DIR: {
      // Ничего не меняется пока идёт загрузка — фиксируем.
      h = vtHashStr(h, "LOADING");
      break;
    }
    case S_BROWSE: {
      h = vtHashStr(h, curDir.name);
      h = vtHashUpdate(h, &cursor, sizeof(cursor));
      h = vtHashUpdate(h, &scrollTop, sizeof(scrollTop));
      h = vtHashUpdate(h, &curDir.entrySize, sizeof(curDir.entrySize));
      h = vtHashUpdate(h, &dirStackPos, sizeof(dirStackPos));
      // Для видимых строк хэшируем имя, тип и биты флагов.
      uint16_t top = scrollTop;
      if (cursor < top) top = cursor;
      if (cursor >= top + VT_LIST_HEIGHT) top = cursor - VT_LIST_HEIGHT + 1;
      for (uint8_t r = 0; r < VT_LIST_HEIGHT; r++) {
        uint16_t i = top + r;
        if (i >= curDir.entrySize) break;
        const EntryInfo& e = curDir.entries[i];
        h = vtHashStr(h, e.name);
        h = vtHashUpdate(h, &e.type, 1);
        h = vtHashUpdate(h, &e.mode, 1);
        h = vtHashUpdate(h, &e.hasValue, 1);
        if (e.hasValue) h = vtHashUpdate(h, e.value, sizeof(e.value));
      }
      break;
    }
    case S_EDIT: case S_EDIT_APPLY: {
      h = vtHashUpdate(h, &editor.entryIdx, sizeof(editor.entryIdx));
      h = vtHashUpdate(h, &editor.cursor,   sizeof(editor.cursor));
      h = vtHashUpdate(h, &editor.scrollTop,sizeof(editor.scrollTop));
      h = vtHashUpdate(h, &editor.count,    sizeof(editor.count));
      break;
    }
  }
  return h;
}

void vtFrame() {
  if (!vtInitDone) {
    vtInitDone = true;
    vtCursorOff();
    vtClear();
    vtHome();
    vtLastSig = 0;       // принудить первую перерисовку
    vtLastState = (AppState)-1;
  }

  uint32_t sig = vtComputeSignature();
  if (sig == vtLastSig && appState == vtLastState) return;
  vtLastSig   = sig;
  vtLastState = appState;

  switch (appState) {
    case S_DISCOVER:    vtDrawDiscover();         break;
    case S_DEVICES:     vtDrawDevices();          break;
    case S_LOAD_DIR:    vtDrawLoading("loading menu..."); break;
    case S_BROWSE:      vtDrawBrowse();           break;
    case S_EDIT:        vtDrawEditor();           break;
    case S_EDIT_APPLY:  vtDrawLoading("applying..."); break;
  }
}

// ====== setup / loop ======
void setup() {
  // Штатный Serial (UART0, /dev/ttyUSB0) — в будущем пойдёт VT100-интерфейс меню.
  // Пока выводим туда одну строку-заголовок, чтобы легко было отличить порты.
  Serial.begin(115200);
  // Отладочный UART1 на внешнем конвертере: весь существующий лог идёт сюда.
  Dbg.begin(DBG_UART_BAUD, SERIAL_8N1, DBG_UART_RX, DBG_UART_TX);
  delay(300);
  Serial.println("=== UART0 (USB CP210x): будет использован для VT100-меню ===");
  Dbg.println("\n=== ESP32 LEVCAN browser — debug log on UART1 (GPIO9/10), 115200-N-1 ===");
  Dbg.printf("Build: %s %s\n", __DATE__, __TIME__);
  Dbg.printf("DBG UART: num=%d TX=GPIO%d RX=GPIO%d baud=%lu\n",
             DBG_UART_NUM, DBG_UART_TX, DBG_UART_RX, (unsigned long)DBG_UART_BAUD);
  Dbg.println("--- если вы видите этот текст — внешний конвертер подключён корректно ---");
  initButtons();
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false);
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); display.println("LEVCAN browser"); display.println("init CAN..."); display.display();
  if (!initCAN()) { display.println("CAN FAIL"); display.display(); while(1) delay(1000); }
  Dbg.printf("Buttons: BACK=%d UP=%d DOWN=%d SEL=%d\n", BTN_BACK, BTN_UP, BTN_DOWN, BTN_SELECT);
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
  // VT100-клавиатура — всегда в начале итерации, чтобы pollButton() увидел виртуальные нажатия.
  kbPump();
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
      vtFrame();
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
      vtFrame();
      break;
    }
    case S_LOAD_DIR: {
      drawLoading("menu...");
      vtFrame();
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
      vtFrame();
      break;
    }
    case S_EDIT: {
      if (pollButton(BI_UP)   && editor.cursor > 0) editor.cursor--;
      if (pollButton(BI_DOWN) && editor.cursor + 1 < editor.count) editor.cursor++;
      if (pollButton(BI_BACK)) {
        Dbg.println("  edit cancelled");
        appState = S_BROWSE;
      } else if (pollButton(BI_SEL)) {
        // Показать "Applying..." и отправить
        drawLoading("applying...");
        // Для VT100 этот кадр пропустим (применение обычно мгновенное — vtFrame() переведёт экран в S_BROWSE).
        bool ok = applyEditorValue();
        if (ok) drawLoading("applied"); else drawLoading("error");
        delay(400);
        appState = S_BROWSE;
      } else {
        drawEditor();
        vtFrame();
      }
      break;
    }
  }
  delay(5);
}
