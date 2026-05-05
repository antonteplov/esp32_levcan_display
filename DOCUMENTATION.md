# ESP32 LEVCAN Browser — техническая документация

`esp32-can-oled-monitor.ino` — интерактивный браузер LEVCAN-меню на ESP-WROOM-32: подключается к шине CAN через TJA1050, обнаруживает устройства, читает их меню параметров (ParamServer), отображает на OLED SSD1306 (128×64) и позволяет редактировать значения через четыре кнопки.

Устройство, участвующее в разработке — контроллер света `aLight` (адрес 98), но скетч универсален: обнаруживает любые узлы LEVCAN с реализованным ParamServer.

---

## 1. Аппаратное окружение

| Узел | Подключение |
|------|-------------|
| OLED SSD1306 | I²C, SDA=GPIO5, SCL=GPIO4, addr=0x3C |
| TJA1050 (CAN) | TX=GPIO25, RX=GPIO26 |
| Кнопка Back   | GPIO13 → GND, INPUT_PULLUP |
| Кнопка Up     | GPIO14 → GND, INPUT_PULLUP |
| Кнопка Down   | GPIO27 → GND, INPUT_PULLUP |
| Кнопка Select | GPIO33 → GND, INPUT_PULLUP |

CAN-таймминг настроен под 1 Mbit/s, точка сэмплирования 87.5%, что зеркалит конфигурацию `aLight4` (BTR=0x01050002):

```cpp
twai_timing_config_t t = { .brp = 10, .tseg_1 = 6, .tseg_2 = 1, .sjw = 2, .triple_sampling = false };
```

ESP32 берёт адрес `MY_ADDR = 100`, broadcast — `127`. Целевой узел определяется в момент выбора в меню Devices.

---

## 2. Состав программы

### 2.1 Сводная карта файла

| Раздел | Строки | Назначение |
|--------|--------|-----------|
| Включения, HW-константы | 28–45 | Аддресса OLED/CAN, GPIO кнопок |
| Константы LEVCAN | 47–78 | msgId, командные биты ParamServer, типы LCP, флаги mode |
| Размеры буферов | 80–87 | `MAX_NODES`, `MAX_ENTRIES_DIR`, `NAME_BUF`, `TEXT_BUF`, `DESC_BUF`, `ASM_BUF`, `DEPTH_STACK` |
| Основные структуры | 96–127 | `Node`, `EntryInfo`, `DirInfo` |
| Глобалы | 129–143 | список узлов, текущая директория, курсор, стек возврата |
| Pack/unpack 29-bit ID | 145–172 | `LcHeader`, `packId`, `unpackId` |
| TWAI-обёртка | 174–189 | `initCAN`, `sendFrame` |
| Низкоуровневые запросы | 191–223 | `reqDir`, `reqEntryCmd` и обёртки, `reqSysName`, broadcast |
| Multi-frame TX | 225–321 | `sendTcpToServer`, `sendValueSet` |
| Утилиты | 323–375 | `typeName`, `cp1251ToAscii`, `findOrAddNode`, `nodeDisplayName` |
| Сборщик каналов | 377–396 | `Channel`, `dispatchChannel`, `sendCTS` |
| Состояния ожидания | 398–422 | `pending`, `setResult`, `txWait` |
| Callbacks парсинга | 424–508 | `onDeviceName`, `onNodeName`, `onDirData`, `onEntryData`, `onName`, `onText`, `onDesc`, `onValue` |
| Диспатч и handleFrame | 510–628 | разбор кадров, RTS/CTS-FSM приёмника |
| Загрузка директории | 630–765 | `loadDir`, `clearDir`, `clearEntryFlags`, `waitFor`, ready-предикаты |
| Live-обновление | 767–782 | `refreshLiveValues` |
| Кнопки | 784–801 | `pollButton` с дребезгом 30 мс |
| UI и состояния | 803–884 | `AppState`, `editor`, `drawHeader/Footer/Discover/Devices/Loading` |
| Форматирование значений | 886–975 | `pickEnumLabel`, `formatValue` |
| Браузер меню | 977–1023 | `drawBrowse` |
| Навигация | 1025–1076 | `enterDevice`, `enterFolder`, `goBack`, `fetchFolderDirIndex` |
| Редактор | 1078–1328 | `isEditable`, `prepareEditor`, `formatEditorOption`, `applyEditorValue`, `handleSelect`, `drawEditor` |
| `setup()` / `loop()` | 1330–1431 | главный цикл и FSM приложения |

### 2.2 Зависимости

- Arduino-ESP32 ≥ 2.0 (даёт TWAI-драйвер `driver/twai.h`).
- Adafruit_GFX + Adafruit_SSD1306 для OLED.

Сторонняя реализация LEVCAN не требуется — протокол реализован вручную, чтобы не тащить C-библиотеку и иметь полный контроль над TX/RX.

---

## 3. Типы данных

### 3.1 `LcHeader` — заголовок 29-битного CAN ID

```cpp
struct LcHeader {
  uint8_t  source, target;   // 7 бит каждый
  uint16_t msgId;            // 10 бит
  uint8_t  eom, parity, rts, prio;  // по 1 биту, prio=2 бита
};
```

Раскладка в 29-бит CAN-ID (LSB → MSB):

| Биты | Поле |
|------|------|
| 0–6  | `source` |
| 7–13 | `target` |
| 14–23 | `msgId` |
| 24 | `eom` (End-of-Message) |
| 25 | `parity` |
| 26 | `rts` (Request-To-Send / Clear-To-Send) |
| 27–28 | `prio` |

`packId/unpackId` маршрутизируют `LcHeader` ↔ 29-битный extended-ID. Бит `Request` LEVCAN-протокола соответствует CAN-RTR-биту самого кадра — он передаётся через параметр `rtr` в `sendFrame`, не через `LcHeader`.

### 3.2 `Node` — обнаруженный узел LEVCAN

```cpp
struct Node {
  bool     present;
  uint8_t  addr;
  char     deviceName[NAME_BUF];   // фирменное имя (0x389 LC_SYS_DeviceName)
  char     nodeName[NAME_BUF];     // пользовательское имя (0x388 LC_SYS_NodeName)
  bool     hasDeviceName, hasNodeName;
  uint32_t lastSeenMs;
};
```

Узел появляется при первом получении `AddressClaimed` (0x380) или ответа на запрос имени. `nodeDisplayName` отдаёт первое непустое из {nodeName, deviceName, "node<addr>"}.

### 3.3 `EntryInfo` — один пункт меню

```cpp
struct EntryInfo {
  uint8_t  type;           // LCP_Type
  uint8_t  mode;           // LCP_MODE_RO / WO / LIVE_UPD / LIVE_CHG
  uint16_t entryIdx;
  uint16_t varSize;        // байты значения (для Folder — индекс дочерней директории)
  uint16_t descSize, textSize;
  bool     hasData, hasName, hasDesc, hasText, hasValue;
  char     name[NAME_BUF];
  char     text[TEXT_BUF]; // formatString для чисел; список «лейбл\nлейбл» для Enum/Bool
  uint8_t  desc[DESC_BUF];  uint8_t descLen;
  uint8_t  value[16];       uint8_t valueLen;
};
```

Поле `varSize` для Folder особое: оно содержит индекс дочерней директории — сервер пишет его туда же, см. `levcan_paramserver.c:264 «dindex = entry->VarSize»`.

### 3.4 `DirInfo` — текущая директория

```cpp
struct DirInfo {
  bool      known;
  uint16_t  entrySize;     // сколько entries в директории
  uint16_t  nameSize;
  char      name[NAME_BUF];
  EntryInfo entries[MAX_ENTRIES_DIR];   // фиксированный массив, MAX_ENTRIES_DIR=20
};
```

В каждый момент времени загружена одна директория. При навигации внутрь старая директория стирается.

### 3.5 Стек возврата

```cpp
struct StackFrame { uint16_t dirIndex; uint8_t cursor; uint8_t scrollTop; };
StackFrame dirStack[DEPTH_STACK];   // 8 уровней
```

При входе в Folder в стек кладётся текущее `(dirIndex, cursor, scrollTop)`. Кнопка Back возвращает обратно ровно в ту же позицию.

### 3.6 `Channel` — буфер сборки multi-frame сообщения

```cpp
struct Channel { bool active; uint16_t msgId; uint16_t pos; uint8_t buf[ASM_BUF]; uint8_t srcAddr; };
```

Семь параллельных каналов — по числу msgId, которые ESP32 принимает от сервера: `0x39A, 0x39B, 0x39C, 0x39D, 0x39E, 0x388, 0x389`. На каждый msgId одновременно может собираться только один источник; если пришёл новый кадр от другого src — буфер сбрасывается.

### 3.7 `pending`, `setResult`, `txWait`

Три глобальных статус-блока:

- **`pending`** (`WaitKind = WK_NONE | WK_DIR | WK_ENTRY | WK_VALUE | WK_SET`) — какую транзакцию мы ждём. Используется в `dispatchChannel`, чтобы понять, в какое поле какого entry класть пришедшие байты.
- **`setResult`** — флаг + ErrorCode для `applyEditorValue`. Заполняется когда после ValueSet от сервера приходит однобайтовый ответ на 0x39A.
- **`txWait`** — состояние для multi-frame TX-движка `sendTcpToServer`. Слушает RTR-кадры от сервера: `gotCts` (RTS=1, EoM=0) — разрешение продолжать, `gotFinalAck` (RTS=0, EoM=1) — финальное подтверждение.

### 3.8 `editor` — состояние редактора значений

```cpp
struct {
  uint16_t entryIdx;
  uint8_t  type;
  uint16_t varSize;
  bool     isSigned;
  int32_t  vmin, vmax, vstep;
  uint16_t count;       // сколько пунктов выбора, ≤ EDIT_MAX_OPTIONS=256
  uint16_t cursor, scrollTop;
  uint8_t  decimals;    // только для Decimal32
} editor;
```

Список вариантов значений нигде в памяти не хранится — `i`-й вариант это `vmin + i * vstep`.

### 3.9 Состояния приложения

```cpp
enum AppState { S_DISCOVER, S_DEVICES, S_LOAD_DIR, S_BROWSE, S_EDIT, S_EDIT_APPLY };
```

(`S_EDIT_APPLY` зарезервировано, пока не используется — apply делается синхронно прямо из `S_EDIT`.)

---

## 4. Подпрограммы

### 4.1 Низкий уровень CAN

| Функция | Что делает |
|---------|-----------|
| `initCAN()` | Регистрирует TWAI-драйвер, ставит фильтр accept-all, очереди TX=16/RX=64, включает alerts BUS_OFF/RX_QUEUE_FULL/TX_FAILED |
| `sendFrame(id, data, len, rtr=false)` | Шлёт extended-кадр с таймаутом 50 мс |
| `packId/unpackId` | Конверсия `LcHeader` ↔ 29-bit ID |

### 4.2 Запросы LEVCAN ParamServer

Все идут на msgId=`0x399 ParametersRequest` с маленьким payload-описателем команды.

| Функция | Команда | Payload |
|---------|---------|---------|
| `reqDir(target, dirIndex)` | `LCP_REQ_DIRECTORY_INFO=0x20` | u16 cmd, u16 dirIndex |
| `reqEntry(target, dir, e)` | `LCP_REQ_DATA_NAME=0x03` | u16 cmd, u16 dir, u16 e |
| `reqEntryVariable(...)` | `LCP_REQ_VARIABLE=0x10` | u16 cmd, u16 dir, u16 e |
| `reqEntryDescriptor(...)` | `LCP_REQ_DESCRIPTOR=0x08` | u16 cmd, u16 dir, u16 e |
| `reqEntryText(...)` | `LCP_REQ_TEXT=0x04` | u16 cmd, u16 dir, u16 e |

### 4.3 Запросы системных имён

```cpp
bool reqSysName(uint8_t target, uint16_t msgId);   // 0x388 / 0x389
```

Шлёт **RTR-кадр** на `target/msgId` с критичными битами в заголовке: `RTS_CTS=0, EoM=0, Parity=0`. Эти три нуля обязательны — в `levcan.c` есть проверка:

```c
if (header.Request) {
  if (header.RTS_CTS == 0 && header.EoM == 0) /* new request */ ...
  else  /* trying to control existing TX object */ ...
}
```

Если поставить иначе, сервер посчитает кадр CTS/EoM-ACK для несуществующего TX-объекта и проигнорирует.

`broadcastDeviceNameQuery()` шлёт на target=127 (broadcast) — отвечают все узлы; `broadcastNodeNameQuery` аналогично, но не все серверы отвечают на broadcast 0x388 — потому в discover-цикле на каждый найденный узел дополнительно шлётся адресный `reqNodeName(addr)`.

### 4.4 Сборщик multi-frame входящих сообщений

Алгоритм в `handleFrame` для приёма:

1. Игнорим кадры от себя и не для нас.
2. `AddressClaimed` (0x380) — добавляем узел, если новый, и адресно запрашиваем оба имени.
3. RTR-кадры — это CTS/ACK от сервера во время нашего multi-frame TX (см. 4.5).
4. Иначе ищем `Channel` для msgId. Если канал занят другим src — сброс.
5. Состояния:
   - `RTS=1, EoM=1` — single-frame: данные сразу в буфер, отправляем финальный CTS, диспатч.
   - `RTS=1, EoM=0` — начало multi-frame: данные в буфер, отправляем CTS, ждём продолжения.
   - `RTS=0` — продолжение: данные дописываем; если `EoM=1` — финальный CTS и диспатч, иначе CTS на следующий блок.

Паритет CTS вычисляется как `(~((pos+7)/8)) & 1`, где `pos` — текущая позиция после приёма данных. То же значение использует и сервер на своей стороне.

`sendCTS(target, msgId, parity, rts, eom)` — отправка CTS как **RTR-кадра** (важно: именно RTR, не data).

### 4.5 Multi-frame TX (`sendTcpToServer`)

Используется только для `ValueSet` (10 байт payload). Алгоритм:

| Шаг | Действие | Кадр |
|-----|----------|------|
| 1 | Передать первые 8 байт | data, RTS=1, EoM=0, Parity=1 |
| 2 | Дождаться CTS от сервера | RTR, RTS=1, EoM=0, parity=ожидаемый |
| 3 | Передать оставшийся хвост | data, RTS=0, EoM=last?1:0, parity по формуле |
| 4 | (если ещё не последний) дождаться промежуточного CTS | |
| 5 | Дождаться финального ACK | RTR, RTS=0, EoM=1 |

Для payload ≤ 8 байт алгоритм сворачивается в single-frame: `RTS=1, EoM=1, Parity=1` + ожидание финального ACK.

Все ожидания CTS/ACK с таймаутом 400–800 мс. Внутри ожидания мы продолжаем поллить TWAI-очередь, чтобы остальные кадры (live updates от сервера) не накапливались.

### 4.6 Утилиты

- `cp1251ToAscii` — транслит из cp1251 (в которой alight хранит русские строки) в ASCII. Русские буквы → латиница (`А`→`A`, `я`→`ya`, `Ё`→`Yo`), `°` → ` deg`, всё остальное вне ASCII → `?`. Применяется к: DeviceName, NodeName, имени директории, именам entries, тексту (TextData) — поэтому `\n` (0x0A) проходит как есть.
- `pickEnumLabel(src, idx, out, outSize)` — выбирает n-ю подстроку, разделители — `'\n'`. Используется при отображении Enum/Bool.

### 4.7 Парсинг ответов

Колбэки заполняют поля `EntryInfo` или `Node`:

| Колбэк | На какой msgId | Результат |
|--------|----------------|-----------|
| `onDeviceName` | 0x389 | `nodes[i].deviceName` |
| `onNodeName`   | 0x388 | `nodes[i].nodeName` |
| `onDirData`    | 0x39A в режиме `WK_DIR` | `curDir.entrySize`, `nameSize` |
| `onName`       | 0x39C | имя директории (`entryIdx<0`) или entry |
| `onEntryData`  | 0x39A в режиме `WK_ENTRY` | type, mode, varSize, descSize, textSize |
| `onDesc`       | 0x39B | сырой descriptor |
| `onText`       | 0x39D | TextData (формат для чисел / лейблы Enum) |
| `onValue`      | 0x39E | сырое значение |

Диспатч (`dispatchChannel`) маршрутизирует пакет по полям через `pending.kind` — это позволяет переиспользовать `0x39A/0x39C/0x39E` как для DirInfo, так и для Entry-данных без коллизий.

Особый случай — `WK_SET`: ответ ValueSet приходит на `0x39A` с одним байтом `ErrorCode`. Чтобы не путать с DirData (≥6 байт), в коде сделана отдельная ветка `pending.kind == WK_SET` — она читает `c.buf[0]` и выставляет `setResult`.

### 4.8 Алгоритм `loadDir(target, dirIndex)`

Это синхронная блокирующая операция со внутренним поллингом TWAI-очереди и таймаутами:

1. Сброс всех каналов и `curDir`.
2. **DirInfo**: `pending = WK_DIR`; шлём `reqDir`; ждём `dirNameReady` (известны `entrySize` и имя директории) до 1500 мс.
3. Для каждого `i` из `[0..entrySize)`:
   1. Сброс per-entry флагов, `pending = WK_ENTRY`.
   2. **Шаг A — Data + Name**: `reqEntry(LCP_REQ_DATA_NAME)`, ждём `entryNameReady` (3 ретрая по 1500 мс).
   3. **Шаг A' — Descriptor**: только для `Decimal32` (Decimals) и `Enum` (Min/Size). `reqEntryDescriptor`, ждём `entryDescReady`.
   4. **Шаг A'' — Text** (rc1.1 fast-path): запрашивается только если `textSize > strlen(name)`. **Важная особенность alight server**: он возвращает `TextSize = strlen(name) + strlen(textData)`, а не размер только textData. У большинства Bool textData=NULL, но textSize всё равно равен длине имени. Если бы ориентироваться только на `textSize > 0`, ESP пытался бы ждать Text для каждого Bool — и получал бы 4.5 с timeout (сервер ничего не отправляет). Bool/Enum без textData используют встроенный «off / ON».
   5. **Шаг Б — Value**: для всех типов кроме `Folder/Label/WriteOnly/varSize=0`. `reqEntryVariable`, ждём `entryValueReady`. **Важно**: сервер не присылает Value на FullEntry-запрос, нужен именно отдельный `LCP_REQ_VARIABLE`.
   6. `delay(20)` между entries — alight TX-очередь успевает разгрузиться.

Между ретраями вызывается `resetAllChannels()`, чтобы не пытаться продолжить уже частично собранное и порченное multi-frame.

### 4.9 `refreshLiveValues(target, dirIndex)`

Запускается в `S_BROWSE` каждые `LIVE_PERIOD = 700` мс. Для каждого entry с флагом `mode & LCP_MODE_LIVE_UPD` (например, температуры T1/T2/CPU в alight):

```cpp
e.hasValue = false;
pending = { WK_VALUE, target, dirIndex, i, millis() };
reqEntryVariable(target, dirIndex, i);
waitFor(400, entryValueReady);
```

Не блокирует UI больше 400 мс на entry. Если сервер не успел ответить — спокойно идём дальше, на следующем тике повторим.

### 4.10 Кнопки

`pollButton(i)` — debounce 30 мс, возвращает `true` ровно один раз на нажатие (фронт «1→0», т.к. INPUT_PULLUP). Все четыре кнопки опрашиваются в текущем `case` `loop()`-а.

### 4.11 UI-компоненты

| Функция | Что рисует |
|---------|------------|
| `drawHeader(line)` | Инверсный белый прямоугольник 128×9 в верхней строке + текст |
| `drawFooter(line)` | Текст в нижней строке (y=`SCREEN_HEIGHT-8`) |
| `drawDiscover()` | «Searching CAN…», список найденных, подсказка `Sel=Continue` |
| `drawDevices()` | Список узлов с подсветкой курсора, подсказки на футере |
| `drawLoading(what)` | «Loading… <what>» |
| `drawBrowse()` | 4 строки меню по 10 пикс, формат `prefix(1) + name(14) + value(6)` |
| `drawEditor()` | Заголовок=имя параметра, 4 строки вариантов с курсором, футер `i/N S=apply B=cancel` |

### 4.12 `formatValue(e, out, outSize)`

Форматирует value для отображения в `drawBrowse`:

- **Bool**: индекс 0/1 → лейбл из `e.text` через `pickEnumLabel`, иначе `off/ON`.
- **Int32 / Uint32**: десятичное число.
- **Decimal32**: int32/16/8 со знаком, `value / 10^Decimals` с правильным форматом дробной части. Особый случай `-0.x` обрабатывается отдельно, чтобы не потерять знак при `whole==0`.
- **Float**: 2 знака после запятой.
- **Enum**: `idx = (value - Min)`, лейбл из TextData; если нет — числовой индекс.
- **Default**: hex-дамп первых 4 байт.

### 4.13 Редактор

`isEditable(e)` — entry редактируем, если он не {Folder, Label, String, Float, Double, Bitfield32}, не RO, не WO, и `varSize > 0`.

`prepareEditor(entryIdx)`:

- Bool: `vmin=0, vmax=1, vstep=1, count=2`.
- Enum: читает `LCP_Enum_t = {Min:u32, Size:u32}` из дескриптора, `count = Size`.
- Int32/Int64: `Min, Max, Step` из дескриптора (12 байт), `count = (Max-Min)/Step+1`, ограничение `EDIT_MAX_OPTIONS=256`.
- Uint32/Uint64: то же, но без знака.
- Decimal32: `Min, Max, Step` (12 байт) + `Decimals` (1 байт по offset 12).

После расчёта диапазона курсор автопозиционируется на текущее значение, если оно есть.

`formatEditorOption(i, out, outSize)` — форматирует i-й вариант:

- Bool/Enum — лейбл из TextData.
- Decimal32 — десятичная дробь по `Decimals`.
- Численные — пытается распарсить и применить format-string из TextData (например `"%d%%"`). Парсер пропускает только безопасные конверсии `%d/%i/%u` (без `%s/%n`); если строка некорректна — fallback на голое число.

`applyEditorValue()`:

1. Кодирует выбранное значение в LE-байты по `varSize` (с расширением знаком для 8-байтных).
2. `pending = WK_SET`, `setResult.received = false`.
3. Шлёт `sendValueSet(target, dir, entry, bytes, varSize)`.
4. Поллит TWAI до прихода ответа (1 байт ErrorCode на 0x39A) или таймаута 1000 мс.
5. Если `errorCode == 0` — обновляет `e.value` локально, чтобы не ждать следующего live-цикла.

### 4.14 `handleSelect()` — общий обработчик кнопки Select

| Тип entry | Действие |
|-----------|----------|
| Folder | `enterFolder(e.varSize)` — толкаем стек, идём в `S_LOAD_DIR` |
| editable (см. `isEditable`) | `prepareEditor(cursor)` + `appState = S_EDIT` |
| остальное | Лог в Serial: «read-only» |

---

## 5. Алгоритмы взаимодействия

### 5.1 Главная конечная-автомат-машина (`loop()`)

```
       ┌────────────┐
       │ S_DISCOVER │  пассивный приём 0x380 + бродкаст 0x389 каждые 5 c
       └─────┬──────┘
       Sel + nodeCount>0 + 2.5 c прошло
             ▼
       ┌────────────┐
       │ S_DEVICES  │  Up/Down — выбор; Sel — открыть; Back — заново DISCOVER
       └─────┬──────┘
             │ Sel
             ▼
       ┌────────────┐  loadDir(target, 0)
       │ S_LOAD_DIR │  blocking, ~1–2 с
       └─────┬──────┘
             │ ok
             ▼
       ┌────────────┐
       │ S_BROWSE   │  Up/Down — курсор; Sel — выбор; Back — pop
       └──┬─────┬───┘
          │     │ Sel на Folder
          │     │ → enterFolder → S_LOAD_DIR
          │
          │ Sel на editable
          ▼
       ┌────────────┐
       │   S_EDIT   │  Up/Down — выбор значения
       └─────┬──────┘
             │
        Sel ─┴─ Back
        apply       cancel
        ▼            ▼
       назад в S_BROWSE
```

`S_BROWSE` — единственное состояние с активным live-обновлением: каждые 700 мс параллельно с UI запрашиваем Variable у entries с флагом `LIVE_UPD`.

### 5.2 Обнаружение узлов

В `setup()` сразу шлётся `broadcastDeviceNameQuery()`. Все узлы LEVCAN, у которых есть DeviceName, отвечают (адресно — на нашу `MY_ADDR=100`). В `S_DISCOVER` каждые 5 секунд бродкаст повторяется + для уже найденных узлов без NodeName делается адресный `reqNodeName(addr)`.

Параллельно ESP32 автоматически реагирует на любые `AddressClaimed (0x380)` — не дожидаясь нашего запроса; это нужно потому, что alight шлёт `AddressClaimed` периодически.

### 5.3 Чтение меню (что происходит при выборе устройства)

Хронология обмена для входа в директорию 0:

```
ESP→alight  0x399 RTS=1 EoM=1 par=1 [0x20 0x00, 0x00 0x00]                 ; LCP_REQ_DIRECTORY_INFO, dir=0
alight→ESP  0x39A RTS=1 EoM=1 [entrySz, nameSz, dirIdx, ...payload]        ; DirData
ESP→alight  0x39A RTR rts=0 eom=1 par=...                                  ; финальный CTS
alight→ESP  0x39C RTS=1 EoM=0 par=1 [first 8 bytes of dir name]            ; начало имени
ESP→alight  0x39C RTR rts=1 eom=0 par=ожидаемый                            ; CTS на продолжение
alight→ESP  0x39C RTS=0 EoM=1 par=... [хвост]
ESP→alight  0x39C RTR rts=0 eom=1                                          ; финальный CTS

Затем для каждой entry (i = 0..entrySize-1):
  ESP→alight  0x399 [LCP_REQ_DATA_NAME=0x03, dir, i]   → 0x39A (data) + 0x39C (name)
  если type ∈ {Decimal32, Enum}:
    ESP→alight  0x399 [LCP_REQ_DESCRIPTOR=0x08, dir, i] → 0x39B
  если type ∈ {Enum, Bool} && textSize>0:
    ESP→alight  0x399 [LCP_REQ_TEXT=0x04, dir, i]       → 0x39D
  если editable & не Folder/Label/WO:
    ESP→alight  0x399 [LCP_REQ_VARIABLE=0x10, dir, i]   → 0x39E
```

**Почему не FullEntry:** сервер `aLight` не присылает Value в ответ на `LCP_REQ_FULL_ENTRY=0x1F` — пробовали, проверено в логах. Поэтому value запрашивается отдельным `LCP_REQ_VARIABLE`.

### 5.4 Запись значения (ValueSet)

Пользовательский сценарий:

1. В `S_BROWSE` курсор на editable entry, нажат Sel.
2. `handleSelect` → `prepareEditor` → `appState=S_EDIT`.
3. `drawEditor` рисует список вариантов; Up/Down двигают `editor.cursor`.
4. Sel → `applyEditorValue()`:
   - Кодируем `(vmin + cursor*vstep)` в LE-байты.
   - Собираем `lc_value_set_t = [u16 cmd=0x40, u16 dir, u16 entry, value_bytes]` — 6 + varSize байт.
   - `sendTcpToServer(target, 0x399, ...)` отправляет multi-frame TCP.
   - Ждём ответ (1 байт `ErrorCode` на 0x39A) до 1000 мс.
5. Показываем `applied` или `error` на 400 мс, возвращаемся в `S_BROWSE`.

Кнопка Back в `S_EDIT` — отмена, без отправки.

### 5.5 Live-обновление

Только в `S_BROWSE`, период 700 мс. Для каждого entry с `mode & LCP_LiveUpdate`:
- Сбросить `e.hasValue = false`.
- `pending = WK_VALUE`.
- Шлём `LCP_REQ_VARIABLE`.
- Ждём ответ ≤ 400 мс.

Это даёт «живые» T1/T2/CPU температуры в корне `aLight`.

---

## 6. Известные ограничения

- **`MAX_ENTRIES_DIR = 20`** — превышение в reальном меню alight нет, но если появится — `onDirData` отвергнет директорию с «bogus size».
- **`EDIT_MAX_OPTIONS = 256`** — для параметров типа `(Min=0, Max=1023, Step=1)` редактор покажет только первые 256 значений.
- **String / Float / Double / Bitfield32** — отображение есть (для Float — `%.2f`), редактирование не реализовано.
- **WriteOnly** не поддерживается ни в чтении (понятно), ни в записи через UI.
- **Multi-frame TX** для payload >16 байт работает, но на каждый блок ждёт промежуточный CTS — на практике не возникает, т.к. ValueSet всегда ≤ 14 байт.
- **TextData буфер 96 байт** — для очень длинного списка лейблов (или большой format-строки) хвост обрежется.
- **Один активный target за раз** — переключиться на другой узел можно через Back до S_DEVICES.

---

## 7. Отладка

В скетче есть две глобальные опции (строки 90–91):

```cpp
static const bool VERBOSE_RX  = true;   // печатает каждый принятый кадр
static const bool VERBOSE_CTS = false;  // печатает каждый исходящий CTS
```

При `VERBOSE_RX=true` Serial-лог выглядит как:

```
    RX 0x12640D9A src=98 msg=0x39A eom=1 par=1 rts=1 dlc=8
  EntryData[0]: type=Dec mode=0x05 varSz=2
    RX 0x12640D9C src=98 ...
  EntryName[0]: "T-sensor T1"
  EntryValue[0]: len=2
```

При проблемах с ValueSet полезно временно включить `VERBOSE_CTS=true` — будет видно как сервер двигает паритеты в CTS-кадрах.

---

## 8. Расширения, которые легко добавить

- Поддержка String entries (read через `LCP_REQ_VARIABLE`, отображение как обычный текст).
- Поддержка редактирования Float (вместо генерации списка — целевое значение по нажатиям Up/Down).
- Уход в LIVE_CHG-режим (отправка ValueSet на каждом движении курсора, а не только на apply).
- Сохранение последнего выбранного device в NVS, чтобы при перезагрузке не нужно было заново сканировать.
- Полный обход всего дерева для дампа меню — для этого зарезервирован `#define ENABLE_FULL_WALKER` (по умолчанию выключен).
