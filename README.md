# ESP32 CAN OLED Monitor

Сборка и прошивка тестового скетча для платы ESP32 с OLED и внешним CAN-трансивером TJA1050 без Arduino IDE, через `arduino-cli`.[1][2]

## Что делает скетч

Скетч использует встроенный в ESP32 драйвер TWAI (CAN-контроллер ESP32), настраивает скорость 1 Мбит/с, принимает все CAN-кадры в режиме listen-only и выводит счетчики на OLED и в serial. [3][4]

Пины в скетче зафиксированы так:
- OLED: SDA=GPIO5, SCL=GPIO4.
- CAN: TX=GPIO25, RX=GPIO26. [5]

## Что установить в Ubuntu

```bash
sudo apt update
sudo apt install -y curl make
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo install -m 755 bin/arduino-cli /usr/local/bin/arduino-cli
arduino-cli version
```

У Arduino CLI нужно добавить индекс плат Espressif и установить core `esp32:esp32`, после чего можно собирать и загружать ESP32-проекты из командной строки. [6][7]

## Подготовка окружения

```bash
cd esp32-can-oled-monitor
make setup
```

Эта цель добавляет индекс плат Espressif, обновляет индекс, ставит core `esp32:esp32` и библиотеки `Adafruit SSD1306`, `Adafruit GFX Library`, `Adafruit BusIO`. [6][2][8]

Если нужен доступ к последовательному порту в Ubuntu, добавь пользователя в группу `dialout`:

```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

## Сборка

```bash
make compile
```

По умолчанию используется плата `esp32:esp32:lolin32`, что соответствует выбранной в IDE плате WEMOS LOLIN32. [1]

## Прошивка

```bash
make upload PORT=/dev/ttyUSB0
```

Загрузка через `arduino-cli upload` для ESP32 использует тот же core и инструменты, что и Arduino IDE, только без GUI. [9][10]

## Монитор порта

```bash
make monitor PORT=/dev/ttyUSB0
```

Для serial используется скорость 115200 бод, как в скетче. [1]

## Полезные замечания

- Если плата не входит в режим загрузчика автоматически, может понадобиться кнопка `BOOT` во время старта upload. Это типичный сценарий для ESP32-плат. [1]
- Предупреждение `WARN: RX queue full` означает, что входной поток кадров выше, чем скетч успевает обработать и печатать; сам прием при этом уже работает. Это особенно заметно на 1 Мбит/с при активном выводе в serial. [3]
- При желании можно изменить FQBN: `make compile FQBN=esp32:esp32:esp32`. Arduino CLI поддерживает сборку и загрузку через указание FQBN платы. [2]
