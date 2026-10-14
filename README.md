# automachine

Учебный проект: автономная самоходная машина в масштабе 1:10. Пока есть связь, ею управляет оператор. Если пропадает пульт, машина продолжает заданный маршрут. Если пропадает GPS, она считает путь по энкодеру колеса, гироскопу и компасу.

**Бюджет:** до 30 000 ₽ (базовый комплект — около 29 600 ₽)
**Срок:** 10–12 недель
**Основа:** шасси 1:10 4WD, контроллер SpeedyBee F405 Wing с ArduPilot Rover, Raspberry Pi Zero 2 W

## Что в репозитории

| Путь | Что это |
|---|---|
| `index.html` | Сайт-макет: симулятор поведения машины, схема, смета, план, сборка, защита |
| `docs/Проект — автономная машина.docx` | Полный документ проекта: план, смета, сборка, ПО, защита, испытания |
| `software/dispatcher.py` | Бортовой диспетчер для Raspberry Pi (Python, pymavlink) |
| `software/rover-start.param` | Стартовые параметры ArduPilot Rover для Mission Planner |

Сайт открывается прямо из файла `index.html`. Его можно опубликовать через GitHub Pages: Settings → Pages → Deploy from a branch → `main`, папка `/ (root)`.

## Как машина ведёт себя без оператора

| Ситуация | Что делает |
|---|---|
| Пропала связь с пультом | В «Авто» продолжает маршрут; в «Ручном» останавливается, через 2 с диспетчер включает «Авто» |
| Пропал или подменён GPS | Переходит на счисление пути (колёса + компас), снижает скорость до 1 м/с |
| Без GPS дольше 60 с | Останавливается и ждёт GPS, затем продолжает |
| Препятствие ближе 1 м | Останавливается, после освобождения пути едет дальше |
| Выезд за геозону | Останавливается |

## Бортовой диспетчер

```bash
python3 -m venv venv && source venv/bin/activate
pip install pymavlink MAVProxy

# маршрутизатор: контроллер -> ноутбук и -> диспетчер
mavproxy.py --master=/dev/serial0 --baudrate 921600 \
  --out udp:<IP_ноутбука>:14550 --out udp:127.0.0.1:14551 --daemon

# сначала в режиме «только журнал»
python3 software/dispatcher.py --connect udpin:127.0.0.1:14551 --dry-run
```

Перед установкой на машину проверьте всё в симуляторе ArduPilot SITL:

```bash
sim_vehicle.py -v Rover --console --map
# потеря пульта:  param set SIM_RC_FAIL 1
# потеря GPS:     param set SIM_GPS1_ENABLE 0   (в старых версиях SIM_GPS_DISABLE 1)
```

## Безопасность

Испытания проводите только на пустой площадке, без людей и машин. Глушилки GPS и связи использовать нельзя: потерю связи имитируйте выключением пульта, потерю GPS — в симуляторе или экранированием модуля.

## Ссылки

- [ArduPilot Rover — реакции на отказы](https://ardupilot.org/rover/docs/rover-failsafes.html)
- [ArduPilot — энкодеры колёс](https://ardupilot.org/rover/docs/wheel-encoder.html)
- [ArduPilot — источники навигации EKF](https://ardupilot.org/rover/docs/common-ekf-sources.html)
- [ArduPilot — SpeedyBee F405 Wing](https://ardupilot.org/rover/docs/common-speedybeef405wing.html)
- [ArduPilot — симулятор SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
