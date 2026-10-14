#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Бортовой диспетчер учебной автономной машины 1:10.

Работает на бортовом компьютере (Raspberry Pi) и общается с контроллером
автопилота ArduPilot Rover по протоколу MAVLink. Задачи диспетчера:

  1. Пропала связь с пультом, а машина ехала в ручном режиме:
     перевести её в режим AUTO, чтобы она продолжила заданный маршрут.
  2. Пропал или искажён сигнал GPS: переключить навигацию на счисление
     пути (энкодер колеса + гироскоп + компас) и снизить скорость.
  3. Счисление пути длится слишком долго (ошибка накапливается):
     остановиться (HOLD) и ждать возвращения GPS, затем продолжить.
  4. Препятствие впереди ближе порога: остановиться, после того как путь
     освободится, продолжить маршрут.

Диспетчер НЕ мешает оператору: если связь есть, он не меняет режимы,
кроме остановки перед препятствием в режиме AUTO.

Перед установкой на машину обязательно проверьте работу в симуляторе
ArduPilot SITL (см. документ проекта, раздел «Испытания»).

Запуск:
    python3 dispatcher.py --connect udpin:127.0.0.1:14551
    python3 dispatcher.py --connect /dev/serial0 --baud 921600
    python3 dispatcher.py --dry-run      # только журнал, без команд
"""

import argparse
import logging
import os
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

mavutil.set_dialect("ardupilotmega")
MAV = mavutil.mavlink

# ---------------------------------------------------------------- настройки
CFG = {
    "rc_resume_delay_s": 2.0,     # пауза после потери пульта перед AUTO
    "gps_min_sats": 6,            # минимум спутников
    "gps_max_hdop": 2.5,          # максимум HDOP
    "gps_bad_confirm_s": 2.0,     # сколько GPS должен быть плохим
    "gps_good_confirm_s": 5.0,    # сколько GPS должен быть хорошим
    "spoof_speed_diff_mps": 1.5,  # расхождение скорости GPS и колёс
    "dr_max_time_s": 60.0,        # максимум езды по счислению пути
    "cruise_speed_mps": 2.0,      # обычная скорость
    "dr_speed_mps": 1.0,          # скорость при счислении пути
    "obstacle_stop_m": 1.0,       # стоп, если препятствие ближе
    "obstacle_clear_m": 1.5,      # путь свободен, если дальше
    "obstacle_clear_s": 2.0,      # сколько путь должен быть свободен
}

# Номера режимов ArduPilot Rover
MODES = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER",
         10: "AUTO", 11: "RTL", 12: "SMART_RTL", 15: "GUIDED"}
MODE_ID = {name: num for num, name in MODES.items()}
MANUAL_MODES = {"MANUAL", "ACRO", "STEERING"}

# Наборы источников навигации EKF3 (EK3_SRC1_* и EK3_SRC2_*)
SRC_GPS = 1
SRC_ODOMETRY = 2


class Dispatcher:
    def __init__(self, link, dry_run=False):
        self.link = link
        self.dry = dry_run
        self.ts = link.target_system
        self.tc = link.target_component

        self.mode = None
        self.armed = False

        self.rc_ok = True
        self.rc_lost_since = None
        self.mode_before_rc_loss = None
        self.rc_resume_done = False

        self.gps_bad_since = None
        self.gps_good_since = None
        self.gps_speed = None
        self.wheel_speed = None
        self._wheel_prev = None
        self.nav_source = SRC_GPS
        self.dr_since = None

        self.distance_m = None
        self.clear_since = None

        # Почему диспетчер сам поставил HOLD: None, "obstacle", "nav"
        self.own_hold = None
        self._pending, self._pending_t = None, 0.0

    # ----------------------------------------------------------- команды
    def _cmd(self, command, *params):
        p = list(params) + [0] * (7 - len(params))
        if self.dry:
            logging.info("[dry-run] команда %s %s", command, p)
            return
        self.link.mav.command_long_send(self.ts, self.tc, command, 0, *p)

    def set_mode(self, name, reason):
        if self.mode == name:
            return
        # Не повторяем ту же команду, пока автопилот её не подтвердил
        now = time.monotonic()
        if self._pending == name and now - self._pending_t < 1.0:
            return
        self._pending, self._pending_t = name, now
        logging.warning("Режим %s -> %s: %s", self.mode, name, reason)
        self._cmd(MAV.MAV_CMD_DO_SET_MODE,
                  MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_ID[name])

    def set_nav_source(self, source, reason):
        if self.nav_source == source:
            return
        logging.warning("Навигация: набор источников %d -> %d: %s",
                        self.nav_source, source, reason)
        self._cmd(MAV.MAV_CMD_SET_EKF_SOURCE_SET, source)
        self.nav_source = source

    def set_speed(self, mps):
        logging.info("Скорость маршрута: %.1f м/с", mps)
        self._cmd(MAV.MAV_CMD_DO_CHANGE_SPEED, 1, mps, -1)

    def request_streams(self):
        """Попросить контроллер присылать нужные сообщения (Гц)."""
        for msg_id, hz in ((MAV.MAVLINK_MSG_ID_SYS_STATUS, 2),
                           (MAV.MAVLINK_MSG_ID_GPS_RAW_INT, 5),
                           (MAV.MAVLINK_MSG_ID_WHEEL_DISTANCE, 10),
                           (MAV.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10)):
            self._cmd(MAV.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    # ------------------------------------------------------ телеметрия
    def handle(self, msg):
        if msg.get_srcSystem() != self.ts:
            return  # сообщения от наземной станции и др. пропускаем
        t = msg.get_type()

        if t == "HEARTBEAT" and msg.get_srcComponent() == self.tc:
            self.mode = MODES.get(msg.custom_mode, str(msg.custom_mode))
            self.armed = bool(msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)

        elif t == "SYS_STATUS":
            bit = MAV.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
            if msg.onboard_control_sensors_present & bit:
                self.rc_ok = bool(msg.onboard_control_sensors_health & bit)

        elif t == "GPS_RAW_INT":
            hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.0
            self.gps_speed = msg.vel / 100.0 if msg.vel != 65535 else None
            good = (msg.fix_type >= 3
                    and msg.satellites_visible >= CFG["gps_min_sats"]
                    and hdop <= CFG["gps_max_hdop"]
                    and not self._spoof_suspected())
            now = time.monotonic()
            if good:
                self.gps_bad_since = None
                self.gps_good_since = self.gps_good_since or now
            else:
                self.gps_good_since = None
                self.gps_bad_since = self.gps_bad_since or now

        elif t == "WHEEL_DISTANCE" and msg.count > 0:
            now_s = msg.time_usec / 1e6
            dist = msg.distance[0]
            if self._wheel_prev:
                dt = now_s - self._wheel_prev[0]
                if dt > 0:
                    self.wheel_speed = abs(dist - self._wheel_prev[1]) / dt
            self._wheel_prev = (now_s, dist)

        elif t == "DISTANCE_SENSOR":
            if msg.orientation == MAV.MAV_SENSOR_ROTATION_NONE:  # вперёд
                self.distance_m = msg.current_distance / 100.0

    def _spoof_suspected(self):
        """Подмена GPS: скорость по GPS не совпадает со скоростью колёс."""
        if self.gps_speed is None or self.wheel_speed is None:
            return False
        return abs(self.gps_speed - self.wheel_speed) > CFG["spoof_speed_diff_mps"]

    # ------------------------------------------------------ логика
    def step(self):
        now = time.monotonic()
        if self.mode is None or not self.armed:
            return
        self._rc_logic(now)
        self._nav_logic(now)
        self._obstacle_logic(now)

    def _rc_logic(self, now):
        if self.rc_ok:
            if self.rc_lost_since is not None:
                logging.warning("Связь с пультом восстановлена")
            self.rc_lost_since = None
            self.rc_resume_done = False
            self.mode_before_rc_loss = None
            return

        if self.rc_lost_since is None:
            self.rc_lost_since = now
            self.mode_before_rc_loss = self.mode
            logging.warning("Связь с пультом потеряна (режим %s)", self.mode)

        # Продолжаем маршрут, только если оператор ехал вручную.
        # Если оператор сам поставил HOLD до потери связи — стоим.
        was_manual = self.mode_before_rc_loss in MANUAL_MODES
        waited = now - self.rc_lost_since >= CFG["rc_resume_delay_s"]
        if (was_manual and waited and not self.rc_resume_done
                and self.own_hold is None and self.mode != "AUTO"):
            self.set_mode("AUTO", "нет пульта — продолжаю маршрут")
            self.rc_resume_done = True

    def _nav_logic(self, now):
        gps_bad = (self.gps_bad_since is not None
                   and now - self.gps_bad_since >= CFG["gps_bad_confirm_s"])
        gps_good = (self.gps_good_since is not None
                    and now - self.gps_good_since >= CFG["gps_good_confirm_s"])

        if self.nav_source == SRC_GPS and gps_bad:
            self.set_nav_source(SRC_ODOMETRY, "GPS потерян или недостоверен")
            self.set_speed(CFG["dr_speed_mps"])
            self.dr_since = now

        elif self.nav_source == SRC_ODOMETRY and gps_good:
            self.set_nav_source(SRC_GPS, "GPS снова в норме")
            self.set_speed(CFG["cruise_speed_mps"])
            self.dr_since = None
            if self.own_hold == "nav" and self.mode == "HOLD":
                self.own_hold = None
                self.set_mode("AUTO", "навигация восстановлена")

        elif (self.nav_source == SRC_ODOMETRY and self.dr_since
              and now - self.dr_since > CFG["dr_max_time_s"]
              and self.mode == "AUTO"):
            self.own_hold = "nav"
            self.set_mode("HOLD", "счисление пути слишком долго — жду GPS")

    def _obstacle_logic(self, now):
        d = self.distance_m
        if d is None:
            return
        if self.mode == "AUTO" and d < CFG["obstacle_stop_m"]:
            self.own_hold = "obstacle"
            self.clear_since = None
            self.set_mode("HOLD", "препятствие в %.2f м" % d)
            return
        if self.own_hold == "obstacle":
            if self.mode != "HOLD":      # оператор сам сменил режим
                self.own_hold = None
                return
            if d > CFG["obstacle_clear_m"]:
                self.clear_since = self.clear_since or now
                if now - self.clear_since >= CFG["obstacle_clear_s"]:
                    self.own_hold = None
                    self.set_mode("AUTO", "путь свободен")
            else:
                self.clear_since = None


def wait_vehicle(link):
    """Ждём heartbeat именно от автопилота, а не от наземной станции."""
    while True:
        hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if hb is None:
            logging.info("Жду автопилот...")
            continue
        if hb.type != MAV.MAV_TYPE_GCS and hb.autopilot != MAV.MAV_AUTOPILOT_INVALID:
            link.target_system = hb.get_srcSystem()
            link.target_component = hb.get_srcComponent()
            return


def main():
    ap = argparse.ArgumentParser(description="Бортовой диспетчер машины")
    ap.add_argument("--connect", default="udpin:127.0.0.1:14551")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--log", default="dispatcher.log")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=[logging.StreamHandler(), logging.FileHandler(args.log, encoding="utf-8")])

    link = mavutil.mavlink_connection(
        args.connect, baud=args.baud,
        source_system=1, source_component=MAV.MAV_COMP_ID_ONBOARD_COMPUTER)
    wait_vehicle(link)
    logging.info("Автопилот найден: система %d", link.target_system)

    d = Dispatcher(link, dry_run=args.dry_run)
    d.request_streams()
    last_hb = 0.0
    while True:
        msg = link.recv_match(blocking=True, timeout=0.1)
        if msg is not None:
            d.handle(msg)
        d.step()
        if time.monotonic() - last_hb > 1.0:   # сообщаем, что диспетчер жив
            link.mav.heartbeat_send(MAV.MAV_TYPE_ONBOARD_CONTROLLER,
                                    MAV.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = time.monotonic()


if __name__ == "__main__":
    main()
