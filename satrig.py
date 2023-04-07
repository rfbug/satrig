# Copyright 2023 Michael Wichmann
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the “Software”),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.

import os
import sys
import configparser
import time
import datetime
import numpy as np
import predict
from asciimatics.screen import ManagedScreen, Screen, KeyboardEvent, MouseEvent
import socket
import serial
import argparse

# Gpredict configuration file and directories
GPREDICT_CONFIG_DIR = os.path.expanduser("~/.config/Gpredict")
GPREDICT_CONFIG_FILE = os.path.join(GPREDICT_CONFIG_DIR, "gpredict.cfg")
GPREDICT_MODULE_DIR = os.path.join(GPREDICT_CONFIG_DIR, "modules")
GPREDICT_SATDATA_DIR = os.path.join(GPREDICT_CONFIG_DIR, "satdata")
GPREDICT_TRSP_DIR = os.path.join(GPREDICT_CONFIG_DIR, "trsp")

# Main loop update interval
UI_INTERVAL_MS = 25
CALC_INTERVAL_MS = 300


def grphex(ba):
    s = ba.hex()
    t = iter(s)
    return " ".join(a+b for a,b in zip(t,t))

def printhex(ba):
    print(grphex(ba))


class Satellite():
    def __init__(self, id, name, tle, qth, trsp={}):
        self.id = id
        self.name = name
        self.tle = tle
        self.trsp = self._build_trsp_list(trsp)
        self.qth = qth
        self.update()

    def update(self):
        new_obs = predict.observe(self.tle, self.qth)
        self.obs = new_obs

    def doppler_at_f(self, f, tx=False):
        doppler100 = self.obs["doppler"]
        doppler = f/100e6 * doppler100
        if not tx:
            return doppler
        else:
            return -doppler

    def doppler_fsat_from_fobs(self, fobs, tx=False, r=0):
        if r == 0:
            r = self.doppler_r
        if not tx:
            return fobs / r
        else:
            return fobs * r

    def doppler_fobs_from_fsat(self, fsat, tx=False, r=0):
        if r == 0:
            r = self.doppler_r
        if not tx:
            return fsat * r
        else:
            return fsat / r

    @property
    def doppler_r(self):
        doppler100 = self.obs["doppler"]
        return 1 + (doppler100 / 100e6)

    def _build_trsp_list(self, transponder):
        tlist = []
        for trsp_name in transponder:
            # Filter out DEFAULT section
            if "DEFAULT" in trsp_name:
                continue
            trsp = transponder[trsp_name]
            inverting = False
            if "INVERT" in trsp:
                inverting = True if trsp["INVERT"] == "true" else False
            f_dwn = int(trsp["DOWN_LOW"])
            f_dwn_delta = 0
            if "DOWN_HIGH" in trsp:
                f_dwn = (f_dwn + int(trsp["DOWN_HIGH"])) // 2
                f_dwn_delta = int(trsp["DOWN_HIGH"]) - int(trsp["DOWN_LOW"])
            else:
                f_dwn_delta = 0
            f_up = 0
            f_up_delta = 0
            if "UP_LOW" in trsp:
                f_up = int(trsp["UP_LOW"])
                if "UP_HIGH" in trsp:
                    f_up = (f_up + int(trsp["UP_HIGH"])) // 2
                    f_up_delta = int(trsp["UP_HIGH"]) - int(trsp["UP_LOW"])
                else:
                    f_up_delta = 0
            mode = trsp["MODE"]
            tlist += [
                {
                    "name": trsp_name,
                    "inverting": inverting,
                    "mode": mode,
                    "f_dwn": f_dwn,
                    "f_up": f_up,
                    "f_dwn_delta": f_dwn_delta,
                    "f_up_delta": f_up_delta
                }
            ]
        return tlist




class Icom821H():
    def __init__(self, serport, baud):
        self.ser = serial.Serial(serport, baud, timeout=0.1)

    def get_freq(self):
        rsp = self.cmd(b"\x03", 11)
        data = rsp[11:16].hex()
        try:
            f = int(data[8]+data[9]+data[6]+data[7]+data[4]+
                data[5]+data[2]+data[3]+data[0]+data[1])
        except:
            f = 0
        return f

    def set_freq(self, f):
        f = f"{f:010d}"
        f = f[8]+f[9]+f[6]+f[7]+f[4]+f[5]+f[2]+f[3]+f[0]+f[1]
        f = bytearray.fromhex(f)
        rsp = self.cmd(b"\x05" + f)

    def main_access(self):
        self.cmd(b"\x07\xd0")

    def sub_access(self):
        self.cmd(b"\x07\xd1")

    def set_vfo_a(self):
        self.cmd(b"\x07\x00")

    def set_vfo_b(self):
        self.cmd(b"\x07\x01")

    def xchg_main_sub(self):
        self.cmd(b"\x07\xb0")

    def set_simplex(self):
        self.cmd(b"\x0f\x10")

    def set_mode(self, mode):
        md = b""
        if "LSB" in mode:
            md = b"\x00"
        elif "USB" in mode:
            md = b"\x01"
        elif "CW" in mode:
            md = b"\x03\x01"
        elif "FM" in mode:
            md = b"\x05"
        else:
            return
        self.cmd(b"\x06" + md)

    def cmd(self, cnscdata, nrsp=6):
        PRE = b"\xFE\xFE\x4C\xE0"
        EOM = b"\xFD"
        msg = PRE + cnscdata + EOM
        nrcv = nrsp + len(msg)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(msg)
        return self.ser.read(nrcv)


class Rig():
    def __init__(self, port=None, baud=None):
        try:
            self.icom = Icom821H(port, baud)
        except serial.SerialException:
            self.icom = None

        # Init rig
        if self.icom:
            self.icom.main_access()
            self.icom.set_simplex()
            self.icom.set_vfo_a()
            self.icom.set_mode("USB")
            self.icom.sub_access()
            self.icom.set_simplex()
            self.icom.set_vfo_a()
            self.icom.set_mode("USB")

        self._f_main = 0
        self._f_sub = 0

        self.read()

    @property
    def f_main(self):
        return self._f_main

    @property
    def f_sub(self):
        return self._f_sub

    @f_main.setter
    def f_main(self, value):
        if self.icom:
            self.icom.main_access()
            if str(value)[0] != str(self._f_main)[0]:
                self.icom.xchg_main_sub()
                self._f_sub = self._f_main
            self.icom.set_freq(int(value))
            self.icom.sub_access()
        self._f_main = value

    @f_sub.setter
    def f_sub(self, value):
        if self.icom:
            self.icom.sub_access()
            if str(value)[0] != str(self._f_sub)[0]:
                self.icom.xchg_main_sub()
                self._f_main = self._f_sub
            self.icom.set_freq(int(value))
        self._f_sub = value

    def set_mode_main(self, mode):
        if self.icom:
            self.icom.main_access()
            self.icom.set_mode(mode)
            self.icom.sub_access()

    def set_mode_sub(self, mode):
        if self.icom:
            self.icom.sub_access()
            self.icom.set_mode(mode)

    def read(self):
        # Read frequencie from radio but keep fractional parts to avoid
        # rounding errors if frequencies have not changed
        if self.icom:
            self.icom.main_access()
            f = self.icom.get_freq()
            if f > 0:
                if int(self._f_main) != f:
                    self._f_main = f
            self.icom.sub_access()
            f = self.icom.get_freq()
            if f > 0:
                if int(self._f_sub) != f:
                    self._f_sub = f


class Application():
    def __init__(self, satellites, port=None, baud=None):
        self.satellites = satellites
        # Currently selected satellite
        self.current_sat = self.satellites[0]
        # Currently selected transponder
        self.current_trsp = 0
        # Program state engage and tracking
        self.is_engaged = False
        # Indicates rig ctrl activity
        self.txrig = False
        # Rig
        self.rig = Rig(port, baud)
        # Start main loop and pass screen object
        try:
            Screen.wrapper(self._loop)
        except KeyboardInterrupt:
            pass

    def _next_sat(self):
        idx = self.satellites.index(self.current_sat)
        n = len(self.satellites)
        self.current_sat = self.satellites[(idx+1)%n]
        self.current_trsp = 0
        self.is_engaged = False

    def _previous_sat(self):
        idx = self.satellites.index(self.current_sat)
        n = len(self.satellites)
        self.current_sat = self.satellites[(idx-1)%n]
        self.current_trsp = 0
        self.is_engaged = False

    def _next_trsp(self):
        self.current_trsp = (self.current_trsp + 1) % len(self.current_sat.trsp)

    def _previous_trsp(self):
        self.current_trsp = (self.current_trsp - 1) % len(self.current_sat.trsp)

    def _toggle_engage(self):
        self.is_engaged = not self.is_engaged

    def format_freq(self, freq, sign=False):
        freq = int(freq)
        if sign:
            return ("+" if freq>0 else "") + f"{freq:,}".replace(",", " ")
        else:
            return f"{freq:,}".replace(",", " ")

    def _loop(self, screen):
        CALC_INTERVAL_START = CALC_INTERVAL_MS / UI_INTERVAL_MS
        calc_interval = 0
        self.f_main_old = self.rig.f_main
        self.f_sub_old = self.rig.f_sub
        self.r_old = self.current_sat.doppler_r
        while True:
            # Decrement calculation interval counter and do an actual
            # predict recalculation when the counter reaches zero.
            calc_interval -= 1
            if calc_interval <= 0:
                # Read rig
                self.rig.read()
                # Do satellite calculations
                try:
                    self.current_sat.update()
                except AttributeError:
                    pass
                calc_interval = CALC_INTERVAL_START
                # Do rig control
                if self.is_engaged:
                    # Adjust to current doppler shift if change is above threshold
                    f_th = 10
                    sat = self.current_sat
                    # Check if frequencies have changes. if so do not adjust
                    # doppler, as we are assuming operator is adjusting dials
                    f_main_changed = bool(self.rig.f_main != self.f_main_old)
                    f_sub_changed = bool(self.rig.f_sub != self.f_sub_old)
                    # Calculate doppler shifted frequencies
                    f_main_new = sat.doppler_fobs_from_fsat(
                        sat.doppler_fsat_from_fobs(
                            self.rig.f_main, r=self.r_old, tx=True
                        ), tx=True
                    )
                    f_sub_new = sat.doppler_fobs_from_fsat(
                        sat.doppler_fsat_from_fobs(
                            self.rig.f_sub, r=self.r_old,
                        )
                    )
                    # Apply only when diff frequency above threshold
                    df_main = abs(self.rig.f_main - f_main_new)
                    df_sub = abs(self.rig.f_sub - f_sub_new)
                    if (df_main >= f_th) or (df_sub >= f_th):
                        self.r_old = sat.doppler_r
                        if not f_main_changed:
                            self.rig.f_main = f_main_new
                        if not f_sub_changed:
                            self.rig.f_sub = f_sub_new
                        self.txrig = True
                else:
                    self.r_old = self.current_sat.doppler_r

                # Save rig frequencies for next loop
                self.f_main_old = self.rig.f_main
                self.f_sub_old = self.rig.f_sub


            # Sleep for a short while
            time.sleep(UI_INTERVAL_MS / 1000)
            #screen.clear()
            screen.clear_buffer(Screen.COLOUR_WHITE, Screen.A_NORMAL, Screen.COLOUR_BLACK)
            sat = self.current_sat
            obs = sat.obs
            ny, nx = screen.dimensions
            # Print title line
            screen.print_at("satrig v1.0", 0, 0)
            screen.move(0, 1)
            screen.draw(11, 1, thin=True)
            lat = sat.qth[0]
            lon = sat.qth[1]
            alt = sat.qth[2]
            latdesig = "N" if lat > 0 else "S"
            longdesig = "W" if lon > 0 else "E" # this is predict convention
            screen.print_at(f"QTH: {abs(lat):06.3f}°{latdesig} {abs(lon):07.3f}°{longdesig} {alt:.0f}m", 20, 0)
            utc = datetime.datetime.now().astimezone(datetime.timezone.utc)
            screen.print_at(f"{utc.strftime('UTC %H:%M:%S %Y-%m-%d')}", nx-24, 0)
            # Print satellite information
            SAT_LINE = 2
            az = obs["azimuth"]
            el = obs['elevation']
            col = Screen.COLOUR_DEFAULT
            if el > 20:
                col = Screen.COLOUR_GREEN
            elif el > 0:
                col = Screen.COLOUR_YELLOW
            screen.move(5, SAT_LINE)
            screen.draw(35, SAT_LINE, colour=col)
            screen.print_at("SAT:", 0, SAT_LINE)
            screen.print_at(sat.name, 6, SAT_LINE, col, Screen.A_REVERSE)
            screen.print_at(f"Az: {az:5.1f}°", 38, SAT_LINE, col)
            screen.print_at(f"El: {el:+4.1f}°", 50, SAT_LINE, col)
            screen.print_at(f"Δf@100Mhz: {obs['doppler']:+5.0f}Hz", 62, SAT_LINE, col)
            # Rig
            RIG_LINE = 4
            #screen.print_at("RIG:", 0, RIG_LINE)
            col = Screen.COLOUR_RED
            att = Screen.A_NORMAL
            if self.is_engaged:
                att = Screen.A_REVERSE
            screen.print_at(" ENGAGED ", 5, RIG_LINE+1, col, att)
            att = Screen.A_NORMAL
            if self.txrig:
                self.txrig = False
                screen.print_at("  RADIO  ",
                    5, RIG_LINE+3, Screen.COLOUR_RED, Screen.A_REVERSE)
            col = Screen.COLOUR_WHITE
            att = Screen.A_NORMAL
            screen.print_at("   VFO RX [Hz]", 45, RIG_LINE, col, att)
            screen.print_at("   VFO TX [Hz]", 61, RIG_LINE, col, att)
            screen.print_at("     Dial", 34, RIG_LINE+1, col, att)
            screen.print_at("  Doppler", 34, RIG_LINE+2, col, att)
            screen.print_at("Satellite", 34, RIG_LINE+3, col, att)
            f_main = self.rig.f_main
            f_sub = self.rig.f_sub
            f_main_sat = self.current_sat.doppler_fsat_from_fobs(f_main, tx=True)
            f_sub_sat = self.current_sat.doppler_fsat_from_fobs(f_sub)
            doppler_main = self.current_sat.doppler_at_f(f_main_sat, tx=True)
            doppler_sub = self.current_sat.doppler_at_f(f_sub_sat)

            col = Screen.COLOUR_GREEN
            att = Screen.A_REVERSE
            screen.print_at("{:>14}".format(self.format_freq(f_sub)), 45, RIG_LINE+1, col, att)
            col = Screen.COLOUR_YELLOW
            screen.print_at("{:>14}".format(self.format_freq(f_main)), 61, RIG_LINE+1, col, att)
            col = Screen.COLOUR_MAGENTA
            att = Screen.A_NORMAL
            screen.print_at("{:>14}".format(self.format_freq(doppler_sub, True)), 45, RIG_LINE+2, col, att)
            screen.print_at("{:>14}".format(self.format_freq(doppler_main, True)), 61, RIG_LINE+2, col, att)
            att = Screen.A_BOLD
            col = Screen.COLOUR_GREEN
            screen.print_at("{:>14}".format(self.format_freq(f_sub_sat)), 45, RIG_LINE+3, col, att)
            col = Screen.COLOUR_YELLOW
            screen.print_at("{:>14}".format(self.format_freq(f_main_sat)), 61, RIG_LINE+3, col, att)

            # Print transponder frequency info
            trsp = self.current_sat.trsp[self.current_trsp]
            col = Screen.COLOUR_GREEN
            att = Screen.A_NORMAL
            if trsp["f_dwn_delta"]:
                n = 13
                f_lo = trsp["f_dwn"] - trsp["f_dwn_delta"]/2
                f_hi = trsp["f_dwn"] + trsp["f_dwn_delta"]/2
                screen.print_at(n*"ᐧ", 46, RIG_LINE+4, col, att)
                if f_sub_sat < f_lo:
                    screen.print_at("<<<", 46, RIG_LINE+4, col, att)
                elif f_sub_sat > f_hi:
                    screen.print_at(">>>", 56, RIG_LINE+4, col, att)
                else:
                    x = round((f_sub_sat - f_lo) / (f_hi - f_lo) * (n-1))
                    screen.print_at("|", 46+x, RIG_LINE+4, col, att)
            col = Screen.COLOUR_YELLOW
            att = Screen.A_NORMAL
            if trsp["f_up_delta"]:
                n = 13
                f_lo = trsp["f_up"] - trsp["f_up_delta"]/2
                f_hi = trsp["f_up"] + trsp["f_up_delta"]/2
                screen.print_at(n*"ᐧ", 61, RIG_LINE+4, col, att)
                if f_main_sat < f_lo:
                    screen.print_at("<<<", 61, RIG_LINE+4, col, att)
                elif f_main_sat > f_hi:
                    screen.print_at(">>>", 71, RIG_LINE+4, col, att)
                else:
                    x = round((f_main_sat - f_lo) / (f_hi - f_lo) * (n-1))
                    screen.print_at("|", 61+x, RIG_LINE+4, col, att)



            # Print transponder info
            TRSP_LINE = 9
            col = Screen.COLOUR_WHITE
            att = Screen.A_BOLD
            screen.print_at("Nr  Transponder", 0, TRSP_LINE, col, att)
            screen.print_at("Mode", 36, TRSP_LINE, col, att)
            screen.print_at("I", 42, TRSP_LINE, col, att)
            screen.print_at(" Downlink [Hz]", 45, TRSP_LINE, col, att)
            screen.print_at("   Uplink [Hz]", 61, TRSP_LINE, col, att)

            for i, trsp in enumerate(sat.trsp):
                if i == self.current_trsp:
                    att = Screen.A_REVERSE
                else:
                    att = Screen.A_NORMAL
                if "lin" in trsp["name"].lower():
                    col = Screen.COLOUR_GREEN
                elif "beac" in trsp["name"].lower():
                    col = Screen.COLOUR_BLUE
                elif "fm" in trsp["name"].lower():
                    col = Screen.COLOUR_YELLOW
                else:
                    col = Screen.COLOUR_WHITE
                f_dwn = f"{trsp['f_dwn']:,}".replace(",", " ")
                f_up = "-"
                if trsp["f_up"] > 0:
                    f_up = f"{trsp['f_up']:,}".replace(",", " ")
                screen.print_at(nx*" ", 0, TRSP_LINE+i+1, col, att)
                screen.print_at(f"{i:2d}  {trsp['name'][:30]}", 0, TRSP_LINE+i+1, col, att)
                screen.print_at(f"{trsp['mode']:>4}", 36, TRSP_LINE+i+1, col, att)
                screen.print_at("I" if trsp["inverting"] else "-", 42, TRSP_LINE+i+1, col, att)
                screen.print_at(f"{f_dwn:>14}", 45, TRSP_LINE+i+1, col, att)
                screen.print_at(f"{f_up:>14}", 61, TRSP_LINE+i+1, col, att)

            # Print help line at bottom
            screen.print_at("🡄 🡆 : sat", 0, ny-1)
            screen.print_at("🡅 🡇 : transponder", 13, ny-1)
            #screen.print_at("e: engage   t: track   x: set freq   ctrl-c: quit", 31, ny-1)
            screen.print_at("    e: engage      x: set freq       ctrl-c: quit", 31, ny-1)

            # Handle events
            evt = screen.get_event()
            if isinstance(evt, KeyboardEvent):
                # Right arrow = next satellite
                if evt.key_code == Screen.KEY_RIGHT:
                    self._next_sat()
                # Left arrow = previous satellite
                elif evt.key_code == Screen.KEY_LEFT:
                    self._previous_sat()
                # Down arrow = next transponder
                elif evt.key_code == Screen.KEY_DOWN:
                    self._next_trsp()
                # Up arrow = previous transponder
                elif evt.key_code == Screen.KEY_UP:
                    self._previous_trsp()
                # Engage
                elif evt.key_code == ord('e'):
                    self._toggle_engage()
                # Set freq
                elif evt.key_code == ord("x"):
                    trsp = self.current_sat.trsp[self.current_trsp]
                    f = trsp["f_dwn"]
                    fobs = self.current_sat.doppler_fobs_from_fsat(f)
                    self.rig.f_sub = fobs
                    f = trsp["f_up"]
                    if f > 0:
                        fobs = self.current_sat.doppler_fobs_from_fsat(f, tx=True)
                        self.rig.f_main = fobs
                    if trsp["mode"] in ["USB", "LSB", "CW", "FM"]:
                        self.rig.set_mode_sub(trsp["mode"])
                    else:
                        self.rig.set_mode_sub("USB")
                    if trsp["inverting"]:
                        if trsp["mode"] == "USB":
                            self.rig.set_mode_main("LSB")
                        elif trsp["mode"] == "LSB":
                            self.rig.set_mode_main("USB")
                    else:
                        self.rig.set_mode_main(trsp["mode"])
                elif evt.key_code == ord("r"):
                    self.rig.f_main = 145000000
                    self.rig.f_sub = 435000000
                elif evt.key_code == ord("p"):
                    self.rig.f_main += 1000
                elif evt.key_code == ord("m"):
                    self.rig.f_main -= 1000
                elif evt.key_code == ord("o"):
                    self.rig.f_sub += 1000
                elif evt.key_code == ord("n"):
                    self.rig.f_sub -= 1000
            # Refresh screen
            screen.refresh()


def main(args):
    if not os.path.isdir(GPREDICT_CONFIG_DIR):
        print("No Gpredict config data found. Exiting.")
        sys.exit()
    # Read open modules from Gpredict config file
    config = configparser.ConfigParser()
    config.read_file(open(GPREDICT_CONFIG_FILE))
    open_modules = config["GLOBAL"]["OPEN_MODULES"].split(";")
    # Read qth information
    default_qth = config["GLOBAL"]["DEFAULT_QTH"]
    try:
        with open(os.path.join(GPREDICT_CONFIG_DIR, default_qth)) as f:
            qth_config = configparser.ConfigParser()
            qth_config.read_file(f)
            lat = float(qth_config["QTH"]["LAT"])
            lon = float(qth_config["QTH"]["LON"])
            alt = float(qth_config["QTH"]["ALT"])
            qth = (lat, -lon, alt)
    except:
        print(f"Can't open '{default_qth}'")
        sys.exit()
    # Open the modules and compile a list satellites
    satids = []
    for module in open_modules:
        module_fname = os.path.join(GPREDICT_MODULE_DIR, module + ".mod")
        try:
            with open(module_fname) as f:
                module_config = configparser.ConfigParser()
                module_config.read_file(f)
                satids += module_config["GLOBAL"]["SATELLITES"].split(";")
        except EnvironmentError:
            print(f"Can't open '{module_fname}'")
            sys.exit()
    # Create a Satellite object for every sat in list
    satellites = []
    for satid in satids:
        satdata_fname = os.path.join(GPREDICT_SATDATA_DIR, satid + ".sat")
        satellite = None
        # Get basic satellite information
        try:
            with open(satdata_fname) as f:
                sat_config = configparser.ConfigParser()
                sat_config.read_file(f)
                version = sat_config["Satellite"]["VERSION"]
                if version != "1.1":
                    print(f"Unknown data format version {version}")
                    sys.exit()
                name = sat_config["Satellite"]["NAME"]
                tle1 = sat_config["Satellite"]["TLE1"]
                tle2 = sat_config["Satellite"]["TLE2"]
                tle = "\n".join(['0 ' + name, tle1, tle2])
        except EnvironmentError:
            print(f"Can't open '{satdata_fname}")
            sys.exit()
        # Get transponder information
        trsp_fname = os.path.join(GPREDICT_TRSP_DIR, satid + ".trsp")
        try:
            with open(trsp_fname) as f:
                trsp_config = configparser.ConfigParser()
                trsp_config.read_file(f)
        except EnvironmentError:
            print(f"Can´t open '{trsp_config}")
            trsp_config = {}
        # Finally create Satellite object and add it to the list
        satellite = Satellite(satid, name=name, tle=tle, qth=qth, trsp=trsp_config)
        satellites += [ satellite ]
    # Create application object and start the main loop
    app = Application(satellites, port=args.port[0], baud=args.baud[0])
    #app.loop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Satellite rig control for IC-821H",
        prog="satrig"
    )
    parser.add_argument("-p", "--port", nargs=1, default=["/dev/ttyUSB0"])
    parser.add_argument("-b", "--baud", nargs=1, type=int, default=[19200])
    args = parser.parse_args()
    main(args)
