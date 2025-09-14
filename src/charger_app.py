#!/usr/bin/env python3
# Minimal web UI (no Flask) to start a prepaid charging session with an energy budget (Wh).
# - PZEM-017 over RS-485/Modbus RTU (manual register decode, robust to word order)
# - Budget cutoff uses integrated energy (W × Δt), so tiny budgets (e.g., 0.15 Wh) are accurate
# - Switch control on Raspberry Pi 5: RPi.GPIO if present, else pinctrl fallback

import threading, time, json, subprocess
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

import minimalmodbus, serial

# ===================== USER CONFIG =====================
SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"  # <-- change to YOUR by-id path
SLAVE_ADDR  = 1
BAUD        = 9600

GPIO_PIN    = 18     # BCM numbering (GPIO18 = physical pin 12)
POLL_SEC    = 0.5
HOST        = "0.0.0.0"
PORT        = 5000
# =======================================================

# -------- GPIO (Pi 5) --------
USE_GPIO_LIB = True
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_PIN, GPIO.OUT, initial=GPIO.LOW)
except Exception:
    USE_GPIO_LIB = False

def gpio_high():
    if USE_GPIO_LIB:
        GPIO.output(GPIO_PIN, GPIO.HIGH)
    else:
        # Pi 5 CLI; try without sudo first (run app with sudo if needed)
        r = subprocess.run(["pinctrl","set",str(GPIO_PIN),"op","dh"], capture_output=True)
        if r.returncode != 0:
            subprocess.run(["sudo","pinctrl","set",str(GPIO_PIN),"op","dh"])

def gpio_low():
    if USE_GPIO_LIB:
        GPIO.output(GPIO_PIN, GPIO.LOW)
    else:
        r = subprocess.run(["pinctrl","set",str(GPIO_PIN),"op","dl"], capture_output=True)
        if r.returncode != 0:
            subprocess.run(["sudo","pinctrl","set",str(GPIO_PIN),"op","dl"])

# -------- PZEM link --------
inst = minimalmodbus.Instrument(SERIAL_PORT, SLAVE_ADDR)
inst.serial.baudrate = BAUD
inst.serial.bytesize = 8
inst.serial.parity   = serial.PARITY_NONE
inst.serial.stopbits = 2
inst.serial.timeout  = 1.0
inst.mode = minimalmodbus.MODE_RTU
# byteorder not used; we decode manually to avoid ambiguity
inst.clear_buffers_before_each_transaction = True
inst.close_port_after_each_call = True

# bus lock so reader & reset never collide
bus_lock = threading.Lock()

def read_all():
    """
    Return (V, A, W, Wh) as floats.
    Reads 7 input registers from 0x0000 and assembles 32-bit values manually.
    Scales: V=0.01 V, A=0.01 A, P=0.1 W, E=1 Wh.
    Word order differs across firmwares, so we try both and sanity-pick.
    """
    with bus_lock:
        regs = inst.read_registers(0x0000, 7, functioncode=4)  # 0..6
    # single-reg quantities
    V = regs[0] / 100.0
    A = regs[1] / 100.0

    # two-reg 32-bit: power at regs[2]&[3], energy at regs[4]&[5] (typical)
    lo_p, hi_p = regs[2], regs[3]
    lo_e, hi_e = regs[4], regs[5]

    # assemble both ways, pick the sane one (0..5000 W)
    P_a = ((hi_p << 16) | lo_p) / 10.0
    P_b = ((lo_p << 16) | hi_p) / 10.0
    if 0.0 <= P_a < 5000.0:
        P = P_a
    elif 0.0 <= P_b < 5000.0:
        P = P_b
    else:
        # fall back to the smaller magnitude
        P = P_a if abs(P_a) < abs(P_b) else P_b

    E_a = float((hi_e << 16) | lo_e)
    E_b = float((lo_e << 16) | hi_e)
    # prefer the one that doesn't jump absurdly if you have a baseline
    E = E_a if E_a <= E_b * 2 else E_b

    return V, A, P, E

def _crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, "little")  # low, high

def reset_energy_counter():
    """
    PZEM-017 vendor reset (function 0x42, no payload).
    Raw RTU frame: [addr][0x42][CRClo][CRChi]
    """
    frame = bytes([SLAVE_ADDR, 0x42]) + _crc16_modbus(bytes([SLAVE_ADDR, 0x42]))
    with bus_lock:
        with serial.Serial(SERIAL_PORT, BAUD, bytesize=8, parity=serial.PARITY_NONE,
                           stopbits=2, timeout=1.0) as s:
            s.write(frame)
            try: s.read(5)
            except Exception: pass

# -------- Session state --------
state = {
    "running": False,
    "target_Wh": 0.0,
    "delivered_Wh": 0.0,      # meter delta (whole-Wh resolution)
    "delivered_Wh_int": 0.0,  # integrated energy W×Δt (used for cut-off)
    "baseline_Wh": 0.0,
    "last": {"V": None, "A": None, "W": None, "Wh": None},
    "last_ts": None,
    "error": None,
}
lock = threading.Lock()

def session_worker(target_Wh):
    # Snapshot baseline for display (we'll cut off using integration)
    try:
        _, _, _, e0 = read_all()
    except Exception as e:
        with lock:
            state.update(running=False, error=f"PZEM read failed: {e}")
        gpio_low()
        return

    with lock:
        state.update(running=True, target_Wh=target_Wh, baseline_Wh=e0,
                     delivered_Wh=0.0, delivered_Wh_int=0.0,
                     last_ts=time.time(), error=None)

    gpio_high()
    try:
        while True:
            time.sleep(POLL_SEC)
            try:
                V, A, W, Wh = read_all()
                now = time.time()
                with lock:
                    prev_ts = state["last_ts"]
                    dt = max(0.0, now - (prev_ts or now))
                    state["last"] = {"V": V, "A": A, "W": W, "Wh": Wh}
                    state["delivered_Wh"] = max(0.0, Wh - state["baseline_Wh"])
                    if 0.0 <= W < 5000.0:
                        state["delivered_Wh_int"] += (W * dt) / 3600.0
                    state["last_ts"] = now
                    if state["delivered_Wh_int"] >= state["target_Wh"]:
                        break
            except Exception as e:
                with lock:
                    state["error"] = f"PZEM error: {e}"
                continue
    finally:
        gpio_low()
        with lock:
            state["running"] = False

HTML = """<!doctype html>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Prepaid Charger</title>
<style>
 body { font-family: system-ui, sans-serif; margin: 20px; max-width: 640px }
 .card { border:1px solid #ddd; border-radius:10px; padding:12px; margin:10px 0 }
 input[type=number]{ width:9em }
 button{ padding:6px 12px }
 .grid { display:grid; grid-template-columns:1fr 1fr; gap:8px }
</style>
<h2>Prepaid Charging</h2>
<div class="card">
  <form onsubmit="return startSession();">
    Energy budget (Wh):
    <input id="budget" type="number" step="0.01" min="0.01" inputmode="decimal" required>
    <button type="submit">Start</button>
  </form>
  <form method="GET" action="/stop" style="display:inline"><button>Stop</button></form>
  <form method="GET" action="/reset" style="display:inline"
        onsubmit="return confirm('Reset meter energy counter?');"><button>Reset meter</button></form>
  <form method="GET" action="/softzero" style="display:inline"><button>Soft zero</button></form>
  <small>Tip: For a ~7 min demo at ~1.3 W, use ~0.15 Wh.</small>
</div>
<div class="card">
  <b>Status:</b> <span id="status">-</span><br>
  <div class="grid">
    <div>V: <span id="v">-</span></div>
    <div>A: <span id="a">-</span></div>
    <div>W: <span id="w">-</span></div>
    <div>Meter Wh: <span id="mwh">-</span></div>
    <div>Delivered Wh (int): <span id="dwhi">-</span></div>
    <div>Target Wh: <span id="twh">-</span></div>
  </div>
  <div id="err" style="color:#b00"></div>
</div>
<script>
function startSession(){
  const v = document.getElementById('budget').value;
  if(!v) return false;
  fetch('/start?wh='+encodeURIComponent(v)).then(()=>setTimeout(update,200));
  return false; // prevent navigation
}
function update(){
  fetch('/json').then(r=>r.json()).then(s=>{
    document.getElementById('status').textContent = s.running ? 'RUNNING' : 'IDLE';
    document.getElementById('v').textContent   = s.last.V==null ? '-' : s.last.V.toFixed(2);
    document.getElementById('a').textContent   = s.last.A==null ? '-' : s.last.A.toFixed(2);
    document.getElementById('w').textContent   = s.last.W==null ? '-' : s.last.W.toFixed(2);
    document.getElementById('mwh').textContent = s.last.Wh==null ? '-' : s.last.Wh.toFixed(0);
    document.getElementById('dwhi').textContent= s.delivered_Wh_int.toFixed(3);
    document.getElementById('twh').textContent = s.target_Wh.toFixed(3);
    document.getElementById('err').textContent = s.error || '';
  }).catch(()=>{}).finally(()=>setTimeout(update,2000));
}
update();
</script>
"""

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        try:
            p = urlparse(self.path)
            if p.path == "/":
                self._ok(HTML)
            elif p.path == "/start":
                qs = parse_qs(p.query)
                wh = float(qs.get("wh",[0])[0])
                with lock:
                    if not state["running"]:
                        state["error"] = None
                        threading.Thread(target=session_worker, args=(wh,), daemon=True).start()
                self._redirect("/")
            elif p.path == "/stop":
                with lock: state["running"] = False
                gpio_low()
                self._redirect("/")
            elif p.path == "/reset":
                try:
                    reset_energy_counter()
                except Exception as e:
                    with lock: state["error"] = f"Reset failed: {e}"
                self._redirect("/")
            elif p.path == "/softzero":
                try:
                    _, _, _, e0 = read_all()
                    with lock:
                        state["baseline_Wh"] = e0
                        state["delivered_Wh"] = 0.0
                        state["delivered_Wh_int"] = 0.0
                        state["error"] = None
                except Exception as e:
                    with lock: state["error"] = f"Soft zero failed: {e}"
                self._redirect("/")
            elif p.path == "/json":
                with lock: payload = json.dumps(state)
                self._ok(payload, "application/json")
            elif p.path == "/debug":
                with lock: msg = state["error"] or "no error"
                self._ok("<pre>"+msg+"</pre>")
            else:
                self.send_error(404)
        except Exception as e:
            self.send_error(500, explain=str(e))

    def _ok(self, body, ctype="text/html; charset=utf-8"):
        body = body if isinstance(body,(bytes,bytearray)) else body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _redirect(self, where):
        self.send_response(302); self.send_header("Location", where); self.end_headers()

def main():
    try:
        srv = HTTPServer((HOST, PORT), Handler)
        print(f"Serving on http://{HOST}:{PORT}  (open http://<pi-ip>:{PORT} on your phone)")
        srv.serve_forever()
    finally:
        if USE_GPIO_LIB:
            GPIO.cleanup()

if __name__ == "__main__":
    main()
