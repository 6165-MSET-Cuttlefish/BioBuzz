"""A fake Control Hub for the offline dry runs, at the level the runner talks to one: an adb binary (devices, connect,
forward with a chosen port, forward --list and --remove, shell, push, pull, install, reboot, a device that disappears
and comes back without its forwards), slothboard's websocket and the hub's HTTP ports. Everything listens on this
Mac's loopback at ports the OS picks; nothing binds or connects to 5037 or 5554-5585, and no real adb runs.

    with fake_hub.Hub(usb=True).installed(rb) as hub:
        rb.Runner(rb.parse_args([...])).run()
    hub.calls   # every adb command, as the runner sent it
    hub.tcp     # every TCP connection the runner opened: (host, port, what it reached)
"""
import base64
import contextlib
import datetime
import hashlib
import json
import pathlib
import re
import shlex
import socket
import struct
import subprocess
import threading
import time
import types
import xml.sax.saxutils

HUB_IP = "192.168.43.1"
WIFI_SERIAL = f"{HUB_IP}:5555"
USB_SERIAL = "1a2b3c4d5e6f"
PACKAGE = "com.qualcomm.ftcrobotcontroller"
LAUNCHER = f"{PACKAGE}/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper"
PREFS = f"shared_prefs/{PACKAGE}_preferences.xml"
PREFS_TMP = "/data/local/tmp/bench_rc_prefs.xml"
SETUP_DONE = "======= INIT FINISH ======="
IDLE = "$Stop$Robot$"
EPOCH0 = int(datetime.datetime(2026, 11, 12, 10, 0, 0).timestamp())
HUB_PORTS = (8000, 8080, 8081, 5800, 5801, 5805, 5807)
ADB_PORTS = {5037} | set(range(5554, 5586))
WORKTREE = pathlib.Path(__file__).resolve().parents[3]
STAMP = re.compile(r"^(?:(\d{4})-)?(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d)")
GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def loopback_listener(port=0):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen(16)
    if s.getsockname()[1] in ADB_PORTS:
        s.close()
        return loopback_listener()
    return s


def stamp_seconds(text):
    m = STAMP.match(text.strip("'\""))
    if m is None:
        return None
    year = int(m.group(1)) if m.group(1) else 2026
    return int(datetime.datetime(year, *(int(g) for g in m.groups()[1:])).timestamp())


class Session:
    """One websocket a client opened on the fake slothboard."""

    def __init__(self, server, sock):
        self.server, self.sock = server, sock
        self.lock = threading.Lock()
        self.open = True

    def send(self, obj):
        payload = json.dumps(obj).encode()
        n = len(payload)
        head = bytes([0x81]) + (bytes([n]) if n < 126 else bytes([126]) + struct.pack(">H", n) if n < 65536
                                else bytes([127]) + struct.pack(">Q", n))
        with self.lock:
            try:
                self.sock.sendall(head + payload)
            except OSError:
                self.open = False

    def close(self):
        self.open = False
        with contextlib.suppress(OSError):
            self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()


class Slothboard:
    """slothboard's websocket server on the hub's port 8000, reduced to what the runner uses."""

    def __init__(self, hub):
        self.hub = hub
        self.listener = loopback_listener()
        self.port = self.listener.getsockname()[1]
        self.sessions = []
        self.lock = threading.Lock()
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                sock, _ = self.listener.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(sock,), daemon=True).start()

    def _read(self, sock, n, buf):
        while len(buf) < n:
            chunk = sock.recv(65536)
            if not chunk:
                raise ConnectionError
            buf += chunk
        return buf[:n], buf[n:]

    def _serve(self, sock):
        request = b""
        while b"\r\n\r\n" not in request:
            chunk = sock.recv(4096)
            if not chunk:
                sock.close()
                return
            request += chunk
        head, buf = request.split(b"\r\n\r\n", 1)
        key = next((line.split(":", 1)[1].strip() for line in head.decode("latin-1").split("\r\n")
                    if line.lower().startswith("sec-websocket-key:")), "")
        accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
        sock.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                      f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
        session = Session(self, sock)
        with self.lock:
            self.sessions.append(session)
        self.hub.on_connect(session)
        try:
            while session.open:
                (b0, b1), buf = self._read(sock, 2, buf)
                n = b1 & 0x7F
                if n == 126:
                    raw, buf = self._read(sock, 2, buf)
                    n = struct.unpack(">H", raw)[0]
                elif n == 127:
                    raw, buf = self._read(sock, 8, buf)
                    n = struct.unpack(">Q", raw)[0]
                mask, buf = self._read(sock, 4, buf) if b1 & 0x80 else (b"\0\0\0\0", buf)
                data, buf = self._read(sock, n, buf)
                data = bytes(c ^ mask[i % 4] for i, c in enumerate(data))
                opcode = b0 & 0x0F
                if opcode == 0x8:
                    break
                if opcode == 0x9:
                    with session.lock:
                        sock.sendall(bytes([0x8A, len(data)]) + data)
                elif opcode == 0x1:
                    self.hub.on_message(session, json.loads(data.decode()))
        except (ConnectionError, OSError, ValueError):
            pass
        finally:
            session.close()
            with self.lock:
                if session in self.sessions:
                    self.sessions.remove(session)

    def broadcast(self, obj):
        with self.lock:
            sessions = list(self.sessions)
        for s in sessions:
            s.send(obj)

    def drop_all(self):
        with self.lock:
            sessions, self.sessions = list(self.sessions), []
        for s in sessions:
            s.close()


class HttpPorts:
    """The hub's other HTTP ports (the RC page and the Limelight proxy): 200 and a small JSON body for any path."""

    def __init__(self, hub, remote):
        self.hub, self.remote = hub, remote
        self.listener = loopback_listener()
        self.port = self.listener.getsockname()[1]
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                sock, _ = self.listener.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(sock,), daemon=True).start()

    def _serve(self, sock):
        with sock:
            request = b""
            while b"\r\n\r\n" not in request:
                chunk = sock.recv(4096)
                if not chunk:
                    return
                request += chunk
            line = request.split(b"\r\n", 1)[0].decode("latin-1")
            self.hub.http.append((self.remote, line))
            switch = re.match(r"POST /pipeline-switch\?index=(\d+) ", line)
            if switch:
                self.hub.pipeline = int(switch.group(1))
            body = json.dumps({"pipelineIndex": self.hub.pipeline}).encode()
            sock.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: "
                         + str(len(body)).encode() + b"\r\nConnection: close\r\n\r\n" + body)


class Forward:
    """One adb forward: a listener on this Mac that pipes each connection to the hub's port, like the adb server's."""

    def __init__(self, hub, serial, local, remote, virtual=False):
        self.hub, self.serial, self.remote, self.virtual = hub, serial, remote, virtual
        self.conns = []
        self.listener = None if virtual else loopback_listener(local)
        self.local = local if virtual else self.listener.getsockname()[1]
        if not virtual:
            threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                sock, _ = self.listener.accept()
            except OSError:
                return
            backend = self.hub.backend_port(self.remote)
            if backend is None:
                sock.close()
                continue
            upstream = socket.socket()
            try:
                upstream.connect(("127.0.0.1", backend))
            except OSError:
                sock.close()
                continue
            self.conns += [sock, upstream]
            for a, b in ((sock, upstream), (upstream, sock)):
                threading.Thread(target=self._pipe, args=(a, b), daemon=True).start()

    @staticmethod
    def _pipe(a, b):
        try:
            while True:
                data = a.recv(65536)
                if not data:
                    break
                b.sendall(data)
        except OSError:
            pass
        finally:
            for s in (a, b):
                with contextlib.suppress(OSError):
                    s.shutdown(socket.SHUT_RDWR)

    def close(self):
        if self.listener is not None:
            self.listener.close()
        for s in self.conns:
            with contextlib.suppress(OSError):
                s.shutdown(socket.SHUT_RDWR)
            s.close()


class FakeLines:
    def __init__(self, lines):
        self.lines = lines

    def __iter__(self):
        return iter(self.lines)


class FakePopen:
    def __init__(self, argv, lines, returncode=0):
        self.args, self.returncode = argv, returncode
        self.stdout = FakeLines(lines)
        self.waited = self.killed = False

    def wait(self, timeout=None):
        self.waited = True
        return self.returncode

    def poll(self):
        return self.returncode if self.waited or self.killed else None

    def kill(self):
        self.killed = True

    def terminate(self):
        pass


class FastTime(types.ModuleType):
    """The runner's view of time, running `speed` times faster: its sleeps and timeouts shrink, nothing else does."""

    def __init__(self, speed):
        super().__init__("time")
        self.speed = speed
        self.t0 = time.monotonic()

    def monotonic(self):
        return self.t0 + (time.monotonic() - self.t0) * self.speed

    def sleep(self, seconds):
        time.sleep(max(0.0, seconds) / self.speed)

    def __getattr__(self, name):
        return getattr(time, name)


class Hub:
    """One Control Hub behind a fake adb server. usb=True: its USB serial is in `adb devices` from the start;
    usb=False: it appears once `adb connect 192.168.43.1:5555` runs."""

    def __init__(self, usb=False, extra_usb=(), config="betta_decode", busy_local_ports=(), foreign_forwards=(),
                 speed=20.0, everything=False, control_hub=True, socket_table=True, http_answers=True):
        self.usb = usb
        self.control_hub, self.socket_table, self.http_answers = control_hub, socket_table, http_answers
        self.serial = USB_SERIAL if usb else WIFI_SERIAL
        self.present = usb
        self.extra_usb = list(extra_usb)
        self.busy_local_ports = set(busy_local_ports)
        self.speed = speed
        self.lock = threading.RLock()
        self.calls, self.tcp, self.http, self.unknown, self.deploys = [], [], [], [], []
        self.forwards = [Forward(self, s, local, remote, virtual=True) for s, local, remote in foreign_forwards]
        self.foreign = list(self.forwards)
        self.made_forwards = []
        self.down_polls = 0
        self.reboots = 0
        self.app_up = True
        self.pid = 4000
        self.clock = EPOCH0
        self.boot_clock = EPOCH0 - 600
        self.logcat = []
        self.pipeline = 4
        self.status = {"enabled": True, "available": True, "activeOpMode": IDLE, "activeOpModeStatus": "STOPPED",
                       "errorMessage": "", "warningMessage": "", "batteryVoltage": 12.6}
        self.configs = {config: (WORKTREE / f"TeamCode/src/main/res/xml/{config}.xml").read_text()}
        self.active = {"name": config, "location": "LOCAL_STORAGE", "resourceId": 0, "isDirty": False}
        self.staged_prefs = None
        self.bench = {}
        self.probe = {
            "hubs": [{"parent": True, "imuType": "BHI260", "firmware": "1.8.2", "address": 173, "responding": True}],
            "webcams": [], "limelights": [], "interfaces": ["wlan0", "lo"],
            "shippedConfigs": {"betta_decode": 2131820545, "cuttle_decode": 2131820546}}
        if everything:
            self.probe["hubs"].append({"parent": False, "address": 2, "firmware": "1.8.2", "responding": True})
            self.probe["webcams"] = [{"attached": True, "serial": "UC762"}]
            self.probe["limelights"] = [{"status": "ok", "ipAddress": "172.29.0.1",
                                         "serial": "EthernetOverUsb:eth0:172.29.0.22"}]
        self.start_telemetry = {"Bench: Limelight": {"Bench phase": "proxyQuery", "Bench status": "ready"}}
        self.on_present = None
        self.hub_serial = "FAKEHUB0001"
        self.usb_rev_hubs = []
        self.lsof = "p4242\ncadb\n"
        self.dashboard_off = False
        self.dashboard_unreadable = False
        self.app_broken = False
        self.forward_list_fails = False
        self.socket_table_failures = 0
        self.hang_no_rebind = False
        self.gradle_procs = []
        self.dash = Slothboard(self)
        self.http_ports = {p: HttpPorts(self, p) for p in HUB_PORTS if p != 8000}
        self.log_line(SETUP_DONE, tag="FtcEventLoop")

    def backend_port(self, remote):
        if not (self.present and self.app_up):
            return None
        if remote == 8000:
            return self.dash.port
        return self.http_ports[remote].port if remote in self.http_ports and self.http_answers else None

    def log_line(self, text, tag="BENCH", level="I"):
        t = datetime.datetime.fromtimestamp(self.clock)
        self.logcat.append((self.clock, f"{t:%Y-%m-%d %H:%M:%S}.000  {self.pid}  {self.pid} {level} {tag}: {text}"))

    def prefs_xml(self, entry=None):
        value = xml.sax.saxutils.escape(json.dumps(entry or self.active))
        return ("<?xml version='1.0' encoding='utf-8' standalone='yes' ?>\n<map>\n"
                f'    <string name="pref_hardware_config_filename">{value}</string>\n</map>\n')

    def opmodes(self):
        return [{"name": n, "group": "Test"} for n in self.opmode_names]

    def on_connect(self, session):
        session.send({"type": "RECEIVE_OP_MODE_LIST", "opModeInfoList": self.opmodes()})
        session.send({"type": "RECEIVE_HARDWARE_CONFIG_LIST", "currentHardwareConfig": self.active["name"],
                      "hardwareConfigList": [{"name": n} for n in sorted(self.configs)]})
        session.send({"type": "RECEIVE_CONFIG", "configRoot": {"__type": "custom", "__value": {
            "Optimization Toggles": {"__type": "custom", "__value": {}}}}})
        session.send({"type": "RECEIVE_ROBOT_STATUS", "status": dict(self.status)})

    def set_active(self, name, state):
        self.status.update(activeOpMode=name, activeOpModeStatus=state)
        self.dash.broadcast({"type": "RECEIVE_ROBOT_STATUS", "status": dict(self.status)})

    def on_message(self, session, msg):
        kind = msg.get("type")
        with self.lock:
            if kind == "GET_ROBOT_STATUS":
                session.send({"type": "RECEIVE_ROBOT_STATUS", "status": dict(self.status)})
            elif kind == "INIT_OP_MODE":
                self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": []})
                self.set_active(msg["opModeName"], "INIT")
                self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": [
                    {"data": {"BENCH DONE": "dry run"}, "timestamp": self.clock, "fieldOverlay": {"ops": []}}]})
            elif kind == "START_OP_MODE" and self.status["activeOpMode"] != IDLE:
                self.set_active(self.status["activeOpMode"], "RUNNING")
                data = self.start_telemetry.get(self.status["activeOpMode"])
                if data:
                    self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": [
                        {"data": data, "timestamp": self.clock, "fieldOverlay": {"ops": []}}]})
            elif kind == "STOP_OP_MODE":
                self.set_active(IDLE, "STOPPED")
            elif kind == "GET_CONFIG_BASELINE":
                session.send({"type": "RECEIVE_CONFIG_BASELINE", "configBaseline": {"__type": "custom", "__value": {
                    "Optimization Toggles": {"__type": "custom", "__value": {}}}}})
            elif kind == "GET_CONFIG":
                session.send({"type": "RECEIVE_CONFIG", "configRoot": {"__type": "custom", "__value": {}}})
            elif kind in ("SET_HARDWARE_CONFIG", "WRITE_HARDWARE_CONFIG", "DELETE_HARDWARE_CONFIG"):
                self.restart_app(same_pid=True)

    def restart_app(self, same_pid=False):
        self.dash.drop_all()
        if not same_pid:
            self.pid += 1
        self.app_up = not self.app_broken
        self.status.update(activeOpMode=IDLE, activeOpModeStatus="STOPPED")
        self.log_line(SETUP_DONE, tag="FtcEventLoop")

    def disappear(self, polls):
        """The device drops off adb (a reboot or a pulled cable) for `polls` `adb devices` calls, taking its forwards."""
        with self.lock:
            self.present = False
            self.app_up = False
            self.down_polls = polls
            for f in [f for f in self.forwards if f.serial == self.serial]:
                f.close()
                self.forwards.remove(f)
            self.dash.drop_all()

    def come_back(self):
        self.present = True
        self.reboots += 1
        self.boot_clock = self.clock
        self.pid += 1
        self.restart_app(same_pid=True)
        if self.on_present is not None:
            self.on_present()

    def device_rows(self):
        rows = [(self.serial, "device")] if self.present else []
        return rows + [(s, "device") for s in self.extra_usb]

    def adb(self, argv, env=None):
        with self.lock:
            args = list(argv[1:])
            self.calls.append(" ".join(["adb"] + args))
            self.clock += 1
            serial = None
            if args[:1] == ["-s"]:
                serial, args = args[1], args[2:]
            if self.hang_no_rebind and "--no-rebind" in args:
                raise subprocess.TimeoutExpired(argv, 20)
            out, err, code = self._command(serial, args)
            return subprocess.CompletedProcess(argv, code, out, err)

    def _missing(self, serial):
        return f"adb: device '{serial}' not found\n"

    def _command(self, serial, args):
        cmd = args[0] if args else ""
        if cmd == "devices":
            if not self.present and self.down_polls > 0:
                self.down_polls -= 1
                if self.down_polls == 0:
                    self.come_back()
            return "List of devices attached\n" + "".join(f"{s}\t{st}\n" for s, st in self.device_rows()) + "\n", "", 0
        if cmd == "version":
            return "Android Debug Bridge version 1.0.41\nVersion 37.0.1-fake\n", "", 0
        if cmd == "connect":
            if self.usb or args[1] != WIFI_SERIAL or self.down_polls > 0:
                return f"failed to connect to '{args[1]}'\n", "", 1
            self.present = True
            return f"connected to {WIFI_SERIAL}\n", "", 0
        if cmd == "forward" and args[1:] == ["--list"]:
            if self.forward_list_fails:
                return "", "adb: error: cannot connect to daemon\n", 1
            return "".join(f"{f.serial} tcp:{f.local} tcp:{f.remote}\n" for f in self.forwards), "", 0
        if serial is None and cmd not in ("forward",):
            return "", "adb: no serial given in the dry run\n", 1
        if serial not in [s for s, _ in self.device_rows()]:
            return "", self._missing(serial), 1
        if cmd == "forward":
            return self._forward(serial, args[1:])
        if cmd == "shell":
            return self._shell(args[1] if len(args) == 2 else " ".join(args[1:]))
        if cmd == "push":
            return self._push(args[1], args[2])
        if cmd == "pull":
            return self._pull(args[1], args[2])
        if cmd in ("install", "install-multiple"):
            self.restart_app()
            return "Performing Streamed Install\nSuccess\n", "", 0
        if cmd == "reboot":
            self.disappear(3)
            return "", "", 0
        if cmd == "get-state":
            return "device\n", "", 0
        self.unknown.append(args)
        return "", "", 0

    def _forward(self, serial, args):
        if args[:1] == ["--remove"]:
            f = next((f for f in self.forwards if f"tcp:{f.local}" == args[1]), None)
            if f is None:
                return "", f"adb: error: listener '{args[1]}' not found\n", 1
            f.close()
            self.forwards.remove(f)
            return "", "", 0
        if args[:1] == ["--remove-all"]:
            for f in self.forwards:
                f.close()
            self.forwards = []
            return "", "", 0
        rebind = True
        if args[:1] == ["--no-rebind"]:
            rebind, args = False, args[1:]
        local, remote = args
        port = int(remote[len("tcp:"):])
        if local == "tcp:0":
            f = Forward(self, serial, 0, port)
            self.forwards.append(f)
            self.made_forwards.append(f)
            return f"{f.local}\n", "", 0
        wanted = int(local[len("tcp:"):])
        existing = next((f for f in self.forwards if f.local == wanted), None)
        if existing is not None and not rebind:
            return "", "adb: error: cannot rebind existing socket\n", 1
        if wanted in self.busy_local_ports:
            return "", f"adb: error: cannot bind listener: cannot bind to 127.0.0.1:{wanted}: Address already in use\n", 1
        if existing is not None:
            self.forwards.remove(existing)
        f = Forward(self, serial, wanted, port, virtual=True)
        self.forwards.append(f)
        self.made_forwards.append(f)
        return "", "", 0

    def _push(self, local, remote):
        text = pathlib.Path(local).read_text()
        if remote == PREFS_TMP:
            self.staged_prefs = text
        elif remote.startswith("/sdcard/FIRST/") and remote.endswith(".xml"):
            self.configs[remote[len("/sdcard/FIRST/"):-4]] = text
        return f"{local}: 1 file pushed\n", "", 0

    def _pull(self, remote, local):
        path = pathlib.Path(local)
        if remote.endswith("/bench"):
            path.mkdir(parents=True, exist_ok=True)
            return "0 files pulled\n", "", 0
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"apk")
        return "1 file pulled\n", "", 0

    def _bench_read(self, path):
        name = path.rsplit("/", 1)[-1]
        if path in self.bench:
            return self.bench[path]
        if name == "probe.json":
            return json.dumps(self.probe)
        if name.startswith("thread_lifecycle_"):
            return json.dumps({"afterCrash": {"done": True}})
        if name.endswith(".json") and name != "params.json":
            return "{}"
        return None

    def _restart_from_prefs(self):
        import xml.etree.ElementTree as ElementTree
        root = ElementTree.fromstring(self.staged_prefs)
        for e in root.iter("string"):
            if e.get("name") == "pref_hardware_config_filename":
                self.active = json.loads(e.text)
        self.restart_app()

    def _shell(self, command):
        hub_date = datetime.datetime.fromtimestamp(self.clock)
        if command.startswith("log -t BENCH_RUNNER "):
            self.log_line(shlex.split(command)[3], tag="BENCH_RUNNER")
            return "", "", 0
        if command.startswith("logcat -d -v threadtime -T "):
            head, _, grep = command.partition(" | grep -F ")
            since = stamp_seconds(shlex.split(head)[5])
            lines = [line for t, line in self.logcat if since is None or t >= since]
            if grep:
                words = shlex.split(grep)
                needles = [words[i + 1] for i, w in enumerate(words) if w == "-e"]
                lines = [line for line in lines if any(n in line for n in needles)]
            return "".join(line + "\n" for line in lines), "", 0
        if command.startswith("date '+"):
            return hub_date.strftime(command[len("date '+"):-1]) + "\n", "", 0
        if command == "date +%s":
            return f"{self.clock}\n", "", 0
        if command == "date +%s; cat /proc/uptime":
            return f"{self.clock}\n{self.clock - self.boot_clock}.00 100.00\n", "", 0
        if command.startswith(f"pidof {PACKAGE}"):
            return (f"{self.pid}\n" if self.app_up else ""), "", 0 if self.app_up else 1
        if command == "cat /proc/net/tcp /proc/net/tcp6":
            if self.socket_table_failures > 0:
                self.socket_table_failures -= 1
                return "", "error: closed\n", 1
            if not self.socket_table:
                return "", "cat: /proc/net/tcp: Permission denied\n", 1
            rows = ["  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode"]
            if self.app_up:
                rows += [f"   {i}: 00000000:{p:04X} 00000000:0000 0A 00000000:00000000 00:00000000 00000000 10070 0 1"
                         for i, p in enumerate(HUB_PORTS)]
            return "\n".join(rows) + "\n", "", 0
        if command == f"run-as {PACKAGE} cat {PREFS}":
            return self.prefs_xml(), "", 0
        if command.startswith(f"run-as {PACKAGE} cat shared_prefs/FtcDashboard.xml"):
            if self.dashboard_unreadable:
                return "", f"run-as: package not debuggable: {PACKAGE}\n", 1
            if self.dashboard_off:
                return ("<?xml version='1.0' encoding='utf-8' standalone='yes' ?>\n<map>\n"
                        '    <boolean name="autoEnable" value="false" />\n</map>\n'), "", 0
            return "", "cat: shared_prefs/FtcDashboard.xml: No such file or directory\n", 1
        if command.startswith(f"run-as {PACKAGE} cat "):
            text = self._bench_read(command.split()[-1])
            if text is None:
                return "", f"cat: {command.split()[-1]}: No such file or directory\n", 1
            return text, "", 0
        if command.startswith(f"run-as {PACKAGE} sh -c "):
            return self._bench_shell(shlex.split(command)[4])
        if command == "ls -1 /sdcard/FIRST":
            return "".join(f"{n}.xml\n" for n in sorted(self.configs)) + "bench\n", "", 0
        if command == "ls -l /sdcard/FIRST/":
            return "".join(f"-rw-rw---- 1 root sdcard_rw {len(x)} 2026-11-01 12:00 {n}.xml\n"
                           for n, x in sorted(self.configs.items())), "", 0
        if command.startswith("cat '/sdcard/FIRST/") or command.startswith("cat /sdcard/FIRST/"):
            name = shlex.split(command)[1][len("/sdcard/FIRST/"):-4]
            if name not in self.configs:
                return "", "No such file or directory\n", 1
            return self.configs[name], "", 0
        if command.startswith("rm '/sdcard/FIRST/") or command.startswith("rm /sdcard/FIRST/"):
            self.configs.pop(shlex.split(command)[1][len("/sdcard/FIRST/"):-4], None)
            return "", "", 0
        if command.startswith(f"am force-stop {PACKAGE} && run-as {PACKAGE} cp {PREFS_TMP} {PREFS}"):
            self._restart_from_prefs()
            return "BENCH_PREFS_COPIED\nStarting: Intent\n", "", 0
        if command.startswith(f"am force-stop {PACKAGE} && run-as {PACKAGE} rm -f shared_prefs/FtcDashboard.xml"):
            self.dashboard_off = False
            self.restart_app()
            return "Starting: Intent\n", "", 0
        if command.startswith(f"am force-stop {PACKAGE}"):
            self.restart_app()
            return "Starting: Intent\n", "", 0
        if command.startswith(f"am start -n {LAUNCHER}"):
            if not self.app_up:
                self.restart_app()
            return "Starting: Intent\n", "", 0
        if command == "getprop persist.ftcandroid.serialasusb":
            return ("1\n" if self.control_hub else "\n"), "", 0
        if command == "getprop ro.serialno":
            return f"{self.hub_serial}\n", "", 0
        if command.startswith("for d in /sys/bus/usb/devices/*;"):
            return "".join(f"{s}\n" for s in self.usb_rev_hubs), "", 0
        if command == f"pm path {PACKAGE}":
            return f"package:/data/app/{PACKAGE}-1/base.apk\n", "", 0
        if command.startswith("for f in /sdcard/FIRST/dairy/sloth/*.jar"):
            return f"now {self.clock}\n", "", 0
        return "", "", 0

    def _bench_shell(self, inner):
        words = shlex.split(inner)
        if words[:1] == ["printf"]:
            text, path = words[2], words[4][:-len(".tmp")]
            self.bench[path] = text
            return "", "", 0
        if words[:1] == ["cat"]:
            text = self._bench_read(words[1])
            return (text, "", 0) if text is not None else ("", f"cat: {words[1]}: No such file or directory\n", 1)
        if words[:2] == ["rm", "-f"]:
            for p in words[2:]:
                self.bench.pop(p, None)
            return "", "", 0
        if words[:1] == ["ls"]:
            return "".join(p.rsplit("/", 1)[-1] + "\n" for p in self.bench), "", 0
        return "", "", 0

    def popen(self, argv, env=None, **kw):
        with self.lock:
            name = pathlib.Path(argv[0]).name
            if name == "gradlew":
                listed = [s for s, _ in self.device_rows()]
                self.deploys.append({"argv": list(argv[1:]), "ANDROID_SERIAL": (env or {}).get("ANDROID_SERIAL"),
                                     "listedDevices": listed})
                self.log_line("Processed Sloth Load", tag="Sloth")
                proc = FakePopen(argv, ["checked for lock file\n", "pushed jar\n", "waiting for lock file to be removed\n",
                                        "lock file removed\n", "BUILD SUCCESSFUL\n"])
                self.gradle_procs.append(proc)
                return proc
            self.calls.append(" ".join(["adb"] + list(argv[1:])))
            return FakePopen(argv, [])

    def run(self, argv, **kw):
        if pathlib.Path(argv[0]).name == "lsof":
            return subprocess.CompletedProcess(argv, 0, self.lsof, "")
        return self.adb(argv, kw.get("env"))

    def create_connection(self, address, timeout=None, source_address=None, **kw):
        host, port = address[0], address[1]
        with self.lock:
            forward = next((f for f in self.forwards if not f.virtual and f.local == port), None)
            if host == HUB_IP and not self.usb:
                backend = self.backend_port(port)
                self.tcp.append((host, port, "hub" if backend else "refused"))
            elif host in ("127.0.0.1", "localhost") and forward is not None and forward.serial == self.serial:
                backend = port
                self.tcp.append((host, port, f"forward to {forward.remote}"))
            else:
                backend = None
                self.tcp.append((host, port, "not the hub"))
        if backend is None:
            raise ConnectionRefusedError(f"the dry run refuses {host}:{port}")
        s = socket.socket()
        s.settimeout(timeout)
        s.connect(("127.0.0.1", backend))
        return s

    def no_tcp_listener(self, port):
        return False

    @contextlib.contextmanager
    def installed(self, rb):
        """Points run_bench's adb, lsof, Gradle, clock and sockets at this hub until the block ends."""
        fake = types.SimpleNamespace(**{k: getattr(subprocess, k) for k in dir(subprocess) if not k.startswith("__")})
        fake.run, fake.Popen = self.run, self.popen
        self.opmode_names = list(rb.BENCH_OPMODES) + [rb.HARDWARE_OPMODE, "Bench: Dashboard Off"]
        saved = rb.subprocess, rb.find_adb, rb.port_answers, rb.time, socket.create_connection
        rb.subprocess, rb.find_adb, rb.port_answers = fake, (lambda: "/fake/platform-tools/adb"), self.no_tcp_listener
        rb.time = FastTime(self.speed)
        socket.create_connection = self.create_connection
        try:
            yield self
        finally:
            rb.subprocess, rb.find_adb, rb.port_answers, rb.time, socket.create_connection = saved
            self.close()

    def close(self):
        for f in self.forwards:
            f.close()
        self.dash.drop_all()
        self.dash.listener.close()
        for h in self.http_ports.values():
            h.listener.close()
