"""
Open-loop motor + servo controller ROS2 node — self-contained evdev keyboard.

Same job as the original mbedMotorServoCtrl node (read keys, forward each raw
byte to the Nucleo, publish feedback telemetry), but the keyboard input now uses
the robust evdev method from the enhanced nodes instead of termios/auto-repeat:

  * grabs ALL keyboard-like input devices (exclusive — no keys leak to the
    terminal), so it also works cleanly over SSH / headless;
  * a background thread tracks the live set of physically-held keys via real
    key-down / key-up events (kernel auto-repeat is ignored on purpose);
  * the main loop re-sends every held recognized key over serial at a fixed rate
    (~20 Hz) to feed the firmware's hold-to-move deadman watchdog, then stops the
    instant the key is released.

The Nucleo firmware still owns ALL interpretation (direction, speed scaling,
servo increments/clamping, watchdog) — see combined_node/src/main.cpp. This node
only decides WHICH key bytes to send.

Setup:  pip install evdev   (and be in the 'input' group, or run with sudo)
Quit:   press ESC  (Ctrl+C also works if started with --no-grab)

Change NUM_MOTORS / NUM_SERVOS to match the #defines in mbed main.cpp — only used
to size/validate the feedback line for telemetry, not for any control logic.
"""

import argparse
import selectors
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from elibot_robot_ctrl.SerialSimple import SerialSimple, getFeedbackVals

try:
    from evdev import InputDevice, ecodes, list_devices
except ImportError:
    raise SystemExit("evdev is not installed. Run:  pip install evdev")

# =============================================================
#  Change these to match the #defines in mbed main.cpp
#  (only used to size/validate the feedback line below)
# =============================================================
NUM_MOTORS = 4
NUM_SERVOS = 3

# Every key the firmware recognizes — anything else is never sent over serial.
# See combined_node/src/main.cpp's header comment for what each one does.
MOTOR_KEYS = set('wsadq')
SPEED_KEYS = set('0123456789')
SERVO_KEYS = set('ujikolp;')
CENTER_KEY = 'c'
RECOGNIZED_KEYS = MOTOR_KEYS | SPEED_KEYS | SERVO_KEYS | {CENTER_KEY}

# evdev reports physical keys by name (e.g. KEY_SEMICOLON -> "semicolon").
# Map the ones the firmware wants as punctuation back to their raw char.
ALIASES = {
    'semicolon': ';',
}

SEND_PERIOD = 0.05   # re-send held keys at ~20 Hz (feeds the firmware watchdog)


# ---------------------------------------------------------------------------
#  Keyboard helpers (evdev)
# ---------------------------------------------------------------------------
def key_to_str(code):
    """Convert an evdev key code (KEY_W) to a friendly string ('w')."""
    name = ecodes.KEY.get(code, f'KEY_{code}')
    if isinstance(name, (list, tuple)):
        name = name[0]
    return name[4:].lower() if name.startswith('KEY_') else name.lower()


def normalize_key(code):
    """evdev code -> the single char the firmware expects (or None if unusable)."""
    s = key_to_str(code)
    return ALIASES.get(s, s)


def looks_like_keyboard(dev):
    keys = dev.capabilities().get(ecodes.EV_KEY, [])
    return ecodes.KEY_A in keys and ecodes.KEY_Z in keys and ecodes.KEY_SPACE in keys


def find_keyboards(preferred=None):
    if preferred:
        return [InputDevice(p) for p in preferred]
    found = []
    for path in list_devices():
        try:
            dev = InputDevice(path)
        except OSError:
            continue
        if looks_like_keyboard(dev):
            found.append(dev)
    return found


def reader_thread(devices, pressed, lock, stop_event):
    """Maintain the set of currently-held RECOGNIZED key chars. ESC -> quit."""
    sel = selectors.DefaultSelector()
    for dev in devices:
        sel.register(dev.fd, selectors.EVENT_READ, dev)
    try:
        while not stop_event.is_set():
            for sk, _ in sel.select(timeout=0.1):
                dev = sk.data
                try:
                    for event in dev.read():
                        if event.type != ecodes.EV_KEY:
                            continue
                        if event.value == 1:            # key down
                            if event.code == ecodes.KEY_ESC:
                                stop_event.set()
                                return
                            ch = normalize_key(event.code)
                            if ch in RECOGNIZED_KEYS:
                                with lock:
                                    pressed.add(ch)
                        elif event.value == 0:          # key up
                            ch = normalize_key(event.code)
                            if ch in RECOGNIZED_KEYS:
                                with lock:
                                    pressed.discard(ch)
                        # value == 2 (kernel auto-repeat) ignored — we do our own
                except OSError:
                    stop_event.set()
                    return
    finally:
        sel.close()


# ---------------------------------------------------------------------------
#  ROS2 node
# ---------------------------------------------------------------------------
class MotorServoController(Node):
    def __init__(self):
        super().__init__('mbedMotorServoCtrl')
        self.duty_publisher  = self.create_publisher(Float32MultiArray, 'motor_duty', 10)
        self.servo_publisher = self.create_publisher(Float32MultiArray, 'servo_angle', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)   # 50 Hz telemetry
        self.duties = [0.0] * NUM_MOTORS
        self.angles = [0.0] * NUM_SERVOS

    def timer_callback(self):
        duty_msg = Float32MultiArray()
        duty_msg.data = self.duties
        self.duty_publisher.publish(duty_msg)

        servo_msg = Float32MultiArray()
        servo_msg.data = self.angles
        self.servo_publisher.publish(servo_msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='Mbed evdev motor + servo controller')
    parser.add_argument('--port',     default='/dev/ttyACM0',
                        help='Serial port of the Nucleo board (default: /dev/ttyACM0)')
    parser.add_argument('--baudrate', default=115200, type=int,
                        help='Serial baud rate (default: 115200)')
    parser.add_argument('--device', action='append',
                        help='/dev/input/eventX (repeatable). Default: all keyboards.')
    parser.add_argument('--no-grab', action='store_true',
                        help='do not take exclusive control (keys also reach terminal)')
    parser.add_argument('--rate', type=float, default=1.0 / SEND_PERIOD,
                        help='serial re-send rate in Hz while a key is held (default 20)')
    parser.add_argument('--debug', action='store_true',
                        help='print the held-key set whenever it changes')
    parsed, ros_args = parser.parse_known_args()

    # --- keyboards ---
    devices = find_keyboards(parsed.device)
    if not devices:
        raise SystemExit("No keyboard found. Try:  python3 -m evdev.evtest")

    # --- serial ---
    ser = SerialSimple(port=parsed.port, baudrate=parsed.baudrate)
    ser.init()
    ser.start()
    if not ser.conn:
        print(f"Failed to open {parsed.port}. Check the port and try again.")
        return

    rclpy.init(args=ros_args)
    node = MotorServoController()

    print(f"Connected on {parsed.port} | {NUM_MOTORS} motor(s), {NUM_SERVOS} servo(s)")
    for dev in devices:
        print(f"Keyboard: {dev.path}  ({dev.name})")
    print("Keys are captured directly and forwarded raw — the Nucleo interprets them.")
    print("Hold a key to move, release to stop. Servos hold their angle on release.")
    print("Motors: w=forward  s=backward  a=left  d=right  q=stop")
    print("Speed:  0-9=select motor speed (0=10% ... 9=100%), default 7 (80%)")
    print("Servos: u/j=servo1±  i/k=servo2±  o/l=servo3±  p/;=servo4±  c=center all")
    print("ESC=quit")

    # --- grab keyboards so keystrokes don't leak to the terminal ---
    grabbed = []
    if not parsed.no_grab:
        for dev in devices:
            try:
                dev.grab()
                grabbed.append(dev)
            except OSError as e:
                print(f"Could not grab {dev.path} ({e}).")

    pressed = set()
    lock = threading.Lock()
    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread,
                         args=(devices, pressed, lock, stop_event), daemon=True)
    t.start()

    period = 1.0 / parsed.rate if parsed.rate > 0 else SEND_PERIOD
    last_send = 0.0
    prev_held = None

    try:
        while ser.conn and not stop_event.is_set():
            # 1) Drain serial feedback so the topics reflect real setpoints.
            #    Feedback line = NUM_MOTORS duties, then NUM_SERVOS angles.
            resp = ser.read()
            if resp:
                vals = getFeedbackVals(resp)
                if len(vals) == NUM_MOTORS + NUM_SERVOS:
                    node.duties = vals[:NUM_MOTORS]
                    node.angles = vals[NUM_MOTORS:]

            # 2) Re-send every held recognized key as a raw byte at ~SEND rate.
            now = time.monotonic()
            if now - last_send >= period:
                with lock:
                    held = list(pressed)
                for ch in held:
                    ser.write(ch)
                last_send = now

                if parsed.debug:
                    snapshot = frozenset(held)
                    if snapshot != prev_held:
                        print(f"held -> {sorted(snapshot) if snapshot else '(none)'}")
                        prev_held = snapshot

            # 3) Service the telemetry timer / callbacks.
            rclpy.spin_once(node, timeout_sec=0)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        stop_event.set()
        for dev in grabbed:
            try:
                dev.ungrab()
            except OSError:
                pass
        if ser.conn:
            ser.write('q')   # stop the motors; servos keep holding their angle
        ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
