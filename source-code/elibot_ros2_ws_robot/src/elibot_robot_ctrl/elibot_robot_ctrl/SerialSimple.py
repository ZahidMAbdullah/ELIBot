import serial

EOP = 'd'

# ---------------------------------------------------------------------------
# Parsing: mbed sends comma-separated setpoints on one line, motor duties
# first, then servo angles, e.g. with 4 motors + 4 servos:
#   "0.60,-0.60,0.00,0.00,90.0,-45.0,0.0,135.0\n"
# ---------------------------------------------------------------------------

def getFeedbackVals(raw_data: str) -> list:
    """Return list of floats parsed from an mbed feedback line."""
    try:
        return [float(x) for x in raw_data.strip().split(',')]
    except Exception:
        return []

# Backward-compatible alias (old name, same CSV parsing)
getMotorDuties = getFeedbackVals

# ---------------------------------------------------------------------------
# Sending: Python sends slash-separated values terminated with /d —
# motor duties first, then servo angles:
#   4 motors + 4 servos: "0.600/-0.600/0.000/0.000/90.0/-45.0/0.0/135.0/d"
# The mbed firmware always expects NUM_MOTORS + NUM_SERVOS values.
# ---------------------------------------------------------------------------

def setCommand(ser, duties: list, angles: list, servo_range: float = 135.0):
    """Send one combined command: N motor duties + M servo angles.

    Duties are clamped to [-1.0, 1.0], angles to [-servo_range, +servo_range].
    """
    d = [max(-1.0, min(1.0, x)) for x in duties]
    a = [max(-servo_range, min(servo_range, x)) for x in angles]
    payload = ('/'.join(f'{x:.3f}' for x in d) + '/'
               + '/'.join(f'{x:.1f}' for x in a) + '/d')
    ser.write(payload)

def setMotorDuties(ser, duties: list):
    """Motors-only send (legacy firmware without servos). Kept for the old
    mbed_node firmware; do NOT use with the combined firmware — it expects
    servo angles too."""
    clamped = [max(-1.0, min(1.0, d)) for d in duties]
    payload = '/'.join(f'{d:.3f}' for d in clamped) + '/d'
    ser.write(payload)

# ---------------------------------------------------------------------------
# Serial port wrapper
# ---------------------------------------------------------------------------

class SerialSimple:
    def __init__(self, port, baudrate=9600, timeout=0.1):
        self.baudrate = baudrate
        self.port     = port
        self.parity   = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.bytesize = serial.EIGHTBITS
        self.timeout  = timeout
        self.ser  = None
        self.conn = False

    def init(self):
        self.ser = serial.Serial(
            port     = self.port,
            baudrate = self.baudrate,
            parity   = self.parity,
            stopbits = self.stopbits,
            bytesize = self.bytesize,
            timeout  = self.timeout
        )

    def start(self):
        if self.ser is not None:
            self.conn = self.ser.is_open
            if not self.conn:
                print(f"Cannot connect to {self.port}")
        else:
            print("Serial port not initialized — call init() first")
        return self.conn

    def read(self) -> str:
        try:
            return self.ser.read_until(expected=b'\n').decode('utf-8')
        except serial.SerialException as e:
            print(f"Serial read error: {e}")
            self.conn = False
            return ''

    def write(self, payload: str) -> int:
        try:
            self.ser.write(payload.encode())
            self.ser.flush()
            return 1
        except serial.SerialException as e:
            print(f"Serial write error: {e}")
            self.conn = False
            return -1

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.conn = False
