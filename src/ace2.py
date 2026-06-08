# https://github.com/DnG-Crafts/U1-Ace
import serial, threading, time, logging, struct, queue, traceback, glob, copy, random, configparser
from datetime import datetime
from . import filament_protocol


_PREAMBLE       = b'\xff\xaa'
_END_MARKER     = 0xFE
_FLAG_REQUEST   = 0x00
_MAX_PAYLOAD    = 100

_CMD_DISCOVER_DEVICE       = 0
_CMD_ASSIGN_DEVICE_ID      = 1
_CMD_GET_STATUS            = 6
_CMD_FEED_OR_ROLLBACK      = 8
_CMD_STOP_FEED_OR_ROLLBACK = 9
_CMD_DRYING                = 11
_CMD_GET_FILAMENT_INFO     = 13
_CMD_SET_FEED_CHECK        = 19
_CMD_GET_TEMP              = 64

_FEED_MODE_FEED          = 0
_FEED_MODE_ROLLBACK      = 1
_FEED_MODE_FEED_ASSIST   = 2
_FEED_MODE_UNWIND_ASSIST = 3
_FEED_ASSIST_SPEED       = 10
_UNWIND_ASSIST_SPEED     = 0

_SLOT_READY              = 0
_SLOT_FEEDING            = 1
_SLOT_ROLLBACK           = 2
_SLOT_ASSISTING          = 3
_SLOT_ROLLBACK_ASSISTING = 4
_SLOT_PRELOADING         = 5
_SLOT_FEED_ERROR         = 129
_FILAMENT_EMPTY          = 0
_FILAMENT_IDENTIFIED     = 2

_VELOCITY_IDLE_THRESHOLD = 0.001
_FF_ACTIVE_PREFIXES = ('load_', 'unload_', 'preload_', 'manual_sta_')

_DRY_STATE_NAMES = {
    0: 'free', 1: 'starting', 2: 'keeping', 3: 'stopping',
    4: 'ptc_error', 5: 'ntc_error',
}
_SLOT_ERROR_BY_RAW = {
    129: 'feed_error', 130: 'rollback_error', 131: 'assist_error',
    132: 'preload_error', 133: 'stuck', 134: 'tangled', 135: 'motor_error',
}

ACE_ERROR_STATUSES = {
    'feed_error', 'rollback_error', 'preload_error',
    'stuck', 'tangled', 'motor_error', 'error',
}
_STATES_INCOMPATIBLE_WITH_ASSIST = {
    'ready', 'feeding', 'unwinding', 'preload', 'empty',
}


def _crc16_kermit(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF

def _pb_varint(value):
    r = bytearray()
    while value > 0x7F:
        r.append((value & 0x7F) | 0x80)
        value >>= 7
    r.append(value & 0x7F)
    return bytes(r)

def _pb_uint32(field, value):
    return _pb_varint((field << 3) | 0) + _pb_varint(int(value) & 0xFFFFFFFF)

def _pb_bool(field, value):
    return _pb_varint((field << 3) | 0) + _pb_varint(1 if value else 0)

def _pb_decode_varint(data, pos):
    result, shift = 0, 0
    while pos < len(data):
        b = data[pos]; pos += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, pos
        shift += 7
    return result, pos

def _pb_decode(data):
    fields = {}
    pos = 0; n = len(data)
    while pos < n:
        tag, pos = _pb_decode_varint(data, pos)
        fnum = tag >> 3; wtype = tag & 7
        if wtype == 0:   val, pos = _pb_decode_varint(data, pos)
        elif wtype == 1:
            if pos + 8 > n: break
            val = struct.unpack_from('<d', data, pos)[0]; pos += 8
        elif wtype == 2:
            ln, pos = _pb_decode_varint(data, pos)
            if pos + ln > n: break
            val = bytes(data[pos:pos + ln]); pos += ln
        elif wtype == 5:
            if pos + 4 > n: break
            val = struct.unpack_from('<f', data, pos)[0]; pos += 4
        else: break
        fields.setdefault(fnum, []).append((wtype, val))
    return fields

def _pb_first(fields, num, default=0):
    lst = fields.get(num)
    return lst[0][1] if lst else default

def _pb_str(fields, num, default=''):
    val = _pb_first(fields, num, default)
    if isinstance(val, (bytes, bytearray)):
        try: return val.decode('utf-8', errors='ignore')
        except: return default
    return val

def _build_packet(cmd, payload=b'', seq=1, flags=_FLAG_REQUEST):
    plen = min(len(payload), _MAX_PAYLOAD)
    inner = bytearray([flags & 0xFF, seq & 0xFF, (seq >> 8) & 0xFF,
                       cmd & 0xFF, plen & 0xFF])
    inner.extend(payload[:plen])
    crc = _crc16_kermit(bytes(inner))
    return _PREAMBLE + bytes(inner) + bytes([crc & 0xFF, (crc >> 8) & 0xFF, _END_MARKER])

def _parse_stream(buf):
    packets = []
    while True:
        if len(buf) < 10: break
        idx = buf.find(_PREAMBLE)
        if idx < 0: del buf[:-1]; break
        if idx > 0: del buf[:idx]; continue
        plen = buf[6]
        if plen > _MAX_PAYLOAD: del buf[:2]; continue
        total = 2 + 5 + plen + 2 + 1
        if len(buf) < total: break
        if buf[total - 1] != _END_MARKER: del buf[:2]; continue
        inner = bytes(buf[2:7 + plen])
        crc_r = buf[7 + plen] | (buf[8 + plen] << 8)
        if crc_r != _crc16_kermit(inner): del buf[:2]; continue
        packets.append({
            'cmd':     buf[5],
            'seq':     buf[3] | (buf[4] << 8),
            'flags':   buf[2],
            'is_resp': bool(buf[2] & 0x80),
            'payload': bytes(buf[7:7 + plen]),
        })
        del buf[:total]
    return packets, buf

def _slot_status_to_v1(slot_state, fil_state):
    if fil_state == _FILAMENT_EMPTY:                               return 'empty'
    if slot_state == _SLOT_FEEDING:                                return 'feeding'
    if slot_state == _SLOT_PRELOADING:                             return 'preload'
    if slot_state == _SLOT_ROLLBACK:                               return 'unwinding'
    if slot_state in (_SLOT_ASSISTING, _SLOT_ROLLBACK_ASSISTING):  return 'assisting'
    err = _SLOT_ERROR_BY_RAW.get(slot_state)
    if err:                                                        return err
    if slot_state >= _SLOT_FEED_ERROR:                             return 'error'
    return 'ready'

def _decode_status(payload):
    f = _pb_decode(payload)
    slots = []
    for _, sd in f.get(9, []):
        sf = _pb_decode(sd)
        ss = _pb_first(sf, 1, 0); fs = _pb_first(sf, 2, 0)
        slots.append({
            'status': _slot_status_to_v1(ss, fs),
            'rfid': 2 if fs == _FILAMENT_IDENTIFIED else (1 if fs != _FILAMENT_EMPTY else 0),
        })
    result = {
        'status': 'ready', 'slots': slots,
        'temp': _pb_first(f, 3, 0), 'humidity': _pb_first(f, 4, 0),
        'feed_assist_count': _pb_first(f, 7, 0),
    }
    if 2 in f:
        d = _pb_decode(f[2][0][1])
        result['dryer_status'] = {
            'status':      _DRY_STATE_NAMES.get(_pb_first(d, 1, 0), 'free'),
            'target_temp': _pb_first(d, 2, 0),
            'duration':    _pb_first(d, 3, 0),
            'remain_time': _pb_first(d, 4, 0),
        }
    return {'result': result}

def _decode_generic(payload):
    return {'code': _pb_first(_pb_decode(payload), 1, 0), 'msg': ''}

def _decode_temp(payload):
    f = _pb_decode(payload)
    def _flt(n):
        lst = f.get(n)
        if not lst: return 0.0
        try: return float(lst[0][1])
        except: return 0.0
    return {'result': {
        'box1_temp': _flt(1), 'box2_temp': _flt(2),
        'ptc1_temp': _flt(3), 'ptc2_temp': _flt(4),
        'env_temp':  _flt(5), 'env_humidity': _flt(6),
    }}

def _decode_filament_info(payload):
    f = _pb_decode(payload)
    sku = _pb_str(f, 3, ''); typ = _pb_str(f, 4, '')
    colors = []; primary_rgb = [0, 0, 0]
    for i, (_, c_data) in enumerate(f.get(5, [])):
        cf = _pb_decode(c_data)
        rgba = _pb_first(cf, 1, 0) & 0xFFFFFFFF
        r = (rgba >> 24) & 0xFF; g = (rgba >> 16) & 0xFF
        b = (rgba >> 8) & 0xFF;  a = rgba & 0xFF
        colors.append([r, g, b, a])
        if i == 0: primary_rgb = [r, g, b]
    if not colors: colors = [[0, 0, 0, 255]]
    ex_min = ex_max = bd_min = bd_max = 0
    if 6 in f:
        ef = _pb_decode(f[6][0][1])
        ex_min = _pb_first(ef, 1, 0); ex_max = _pb_first(ef, 2, 0)
    if 7 in f:
        hf = _pb_decode(f[7][0][1])
        bd_min = _pb_first(hf, 1, 0); bd_max = _pb_first(hf, 2, 0)
    diam_raw = _pb_first(f, 8, 175)
    try:    diameter = diam_raw / 100.0
    except: diameter = 1.75
    return {'result': {
        'index': _pb_first(f, 1, 0), 'sku': sku, 'type': typ, 'brand': '',
        'color': primary_rgb, 'colors': colors,
        'extruder_temp': {'min': ex_min, 'max': ex_max},
        'hotbed_temp':   {'min': bd_min, 'max': bd_max},
        'diameter': diameter, 'total': _pb_first(f, 9, 0),
        'remainder': _pb_first(f, 11, 0), 'code': _pb_first(f, 12, 0),
        'rfid': 2 if (sku or typ) else 1,
    }}

class AceDevice:

    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.add_object('ace_device', self)
        self.reactor = self.printer.get_reactor()
        self.gcode   = self.printer.lookup_object('gcode')

        self.serial_name = config.get('serial', '/dev/serial/by-id/*')
        all_devices = glob.glob('/dev/serial/by-id/*')
        preferred = [d for d in all_devices if 'usb-1a86_USB' in d]
        fallback = [d for d in all_devices if 'Klipper' not in d]
        candidates = preferred or fallback
        if candidates:
            self.serial_name = candidates[0]
            logging.info("ACE: Found serial port: %s", self.serial_name)
        else:
            all_devices = glob.glob('/dev/serial/by-id/*')
            if all_devices:
                logging.warning("ACE: No ACE 2 Pro device found. Available serial devices:")
                for d in sorted(all_devices):
                    logging.warning("ACE:   %s", d)
            else:
                logging.warning("ACE: No serial devices found at all in /dev/serial/by-id/")
            logging.warning("ACE: Stopping initialisation")
            return

        self.baud = config.getint('baud', 230400)
        if self.baud != 230400:
            self.baud = 230400
        self.feed_speed            = config.getint('feed_speed', 0)
        self.load_speed            = min(100, max(0, config.getint('load_speed', 100)))
        self.retract_speed         = min(100, max(0, config.getint('retract_speed', 100)))
        self.feed_check_len        = config.getint('feed_check_len', 100)
        self.feed_check_error_len  = config.getint('feed_check_error_len', 90)
        self.max_dryer_temp        = config.getint('max_dryer_temperature', 65)
        self.enable_feed_assist    = config.getboolean('enable_feed_assist', True)
        self.assist_mode_confirm_time = config.getfloat('assist_mode_confirm_time', 1.0)
        self.feed_assist_idle_timeout = config.getfloat('feed_assist_idle_timeout', 5.0)
        self.enable_feeder_mode    = config.getboolean('enable_feeder_mode', False)
        self.disable_u1_rfid       = config.getboolean('disable_u1_rfid', False)
        self.force_generic         = config.getboolean('force_generic', False)
        self.disable_air_print     = config.getboolean('disable_air_print', False)

        self.feed_lengths    = [config.getint('feed_length_slot%d' % (i+1), 1000) for i in range(4)]
        self.load_lengths    = [config.getint('load_length_slot%d' % (i+1), 850)  for i in range(4)]
        self.retract_lengths = [config.getint('retract_length_slot%d' % (i+1), 3000) for i in range(4)]

        self._connected       = False
        self._serial          = None
        self._queue           = queue.Queue()
        self._seq          = 0
        self._callback_map = {}
        self._rx_buf       = bytearray()
        self._info                  = {}
        self.port_sensor_hit        = [False, False, False, False]
        self._last_filament_status  = ['empty', 'empty', 'empty', 'empty']
        self._last_logged_slot_status = [None, None, None, None]
        self._active_feeds          = set()
        self._feed_start_times      = {}
        self._timeout_threshold     = 60.0
        self.auto_feed_step         = [0, 0, 0, 0]
        self._last_active_tool      = None
        self._last_active_index     = -1
        self._printing              = False
        self._assist_active_slots   = set()
        self._assist_mode_per_slot  = {}
        self._pending_mode_switch   = {}
        self._last_extrusion_times  = {}
        self._retract_in_progress   = set()
        self._writer_thread = None
        self._reader_thread = None

        self.printer.register_event_handler('klippy:ready',       self._handle_ready)
        self.printer.register_event_handler('klippy:disconnect',  self._handle_disconnect)
        self.printer.register_event_handler('filament_feed:port', self._feed_handler)
        self.printer.register_event_handler('print_stats:start',  self._handle_start_print_job)
        self.printer.register_event_handler('print_stats:stop',   self._handle_stop_print_job)
        self.printer.register_event_handler('idle_timeout:idle',  self._handle_not_printing)

        self.gcode.register_command('ACE_START_DRYING',    self.cmd_ACE_START_DRYING)
        self.gcode.register_command('ACE_STOP_DRYING',     self.cmd_ACE_STOP_DRYING)
        self.gcode.register_command('SET_FILAMENT_CONFIG', self.cmd_SET_FILAMENT_CONFIG)

    def feeder_mode(self):   return False
    def feed_assist(self):   return True
    def disable_rfid(self):  return self.disable_u1_rfid
    def disable_ap(self):    return self.disable_air_print
    
    
    def is_ready(self):      return self._connected and bool(self._info)
    def get_filament_detect(self, index):
        return 0 if self._last_filament_status[index] == 'empty' else 1

    def check_rfid_status(self):
        cfg = configparser.ConfigParser()
        try:
            cfg.read('/oem/printer_data/config/extended/extended2.cfg')
            return cfg.get('components', 'rfid', fallback=None) == "openrfid-generic"
        except Exception:
            return False

    def dwell(self, delay):
        self.printer.lookup_object('toolhead').dwell(delay)

    def _send(self, cmd, payload, decoder, callback):
        self._queue.put((cmd, payload, decoder, callback))

    def _handle_ready(self):
        self.connection_timer = self.reactor.register_timer(
            self._connection_timer, self.reactor.NOW)
        logging.info("ACE: connection monitor started")
        self._assist_loop_timer = self.reactor.register_timer(
            self._assist_loop_eval, self.reactor.NOW)

    def _handle_disconnect(self):
        self._connected = False
        if self._serial:
            try: self._serial.close()
            except: pass
        self._serial = None
        try: self.reactor.unregister_timer(self.main_timer)
        except: pass

    def _handle_start_print_job(self):
        logging.info("ACE: print job started")
        self._printing = True
        self._last_active_index = -1
        self._last_active_tool  = None
        self._last_extrusion_times.clear()

    def _handle_not_printing(self, _eventtime):
        logging.info("ACE: printer idle")
        self._printing = False
        if self._last_active_index != -1:
            for i in range(4):
                self._stop_feed_assist(i, why='printer_idle')
        self._last_active_index = -1
        self._last_active_tool  = None

    def _handle_stop_print_job(self):
        logging.info("ACE: print job stopped")
        self._printing = False
        if self._last_active_index != -1:
            for i in range(4):
                self._stop_feed_assist(i, why='print_stopped')
        self._last_active_index = -1
        self._last_active_tool  = None

    def _feed_handler(self, channel, detect):
        self.port_sensor_hit[channel] = detect

    def _connection_timer(self, eventtime):
        if not self._connected:
            self._attempt_connection()
        return eventtime + (5.0 if not self._connected else 20.0)

    def _attempt_connection(self):
        try:
            if self._serial:
                try: self._serial.close()
                except: pass
            self._serial = serial.Serial(
                port=self.serial_name, baudrate=self.baud, timeout=0.2)
            if not self._serial.isOpen():
                return
            self._connected = True
            self._assist_active_slots.clear()
            self._assist_mode_per_slot.clear()
            self._pending_mode_switch.clear()
            self._last_extrusion_times.clear()
            self._retract_in_progress.clear()
            self._rx_buf       = bytearray()
            self._callback_map = {}
            self._seq          = 0
            self._handshake()
            self._send(_CMD_SET_FEED_CHECK,
                       _pb_uint32(1, self.feed_check_len) + _pb_uint32(2, self.feed_check_error_len),
                       _decode_generic, None)
            if self._writer_thread is None or not self._writer_thread.is_alive():
                self._writer_thread = threading.Thread(target=self._writer)
                self._writer_thread.setDaemon(True)
                self._writer_thread.start()
            if self._reader_thread is None or not self._reader_thread.is_alive():
                self._reader_thread = threading.Thread(target=self._reader)
                self._reader_thread.setDaemon(True)
                self._reader_thread.start()
            self.main_timer = self.reactor.register_timer(
                self._main_eval, self.reactor.NOW)
            logging.info("ACE: connected successfully")
        except Exception as e:
            self._connected = False
            logging.error("ACE: connection failed: %s", e)

    def _handshake(self):
        try:
            self._serial.reset_input_buffer()
            self._serial.write(_build_packet(_CMD_DISCOVER_DEVICE, b'', seq=1))
            self._serial.flush()
            buf = bytearray()
            deadline = time.time() + 1.0
            uids = None
            while time.time() < deadline and uids is None:
                n = self._serial.in_waiting
                if n:
                    buf.extend(self._serial.read(n))
                    packets, buf = _parse_stream(buf)
                    for p in packets:
                        if (p['is_resp'] and p['cmd'] == _CMD_DISCOVER_DEVICE and p['payload']):
                            f = _pb_decode(p['payload'])
                            uids = (_pb_first(f, 1, 0), _pb_first(f, 2, 0), _pb_first(f, 3, 0))
                            break
                else:
                    time.sleep(0.02)
            if uids is not None:
                logging.info("ACE: discovered uid 0x%08X-0x%08X-0x%08X", uids[0], uids[1], uids[2])
                assign_pay = (_pb_uint32(1, uids[0]) + _pb_uint32(2, uids[1])
                              + _pb_uint32(3, uids[2]) + _pb_uint32(4, 1))
                self._serial.write(_build_packet(_CMD_ASSIGN_DEVICE_ID, assign_pay, seq=2))
                self._serial.flush()
                time.sleep(0.2)
            else:
                logging.info("ACE: no discover response; proceeding anyway")
            self._serial.reset_input_buffer()
            self._seq = 2
        except Exception as e:
            logging.warning("ACE: handshake failed (continuing): %s", e)

    def _next_seq(self):
        self._seq = (self._seq % 0xFFFE) + 1
        return self._seq

    def _dispatch(self, cmd, payload, decoder, callback):
        if not self._connected or not self._serial:
            return
        seq = self._next_seq()
        self._callback_map[seq] = (callback, decoder)
        if cmd in (_CMD_FEED_OR_ROLLBACK, _CMD_STOP_FEED_OR_ROLLBACK):
            logging.info("ACE: tx cmd=0x%02X seq=%d payload=%s",
                         cmd, seq, payload.hex() or '<empty>')
        try:
            self._serial.write(_build_packet(cmd, payload, seq=seq))
        except Exception as e:
            logging.error("ACE: write failed: %s", e)
            self._callback_map.pop(seq, None)
            self._connected = False

    def _writer(self):
        while self._connected:
            try:
                if not self._queue.empty():
                    task = self._queue.get()
                    if task:
                        cmd, payload, decoder, cb = task
                        self._dispatch(cmd, payload, decoder, cb)
                else:
                    self._dispatch(_CMD_GET_STATUS, b'',
                                   _decode_status, self._handle_status_update)
                time.sleep(0.5)
            except Exception as e:
                logging.error("ACE: writer error: %s", e)
                time.sleep(0.5)

    def _reader(self):
        while self._connected:
            try:
                n = self._serial.in_waiting
                if n:
                    self._rx_buf.extend(self._serial.read(n))
                else:
                    time.sleep(0.02)
                if not self._rx_buf:
                    continue
                packets, self._rx_buf = _parse_stream(self._rx_buf)
                for p in packets:
                    if not p['is_resp']:
                        continue
                    entry = self._callback_map.pop(p['seq'], None)
                    if not entry:
                        continue
                    callback, decoder = entry
                    try:
                        response = decoder(p['payload']) if decoder else {'code': 0, 'msg': ''}
                    except Exception as e:
                        logging.error("ACE: decode error cmd=0x%02X: %s", p['cmd'], e)
                        response = {'code': 0, 'msg': ''}
                    response['id'] = p['seq']
                    if callback:
                        try:
                            callback(self, response)
                        except Exception as e:
                            logging.error("ACE: callback error: %s\n%s", e, traceback.format_exc())
            except Exception as e:
                logging.error("ACE: reader error: %s", e)
                time.sleep(0.1)

    def _handle_status_update(self, _ace_ref, response):
        if 'result' not in response:
            return
        self._info = response['result']
        slots = self._info.get('slots') or []
        for i, slot in enumerate(slots[:4]):
            st   = slot.get('status')
            prev = self._last_logged_slot_status[i]
            if st != prev:
                logging.info("ACE: slot[%d] %s -> %s (rfid=%s)", i, prev, st, slot.get('rfid'))
                self._last_logged_slot_status[i] = st
                if i in self._retract_in_progress and st in ('empty', 'ready'):
                    logging.info("ACE: slot %d retract complete", i)
                    self._retract_in_progress.discard(i)
                if st == 'assist_error' and prev != 'assist_error':
                    self._handle_assist_error(i)
                elif st in ACE_ERROR_STATUSES and prev not in ACE_ERROR_STATUSES:
                    self._handle_slot_error(i, st)
                elif i in self._assist_active_slots and st in _STATES_INCOMPATIBLE_WITH_ASSIST:
                    logging.info("ACE: slot[%d] dropped assist (status=%s)", i, st)
                    self._clear_assist_tracking(i)
        self._check_auto_feed()

    def update_sensors(self, eventtime):
        for i in range(4):
            s = self.printer.lookup_object('filament_motion_sensor e%d_filament' % i, None)
            self.port_sensor_hit[i] = (s.get_status(eventtime).get('filament_detected', False)
                                       if s is not None else False)

    def get_sensor_state(self, index, eventtime):
        s = self.printer.lookup_object('filament_motion_sensor e%d_filament' % index, None)
        return s.get_status(eventtime).get('filament_detected', False) if s else False

    def _get_extruder_velocity(self):
        mr = self.printer.lookup_object('motion_report', None)
        if mr is None: return 0.0
        try:
            v = mr.get_status(self.reactor.monotonic()).get('live_extruder_velocity', 0.0)
            return float(v) if v is not None else 0.0
        except: return 0.0

    def _resolve_active_extruder_index(self, eventtime):
        th = self.printer.lookup_object('toolhead', None)
        if th is None: return None
        try: name = th.get_status(eventtime).get('extruder', '')
        except: return None
        if not name or not name.startswith('extruder'): return None
        suffix = name[8:]
        if suffix == '': return 0
        return int(suffix) if suffix.isdigit() else None

    def _filament_feed_active(self, idx=None):
        fd = self.printer.lookup_object('filament_detect', None)
        if fd is None: return False
        for _, f_obj in getattr(fd, 'filament_feed_objects', None) or []:
            states = getattr(f_obj, 'channel_state', None) or []
            for ch in range(len(states)):
                try:
                    if idx is not None and f_obj.filament_ch[ch] != idx: continue
                    stage = str(states[ch])
                except: continue
                if not any(stage.startswith(p) for p in _FF_ACTIVE_PREFIXES): continue
                if stage.endswith('_finish') or stage.endswith('_fail'): continue
                return True
        return False

    def _assist_log_cb(self, label, idx):
        def _cb(_ace_ref, resp):
            if not resp: return
            code = resp.get('code')
            if code: logging.warning("ACE: %s slot=%d FAILED code=%s", label, idx, code)
            else:    logging.info("ACE: %s slot=%d ack", label, idx)
        return _cb

    def _start_feed_assist(self, idx, why=''):
        logging.info("ACE: -> start_feed_assist slot=%d (%s)", idx, why)
        self._assist_active_slots.add(idx)
        self._assist_mode_per_slot[idx] = 'feed'
        def _cb(_a, resp):
            if not resp: return
            if resp.get('code'): logging.warning("ACE: start_feed_assist slot=%d FAILED", idx)
            else:                logging.info("ACE: start_feed_assist slot=%d ack", idx)
        self._send(_CMD_FEED_OR_ROLLBACK,
                   _pb_uint32(1, idx) + _pb_uint32(2, _FEED_ASSIST_SPEED) + _pb_uint32(3, 0) + _pb_uint32(4, _FEED_MODE_FEED_ASSIST),
                   _decode_generic, _cb)

    def _stop_feed_assist(self, idx, why=''):
        logging.info("ACE: -> stop_feed_assist slot=%d (%s)", idx, why)
        self._clear_assist_tracking(idx)
        self._send(_CMD_STOP_FEED_OR_ROLLBACK, _pb_uint32(1, idx),
                   _decode_generic, self._assist_log_cb('stop_feed_assist', idx))
        self._send(_CMD_FEED_OR_ROLLBACK,
                   _pb_uint32(1, idx) + _pb_uint32(2, self.retract_speed) + _pb_uint32(3, 12) + _pb_uint32(4, _FEED_MODE_ROLLBACK),
                   _decode_generic, None)

    def _start_unwind_assist(self, idx, why=''):
        logging.info("ACE: -> start_unwind_assist slot=%d (%s)", idx, why)
        self._assist_active_slots.add(idx)
        self._assist_mode_per_slot[idx] = 'unwind'
        def _cb(_a, resp):
            if not resp: return
            if resp.get('code'): logging.warning("ACE: unwind_assist slot=%d FAILED", idx)
            else:                logging.info("ACE: unwind_assist slot=%d ack", idx)
        self._send(_CMD_FEED_OR_ROLLBACK,
                   _pb_uint32(1, idx) + _pb_uint32(2, _UNWIND_ASSIST_SPEED) + _pb_uint32(3, 0) + _pb_uint32(4, _FEED_MODE_UNWIND_ASSIST),
                   _decode_generic, _cb)

    def _clear_assist_tracking(self, idx):
        self._assist_active_slots.discard(idx)
        self._assist_mode_per_slot.pop(idx, None)
        self._pending_mode_switch.pop(idx, None)
        self._last_extrusion_times.pop(idx, None)

    def _start_retract(self, idx):
        logging.info("ACE: retract slot=%d (%dmm @ %dmm/s)", idx, self.retract_lengths[idx], self.retract_speed)
        self._retract_in_progress.add(idx)
        self._send(_CMD_FEED_OR_ROLLBACK,
                   _pb_uint32(1, idx) + _pb_uint32(2, self.retract_speed) + _pb_uint32(3, self.retract_lengths[idx]) + _pb_uint32(4, _FEED_MODE_ROLLBACK),
                   _decode_generic, None)
        self._clear_assist_tracking(idx)

    def _assist_loop_eval(self, eventtime):
        if not self._connected:
            return eventtime + 0.25
        velocity = self._get_extruder_velocity()
        abs_vel  = abs(velocity)
        idx      = self._resolve_active_extruder_index(eventtime)
        if idx is None or idx < 0 or idx > 3:
            return eventtime + 0.25
        for s in list(self._assist_active_slots):
            last_t = self._last_extrusion_times.get(s, 0.0)
            if (not self._printing and last_t > 0
                    and eventtime - last_t > self.feed_assist_idle_timeout
                    and not self._filament_feed_active(s)):
                self._stop_feed_assist(s, why='idle_timeout')
        if abs_vel < _VELOCITY_IDLE_THRESHOLD:
            self._pending_mode_switch.pop(idx, None)
            return eventtime + 0.25
        self._last_extrusion_times[idx] = eventtime
        target_mode = 'feed' if velocity > 0 else 'unwind'
        if target_mode == 'unwind' and self._printing:
            target_mode = 'feed'
        self._ensure_assist_mode(idx, target_mode, abs_vel, eventtime)
        return eventtime + 0.25

    def _ensure_assist_mode(self, idx, target, target_velocity, eventtime):
        slots = (self._info or {}).get('slots') or []
        slot_status = slots[idx].get('status') if idx < len(slots) else None
        if slot_status in ('feeding', 'rollback', 'empty') or slot_status in ACE_ERROR_STATUSES:
            return
        current_mode = self._assist_mode_per_slot.get(idx)
        in_set       = idx in self._assist_active_slots
        if in_set and current_mode == target:
            self._pending_mode_switch.pop(idx, None)
            return
        if in_set and current_mode is not None and current_mode != target:
            pending = self._pending_mode_switch.get(idx)
            if pending is None or pending['target'] != target:
                self._pending_mode_switch[idx] = {'target': target, 'since': eventtime}
                return
            if eventtime - pending['since'] < self.assist_mode_confirm_time:
                return
            logging.info("ACE: mode switch slot=%d %s->%s confirmed", idx, current_mode, target)
            self._stop_feed_assist(idx, why='switch_%s_to_%s' % (current_mode, target))
            self._pending_mode_switch.pop(idx, None)
        if target == 'feed':
            self._start_feed_assist(idx, why='loop_v=+%.1fmms' % target_velocity)
        else:
            self._start_unwind_assist(idx, why='loop_v=-%.1fmms' % target_velocity)

    def _handle_assist_error(self, idx):
        self._clear_assist_tracking(idx)
        mode = 'unwind' if self._get_extruder_velocity() < 0 and not self._printing else 'feed'
        logging.info("ACE: slot %d assist_error -> restarting as %s", idx, mode)
        if mode == 'feed': self._start_feed_assist(idx, why='assist_error_restart')
        else:              self._start_unwind_assist(idx, why='assist_error_restart')

    def _handle_slot_error(self, idx, kind):
        msg = "ACE slot %d %s -- aborting" % (idx, kind)
        logging.error(msg)
        self._clear_assist_tracking(idx)
        self._active_feeds.discard(idx)
        self._feed_start_times.pop(idx, None)
        self._retract_in_progress.discard(idx)
        self._send(_CMD_STOP_FEED_OR_ROLLBACK, _pb_uint32(1, idx), _decode_generic, None)
        self.reactor.register_async_callback(
            lambda e, m=msg, i=idx, k=kind: self._raise_ace_error_async(m, i, k))

    _FILAMENT_FAIL_PREFIXES = (
        ('load_', 'load_fail'), ('unload_', 'unload_fail'),
        ('preload_', 'preload_fail'), ('manual_sta_', 'manual_sta_fail'),
    )

    def _abort_filament_op(self, idx):
        fd = self.printer.lookup_object('filament_detect', None)
        if fd is None: return False
        aborted = False
        for obj_name, f_obj in getattr(fd, 'filament_feed_objects', None) or []:
            ch_count = len(getattr(f_obj, 'channel_state', []) or [])
            for ch in range(ch_count):
                try:
                    if f_obj.filament_ch[ch] != idx: continue
                    stage = str(f_obj.channel_state[ch])
                except: continue
                fail_state = next((t for p, t in self._FILAMENT_FAIL_PREFIXES
                                   if stage.startswith(p) and not stage.endswith('_fail')), None)
                if fail_state is None: continue
                try:
                    if hasattr(f_obj, 'channel_error') and f_obj.channel_error[ch] in (None, '', 'ok'):
                        f_obj.channel_error[ch] = 'general'
                    if hasattr(f_obj, 'channel_error_state'):
                        f_obj.channel_error_state[ch] = f_obj.channel_state[ch]
                    if hasattr(f_obj, '_set_channel_state'):
                        f_obj._set_channel_state(ch, fail_state, True)
                    else:
                        f_obj.channel_state[ch] = fail_state
                    aborted = True
                    logging.info("ACE: forced %s ch=%d -> %s for slot %d", obj_name, ch, fail_state, idx)
                except Exception as e:
                    logging.error("ACE: abort op failed slot %d: %s", idx, e)
        return aborted

    def _raise_ace_error_async(self, msg, idx, kind):
        self._abort_filament_op(idx)
        short = "ACE slot %d %s" % (idx, kind)
        ds = self.printer.lookup_object('display_status', None)
        if ds:
            try: ds.message = short
            except: pass
        try: self.gcode.run_script_from_command("M117 %s" % short)
        except: pass
        try: self.gcode.respond_raw("!! %s" % msg)
        except: pass
        ps = self.printer.lookup_object('print_stats', None)
        if ps:
            try:
                if ps.get_status(self.reactor.monotonic()).get('state') == 'printing':
                    self.gcode.run_script_from_command('PAUSE')
            except: pass

    def _main_eval(self, eventtime):
        ps = self.printer.lookup_object('print_stats', None)
        if ps is not None:
            stats = ps.get_status(eventtime)
            current_state = stats.get('state')
            current_layer = ps.info_current_layer
            progress = 0.0
            if current_layer is None:
                vsd = self.printer.lookup_object('virtual_sdcard', None)
                if vsd:
                    progress = vsd.get_status(eventtime).get('progress', 0)
            if current_layer is None:
                current_layer = 0
            if current_state == "printing" and (current_layer > 0 or progress > 0.0):
                th = self.printer.lookup_object('toolhead', None)
                if th:
                    active_name = th.get_status(eventtime).get('extruder')
                    new_idx = 0
                    if active_name and active_name.startswith('extruder'):
                        num_part = active_name[8:]
                        new_idx = int(num_part) if num_part.isdigit() else 0
                    if active_name != self._last_active_tool:
                        logging.info("ACE: extruder %s->%s (idx %s->%s)",
                                     self._last_active_tool, active_name,
                                     self._last_active_index, new_idx)
                        self._last_active_tool  = active_name
                        self._last_active_index = new_idx
                        for s in list(self._assist_active_slots):
                            if s != new_idx:
                                self._stop_feed_assist(s, why='extruder_switch_to_%d' % new_idx)
                        if new_idx not in self._assist_active_slots:
                            self._start_feed_assist(new_idx, why='extruder_switch')
                return eventtime + 0.25

        feeds_to_remove = []
        for idx in self._active_feeds:
            if self.get_sensor_state(idx, eventtime):
                logging.info("ACE: sensor hit slot %d", idx)
                self._send(_CMD_STOP_FEED_OR_ROLLBACK, _pb_uint32(1, idx), _decode_generic, None)
                self._clear_assist_tracking(idx)
                feeds_to_remove.append(idx)
            elif eventtime > self._feed_start_times.get(idx, 0) + self._timeout_threshold:
                logging.warning("ACE: feed slot %d TIMEOUT", idx)
                self._send(_CMD_STOP_FEED_OR_ROLLBACK, _pb_uint32(1, idx), _decode_generic, None)
                self._clear_assist_tracking(idx)
                feeds_to_remove.append(idx)
        for idx in feeds_to_remove:
            self._active_feeds.discard(idx)
            self._feed_start_times.pop(idx, None)

        self.update_sensors(eventtime)

        msm = self.printer.lookup_object('machine_state_manager', None)
        fd  = self.printer.lookup_object('filament_detect', None)

        if msm is not None and fd is not None:
            msm_status  = msm.get_status()
            action_code = msm_status.get('action_code')

            if not hasattr(self, '_processed_extruders'):
                self._processed_extruders  = set()
                self._last_f_stages        = {}
                self._prev_channel_stages  = {}
                self._last_action          = action_code
                self._allow_triggers       = False
                logging.info("ACE: system initialised. Triggers locked until action change.")

            if action_code != self._last_action:
                logging.info("ACE: action changed to %s", action_code)
                self._allow_triggers = True
                self._last_action    = action_code

            for obj_name, f_obj in getattr(fd, 'filament_feed_objects', []):
                for ch in range(2):
                    actual_stage      = str(f_obj.channel_state[ch])
                    assigned_extruder = f_obj.filament_ch[ch]
                    state_key         = "%s_%d" % (obj_name, ch)
                    last_stage        = self._last_f_stages.get(state_key)

                    if actual_stage != last_stage:
                        logging.info("ACE: %s Ch %d | %s -> %s | ext=%s",
                                     obj_name, ch, last_stage, actual_stage, assigned_extruder)
                        if assigned_extruder is not None:
                            self._processed_extruders.discard("%d_%s" % (int(assigned_extruder), last_stage))
                        self._prev_channel_stages[state_key] = last_stage
                        self._last_f_stages[state_key]       = actual_stage

                    is_preload = actual_stage == "preload_finish"
                    if assigned_extruder is None or not (self._allow_triggers or is_preload):
                        continue

                    idx      = int(assigned_extruder)
                    gate_key = "%d_%s" % (idx, actual_stage)
                    if gate_key in self._processed_extruders:
                        continue

                    if actual_stage == "unload_finish":
                        logging.info("ACE: unload_finish slot=%d -> retract", idx)
                        if idx in self._assist_active_slots:
                            self._stop_feed_assist(idx, why='unload_finish')
                        self._start_retract(idx)
                        self._processed_extruders.add(gate_key)

                    elif actual_stage == "unload_fail":
                        logging.info("ACE: unload_fail slot=%d", idx)
                        if idx in self._assist_active_slots:
                            self._stop_feed_assist(idx, why='unload_fail')
                        self._retract_in_progress.discard(idx)
                        self._processed_extruders.add(gate_key)

                    elif actual_stage == "load_feeding":
                        logging.info("ACE: load_feeding slot=%d", idx)
                        if not self.get_sensor_state(idx, eventtime):
                            self._send(_CMD_FEED_OR_ROLLBACK, _pb_uint32(1, idx) + _pb_uint32(2, self.load_speed) + _pb_uint32(3, 3000) + _pb_uint32(4, _FEED_MODE_FEED), _decode_generic, None)
                            self._clear_assist_tracking(idx)
                            self._feed_start_times[idx] = eventtime
                            self._active_feeds.add(idx)
                        else:
                            self._send(_CMD_STOP_FEED_OR_ROLLBACK, _pb_uint32(1, idx), _decode_generic, None)
                            self._clear_assist_tracking(idx)
                        self._processed_extruders.add(gate_key)

                    elif actual_stage == "load_fail":
                        logging.info("ACE: load_fail slot=%d", idx)
                        self._send(_CMD_STOP_FEED_OR_ROLLBACK, _pb_uint32(1, idx), _decode_generic, None)
                        self._clear_assist_tracking(idx)
                        self._active_feeds.discard(idx)
                        self._feed_start_times.pop(idx, None)
                        self._processed_extruders.add(gate_key)

                    elif actual_stage == "load_finish":
                        logging.info("ACE: load_finish slot=%d", idx)
                        self._send(_CMD_STOP_FEED_OR_ROLLBACK, _pb_uint32(1, idx), _decode_generic, None)
                        self._clear_assist_tracking(idx)
                        self._active_feeds.discard(idx)
                        self._feed_start_times.pop(idx, None)
                        self._processed_extruders.add(gate_key)

                    elif actual_stage == "preload_finish":
                        prev = self._prev_channel_stages.get(state_key) or ''
                        is_post_unload = (prev.startswith('unload_')
                                          and not prev.endswith('_finish')
                                          and not prev.endswith('_fail'))
                        if is_post_unload:
                            logging.info("ACE: post-unload preload_finish (prev=%s) -> retract slot=%d", prev, idx)
                            if idx in self._assist_active_slots:
                                self._stop_feed_assist(idx, why='post_unload_preload')
                            self._start_retract(idx)
                        else:
                            logging.info("ACE: preload_finish slot=%d -> rfid", idx)
                            self.set_slot_rfid_info(idx)
                        self._processed_extruders.add(gate_key)

        return eventtime + 0.25

    def _check_auto_feed(self):
        slots = self._info.get('slots', [])
        if not slots: return
        ace_is_busy = any(s.get('status') in ['feeding', 'preload'] for s in slots)
        for i in range(min(len(slots), 4)):
            current_status = slots[i].get('status')
            if current_status == 'empty':
                if self._last_filament_status[i] != 'empty':
                    self.clear_slot_rfid_info(i)
                    logging.info("ACE: slot %d empty", i)
                self.auto_feed_step[i] = 0
                self._last_filament_status[i] = 'empty'
                continue
            if self.auto_feed_step[i] == 0:
                if self._last_filament_status[i] == 'preload' and current_status == 'ready':
                    if not ace_is_busy:
                        self._clear_assist_tracking(i)
                        self._active_feeds.add(i)
                        self.auto_feed_step[i] = 2
                        ace_is_busy = True
            elif self.auto_feed_step[i] == 2:
                if not ace_is_busy:
                    logging.info("ACE: slot %d set rfid", i)
                    self.set_slot_rfid_info(i)
                    self.auto_feed_step[i] = 3; ace_is_busy = True
            elif self.auto_feed_step[i] == 3:
                if current_status == 'ready':
                    self.auto_feed_step[i] = 0
            self._last_filament_status[i] = current_status

    def set_slot_rfid_info(self, index):
        def _callback(_ace_ref, resp):
            if not resp or 'result' not in resp:
                return
            s = resp['result']
            logging.info("ACE: filament_info slot=%d sku=%s type=%s brand=%s "
                         "diameter=%s color=%s total=%s remainder=%s rfid=%s",
                         index, s.get('sku'), s.get('type'), s.get('brand'),
                         s.get('diameter'), s.get('color'), s.get('total'),
                         s.get('remainder'), s.get('rfid'))
            if s.get('rfid', 1) != 2:
                return
            sku        = s.get('sku', '') or ''
            f_type     = s.get('type', '') or ''
            brand      = s.get('brand', '') or ''
            color_list = s.get('color', [0, 0, 0])
            colors_raw = s.get('colors', [[0, 0, 0, 255]])
            r, g, b    = color_list[0], color_list[1], color_list[2]
            rgb_packed  = "%02X%02X%02X" % (r, g, b)
            rgb_ipacked = (r << 16) | (g << 8) | b
            alpha       = colors_raw[0][3] if colors_raw and len(colors_raw[0]) > 3 else 255
            alpha_hex   = "%02X" % alpha
            if brand.lower() == "ac":
                parts  = f_type.split(" ", 1)
                f_type = parts[0]
                brand  = parts[1] if len(parts) > 1 else ""
                sku    = "Anycubic"
            if self.force_generic or self.check_rfid_status():
                if sku.lower() != "snapmaker":
                    sku = "Generic"
            if sku.lower() != "snapmaker" and brand.lower() == "basic":
                brand = ""
            fd = self.printer.lookup_object('filament_detect', None)
            if fd is None:
                self.gcode.run_script_from_command(
                    "SET_PRINT_FILAMENT_CONFIG CONFIG_EXTRUDER=%d "
                    "VENDOR='%s' FILAMENT_TYPE='%s' FILAMENT_SUBTYPE='%s' "
                    "FILAMENT_COLOR_RGBA=%s%s"
                    % (index, sku, f_type, brand, rgb_packed, alpha_hex))
            else:
                lm = {330: 1000, 247: 750, 198: 600, 165: 500, 82: 250}
                total_len = s.get('total')
                info = copy.deepcopy(filament_protocol.FILAMENT_INFO_STRUCT)
                info['VERSION']    = 1
                info['VENDOR']     = sku
                info['MANUFACTURER'] = sku
                info['MAIN_TYPE']  = f_type
                info['SUB_TYPE']   = brand
                info['COLOR_NUMS'] = 1
                info['RGB_1']      = rgb_ipacked
                try:    info['ALPHA'] = max(0x00, min(0xFF, int(alpha)))
                except: info['ALPHA'] = 0xFF
                info['ARGB_COLOR']       = info['ALPHA'] << 24 | info['RGB_1']
                info['OFFICIAL']         = True
                info['LENGTH']           = total_len
                try:    info['DIAMETER'] = int(round(float(s.get('diameter', 1.75)) * 100))
                except: info['DIAMETER'] = 175
                try:    info['WEIGHT']   = int(lm.get(total_len, 1000))
                except: info['WEIGHT']   = 1000
                try:
                    info['HOTEND_MIN_TEMP'] = int(s.get('extruder_temp', {}).get('min', 190))
                    info['HOTEND_MAX_TEMP'] = int(s.get('extruder_temp', {}).get('max', 220))
                except:
                    info['HOTEND_MIN_TEMP'] = 0
                    info['HOTEND_MAX_TEMP'] = 0
                try:
                    bmin = int(s.get('hotbed_temp', {}).get('min', 50))
                    bmax = int(s.get('hotbed_temp', {}).get('max', 60))
                    info['BED_TEMP'] = bmin if bmin > 0 else bmax
                except: info['BED_TEMP'] = 0
                info['FIRST_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
                info['OTHER_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
                info['MF_DATE']  = datetime.now().strftime("%Y%m%d")
                info['CARD_UID'] = [random.randint(0, 255) for _ in range(7)]
                fd._filament_info_update(index, info, is_clear=True)
        self._send(_CMD_GET_FILAMENT_INFO, _pb_uint32(1, index),
                   _decode_filament_info, _callback)

    def clear_slot_rfid_info(self, index):
        fd = self.printer.lookup_object('filament_detect', None)
        if fd is None:
            self.gcode.run_script_from_command(
                "SET_PRINT_FILAMENT_CONFIG CONFIG_EXTRUDER=%d "
                "VENDOR='' FILAMENT_TYPE='' FILAMENT_SUBTYPE='' "
                "FILAMENT_COLOR_RGBA=000000FF" % index)
        else:
            info = copy.deepcopy(filament_protocol.FILAMENT_INFO_STRUCT)
            fd._filament_info_update(index, info, is_clear=True)

    cmd_ACE_START_DRYING_help = 'Start ACE 2 Pro filament dryer'
    def cmd_ACE_START_DRYING(self, gcmd):
        temperature = gcmd.get_int('TEMPERATURE')
        duration    = gcmd.get_int('DURATION', 240)
        if duration <= 0:
            raise gcmd.error('Wrong duration')
        if temperature <= 0 or temperature > self.max_dryer_temp:
            raise gcmd.error('Wrong temperature')
        def callback(_ace_ref, response):
            if 'code' in response and response['code'] != 0:
                raise gcmd.error("ACE Error: " + response.get('msg', ''))
            self.gcode.respond_info('Started ACE drying at %dC for %d min' % (temperature, duration))
        self._send(_CMD_DRYING,
                   _pb_uint32(1, temperature) + _pb_uint32(2, duration) + _pb_bool(3, True),
                   _decode_generic, callback)

    cmd_ACE_STOP_DRYING_help = 'Stop ACE 2 Pro filament dryer'
    def cmd_ACE_STOP_DRYING(self, gcmd):
        def callback(_ace_ref, response):
            if 'code' in response and response['code'] != 0:
                raise gcmd.error("ACE Error: " + response.get('msg', ''))
            self.gcode.respond_info('Stopped ACE drying')
        self._send(_CMD_DRYING, _pb_uint32(1, 0) + _pb_uint32(2, 0), _decode_generic, callback)

    cmd_SET_FILAMENT_CONFIG_help = 'Set filament information'
    def cmd_SET_FILAMENT_CONFIG(self, gcmd):
        channel           = gcmd.get_int('CHANNEL')
        vendor            = gcmd.get('VENDOR', '')
        filament_type     = gcmd.get('TYPE', '')
        subtype           = gcmd.get('SUBTYPE', '')
        color             = gcmd.get('COLOR', '000000')
        alpha             = gcmd.get('ALPHA', 'FF')
        official          = gcmd.get('OFFICIAL', 'false').lower()
        length            = gcmd.get_int('LENGTH', 330)
        diameter          = gcmd.get_int('DIAMETER', 175)
        weight            = gcmd.get_int('WEIGHT', 1000)
        extruder_temp_min = gcmd.get_int('EXT_TEMP_MIN', 190)
        extruder_temp_max = gcmd.get_int('EXT_TEMP_MAX', 220)
        hotbed_temp_min   = gcmd.get_int('BED_TEMP_MIN', 50)
        hotbed_temp_max   = gcmd.get_int('BED_TEMP_MAX', 60)
        if self.force_generic or self.check_rfid_status():
            if vendor.lower() != "snapmaker":
                vendor = "Generic"
        if vendor.lower() != "snapmaker" and subtype.lower() == "basic":
            subtype = ""
        if channel < 0 or channel >= 4:
            raise gcmd.error("ACE channel %d out of range [0,3]" % channel)
        fd = self.printer.lookup_object('filament_detect', None)
        if fd is None:
            self.gcode.run_script_from_command(
                "SET_PRINT_FILAMENT_CONFIG CONFIG_EXTRUDER=%d "
                "VENDOR='%s' FILAMENT_TYPE='%s' FILAMENT_SUBTYPE='%s' "
                "FILAMENT_COLOR_RGBA=%s%s"
                % (channel, vendor, filament_type, subtype, color, alpha))
        else:
            info = copy.deepcopy(filament_protocol.FILAMENT_INFO_STRUCT)
            info['VERSION']          = 1
            info['VENDOR']           = vendor
            info['MANUFACTURER']     = vendor
            info['MAIN_TYPE']        = filament_type
            info['SUB_TYPE']         = subtype
            info['COLOR_NUMS']       = 1
            info['RGB_1']            = int(color, 16)
            info['ALPHA']            = int(alpha, 16)
            info['ARGB_COLOR']       = (int(alpha, 16) << 24) | int(color, 16)
            info['OFFICIAL']         = official in ['true', '1', 'yes']
            info['LENGTH']           = length
            info['DIAMETER']         = diameter
            info['WEIGHT']           = weight
            info['HOTEND_MIN_TEMP']  = extruder_temp_min
            info['HOTEND_MAX_TEMP']  = extruder_temp_max
            info['BED_TEMP']         = hotbed_temp_min if hotbed_temp_min > 0 else hotbed_temp_max
            info['FIRST_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
            info['OTHER_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
            info['MF_DATE']          = datetime.now().strftime("%Y%m%d")
            info['CARD_UID']         = [random.randint(0, 255) for _ in range(7)]
            fd._filament_info_update(channel, info, is_clear=True)
            self.gcode.respond_info(str(info))


def load_config(config):
    return AceDevice(config)
