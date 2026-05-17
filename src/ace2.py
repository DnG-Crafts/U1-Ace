# https://github.com/DnG-Crafts/U1-Ace
import serial, threading, time, logging, struct, queue, glob, copy, random, configparser
from datetime import datetime
from . import filament_protocol

PREAMBLE      = b'\xff\xaa'
END_MARKER    = 0xFE
FLAG_REQUEST  = 0x00
FLAG_RESPONSE = 0x80

class Cmd:
    DISCOVER_DEVICE       = 0
    ASSIGN_DEVICE_ID      = 1
    GET_STATUS            = 6
    GET_INFO              = 7
    FEED_OR_ROLLBACK      = 8
    STOP_FEED_OR_ROLLBACK = 9
    UPDATE_SPEED          = 10
    DRYING                = 11
    SET_DRY_TEMP          = 12
    GET_FILAMENT_INFO     = 13
    SET_RFID_ENABLE       = 14
    SET_FEED_CHECK        = 19
    GET_TEMP              = 64
    SET_FAN               = 71

WORK_STATES = {0: "INIT", 1: "IDLE", 2: "BUSY", 3: "UPGRADE"}
SLOT_STATES = {
    0: "READY", 1: "FEEDING", 2: "ROLLBACK", 3: "ASSISTING",
    4: "ROLLBACK_ASSISTING", 5: "PRELOADING", 6: "UPGRADING",
    129: "FEED_ERROR", 130: "ROLLBACK_ERROR", 131: "ASSIST_ERROR",
    132: "PRELOAD_ERROR", 133: "STUCK_ERROR", 134: "TANGLED_ERROR",
    135: "MOTOR_ERROR",
}
FILAMENT_STATES = {0: "EMPTY", 1: "UNKNOWN", 2: "IDENTIFIED", 3: "IDENTIFYING"}
DRY_STATES = {0: "FREE", 1: "STARTING", 2: "KEEPING", 3: "STOPPING", 4: "PTC_ERROR", 5: "NTC_ERROR"}

FEED_MODE_FORWARD  = 0
FEED_MODE_ROLLBACK = 1
ASSIST_MODE_FORWARD = 2
ASSIST_MODE_ROLLBACK = 3

def _crc16_kermit(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF

def pb_varint(value: int) -> bytes:
    r = bytearray()
    while value > 0x7F:
        r.append((value & 0x7F) | 0x80)
        value >>= 7
    r.append(value & 0x7F)
    return bytes(r)

def pb_uint32(field: int, value: int) -> bytes:
    return pb_varint((field << 3) | 0) + pb_varint(value)

def pb_bool(field: int, value: bool) -> bytes:
    return pb_varint((field << 3) | 0) + pb_varint(1 if value else 0)

def pb_string(field: int, value: str) -> bytes:
    enc = value.encode('utf-8')
    return pb_varint((field << 3) | 2) + pb_varint(len(enc)) + enc

def pb_decode_varint(data: bytes, pos: int):
    result, shift = 0, 0
    while pos < len(data):
        b = data[pos]; pos += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, pos
        shift += 7
    return result, pos

def pb_decode(data: bytes) -> dict:
    fields, pos = {}, 0
    while pos < len(data):
        tag, pos = pb_decode_varint(data, pos)
        fnum, wtype = tag >> 3, tag & 7
        if wtype == 0:
            val, pos = pb_decode_varint(data, pos)
        elif wtype == 1:
            val = struct.unpack_from('<d', data, pos)[0] if pos + 8 <= len(data) else 0
            pos += 8
        elif wtype == 2:
            ln, pos = pb_decode_varint(data, pos)
            val = data[pos:pos + ln]; pos += ln
        elif wtype == 5:
            val = struct.unpack_from('<f', data, pos)[0] if pos + 4 <= len(data) else 0
            pos += 4
        else:
            break
        fields.setdefault(fnum, []).append((wtype, val))
    return fields

def _fval(fields: dict, num: int, default=0):
    return fields.get(num, [(0, default)])[0][1]

def _build_packet(cmd: int, payload: bytes = b'', seq: int = 1) -> bytes:
    plen = min(len(payload), 100)
    inner = bytearray([FLAG_REQUEST, seq & 0xFF, (seq >> 8) & 0xFF, cmd & 0xFF, plen & 0xFF])
    inner.extend(payload[:plen])
    crc = _crc16_kermit(bytes(inner))
    return bytes(PREAMBLE + inner + bytes([crc & 0xFF, (crc >> 8) & 0xFF, END_MARKER]))

def _parse_packet(buf: bytearray):
    while len(buf) >= 2:
        idx = buf.find(PREAMBLE)
        if idx < 0:
            return None, max(0, len(buf) - 1)
        if idx > 0:
            return None, idx
        if len(buf) < 10:
            return None, 0
        for end in range(9, min(len(buf), 120)):
            if buf[end] != END_MARKER:
                continue
            flags = buf[2]
            seq   = buf[3] | (buf[4] << 8)
            cmd   = buf[5]
            plen  = buf[6]
            exp   = 7 + plen + 2
            if end != exp:
                continue
            inner   = bytes(buf[2:7 + plen])
            crc_r   = buf[7 + plen] | (buf[8 + plen] << 8)
            if crc_r != _crc16_kermit(inner):
                return None, end + 1
            return {
                "cmd":     cmd,
                "is_resp": bool(flags & FLAG_RESPONSE),
                "flags":   flags,
                "seq":     seq,
                "payload": bytes(buf[7:7 + plen]),
            }, end + 1
        return None, 2 if len(buf) > 120 else 0
    return None, 0


class AceDevice:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.add_object('ace_device', self)
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        self.serial_name = config.get('serial', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B5F070433-if00')
        all_devices = glob.glob('/dev/serial/by-id/usb-1a86_USB*')
        if all_devices:
            self.serial_name = all_devices[0]
            logging.info("ACE: Found serial port: %s", self.serial_name)
        else:
            logging.info("ACE: No devices auto detected, Using config serial path %s", self.serial_name)

        self.baud = config.getint('baud', 230400)
        self.feed_speed = config.getint('feed_speed', 90)
        self.assist_speed = config.getint('assist_speed', 10)
        self.load_speed = config.getint('load_speed', 100)
        self.retract_speed = config.getint('retract_speed', 25)
        self.max_dryer_temp = config.getint('max_dryer_temperature', 65)
        self.enable_feed_assist = config.getboolean('enable_feed_assist', True)
        self.enable_feeder_mode = config.getboolean('enable_feeder_mode', False)
        self.disable_u1_rfid = config.getboolean('disable_u1_rfid', False)
        self.force_generic = config.getboolean('force_generic', False)
        self.disable_air_print = config.getboolean('disable_air_print', False)
        
        self.feed_lengths = [
            config.getint('feed_length_slot1', 1000),
            config.getint('feed_length_slot2', 1000),
            config.getint('feed_length_slot3', 1000),
            config.getint('feed_length_slot4', 1000)
        ]

        self.load_lengths = [
            config.getint('load_length_slot1', 850),
            config.getint('load_length_slot2', 850),
            config.getint('load_length_slot3', 850),
            config.getint('load_length_slot4', 850)
        ]

        self.retract_lengths = [
            config.getint('retract_length_slot1', 100),
            config.getint('retract_length_slot2', 100),
            config.getint('retract_length_slot3', 100),
            config.getint('retract_length_slot4', 100)
        ]

        self._connected = False
        self._serial = None
        self._seq = 0
        self._cb_lock = threading.Lock()
        self._queue = queue.Queue()
        self._callback_map = {}
        self._info = {}
        self._feed_assist_index = -1
        self._last_filament_data = [("", "", [0, 0, 0])] * 4
        self._virtual_uids = [[0]*7 for _ in range(4)]
        self._initialized = False
        self.port_sensor_hit = [False, False, False, False]
        self._last_filament_status = ['empty', 'empty', 'empty', 'empty']
        self._active_feeds = set()
        self._feed_start_times = {}
        self._timeout_threshold = 60.0
        self.auto_feed_step = [0, 0, 0, 0]
        self.feed_sent = [False, False, False, False]
        self._last_active_tool = None
        self._last_active_index = -1
        self._pending_start_index = -1
        self._next_cmd_time = 0

        self._processed_extruders = set()
        self._last_f_stages = {}
        self._last_action = None
        self._allow_triggers = False

        self._writer_thread = None
        self._reader_thread = None

        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('klippy:disconnect', self._handle_disconnect)
        self.printer.register_event_handler("filament_feed:port", self._feed_handler)
        self.printer.register_event_handler('print_stats:start', self._handle_start_print_job)
        self.printer.register_event_handler('print_stats:stop', self._handle_stop_print_job)
        self.printer.register_event_handler('idle_timeout:idle', self._handle_not_printing)

        self.gcode.register_command('ACE_START_DRYING', self.cmd_ACE_START_DRYING)
        self.gcode.register_command('ACE_STOP_DRYING', self.cmd_ACE_STOP_DRYING)
        self.gcode.register_command('SET_FILAMENT_CONFIG', self.cmd_SET_FILAMENT_CONFIG)

    def feeder_mode(self):
        return self.enable_feeder_mode

    def feed_assist(self):
        return self.enable_feed_assist

    def disable_rfid(self):
        return self.disable_u1_rfid

    def disable_ap(self):
        return self.disable_air_print
        
    def check_rfid_status(self):
        config = configparser.ConfigParser()
        try:
            config.read('/oem/printer_data/config/extended/extended2.cfg')
            rfid_value = config.get('components', 'rfid', fallback=None)
            return rfid_value == "openrfid-generic"
        except Exception:
            return False

    def _handle_start_print_job(self):
        logging.info("ACE: Print job started.")
        self._last_active_index = -1
        self._last_active_tool = None

    def _handle_not_printing(self, eventtime):
        logging.info("ACE: Printer is now IDLE/Ready.")
        if self._last_active_index != -1 and self.enable_feed_assist:
            for i in range(4):
                self.send_request(cmd=Cmd.STOP_FEED_OR_ROLLBACK,
                                  payload=pb_uint32(1, i),
                                  callback=None)
        self._last_active_index = -1
        self._last_active_tool = None

    def _handle_stop_print_job(self):
        logging.info("ACE: Print job stopped/completed.")
        if self._last_active_index != -1 and self.enable_feed_assist:
            for i in range(4):
                self.send_request(cmd=Cmd.STOP_FEED_OR_ROLLBACK,
                                  payload=pb_uint32(1, i),
                                  callback=None)
        self._last_active_index = -1
        self._last_active_tool = None

    def _next_seq(self) -> int:
        with self._cb_lock:
            self._seq = (self._seq % 0xFFFF) + 1
            return self._seq

    def _send_packet(self, cmd: int, payload: bytes, seq: int):
        if not self._connected or not self._serial:
            return
        try:
            data = _build_packet(cmd, payload, seq)
            self._serial.write(data)
        except Exception as e:
            logging.error("ACE: Serial write failed, disconnecting: %s", e)
            self._connected = False

    def send_request(self, cmd: int, payload: bytes = b'', callback=None):
        self._queue.put((cmd, payload, callback))

    def _handle_ready(self):
        self.connection_timer = self.reactor.register_timer(
            self._connection_timer, self.reactor.NOW)
        logging.info("ACE: Connection monitor started")

    def _connection_timer(self, eventtime):
        if not self._connected:
            self._attempt_connection()
        return eventtime + (5.0 if not self._connected else 20.0)

    def _attempt_connection(self):
        try:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass

            self._serial = serial.Serial(
                port=self.serial_name, baudrate=self.baud, timeout=0.2)
            if self._serial.isOpen():
                self._connected = True

                if self._writer_thread is None or not self._writer_thread.is_alive():
                    self._writer_thread = threading.Thread(
                        target=self._writer, daemon=True)
                    self._writer_thread.start()

                if self._reader_thread is None or not self._reader_thread.is_alive():
                    self._reader_thread = threading.Thread(
                        target=self._reader, daemon=True)
                    self._reader_thread.start()

                self.main_timer = self.reactor.register_timer(
                    self._main_eval, self.reactor.NOW)
                logging.info("ACE: Connected successfully")
        except Exception as e:
            self._connected = False
            logging.error("ACE: Connection attempt failed: %s", e)

    def _handle_disconnect(self):
        self._connected = False
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        try:
            self.reactor.unregister_timer(self.main_timer)
        except Exception:
            pass

    def _feed_handler(self, channel, detect):
        self.port_sensor_hit[channel] = detect

    def update_sensors(self, eventtime):
        if self.enable_feeder_mode:
            return
        for i in range(4):
            sensor_name = 'filament_motion_sensor e%d_filament' % i
            s_obj = self.printer.lookup_object(sensor_name, None)
            if s_obj is not None:
                status = s_obj.get_status(eventtime)
                self.port_sensor_hit[i] = status.get('filament_detected', False)
            else:
                self.port_sensor_hit[i] = False

    def get_sensor_state(self, index, eventtime):
        sensor_name = 'filament_motion_sensor e%d_filament' % index
        s_obj = self.printer.lookup_object(sensor_name, None)
        if s_obj is not None:
            status = s_obj.get_status(eventtime)
            return status.get('filament_detected', False)
        return False

    def _main_eval(self, eventtime):
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is not None:
            stats = print_stats.get_status(eventtime)
            current_state = stats.get('state')
            progress = 0.0
            current_layer = print_stats.info_current_layer
            if current_layer is None:
                vsd = self.printer.lookup_object('virtual_sdcard', None)
                if vsd:
                    vsd_status = vsd.get_status(eventtime)
                    if vsd_status:
                        progress = vsd_status.get('progress', 0)

            if current_layer is None:
                current_layer = 0

            if current_state == "printing" and (current_layer > 0 or progress > 0.0):
                toolhead = self.printer.lookup_object('toolhead', None)
                if toolhead:
                    th_status = toolhead.get_status(eventtime)
                    active_name = th_status.get('extruder')
                    new_idx = 0
                    if active_name.startswith('extruder'):
                        num_part = active_name[8:]
                        new_idx = int(num_part) if num_part.isdigit() else 0

                    if active_name != self._last_active_tool:
                        old_idx = self._last_active_index
                        logging.info(
                            "ACE: Swap detected: %s (Idx %s) -> %s (Idx %s)",
                            self._last_active_tool, old_idx, active_name, new_idx)

                        self._last_active_tool = active_name
                        self._last_active_index = new_idx

                        if self.enable_feed_assist:
                            self.send_request(
                                cmd=Cmd.FEED_OR_ROLLBACK,
                                payload=(pb_uint32(1, new_idx)
                                         + pb_uint32(2, self.assist_speed)
                                         + pb_uint32(3, 0)
                                         + pb_uint32(4, ASSIST_MODE_FORWARD)),
                                callback=None)

                return eventtime + 0.25

        if not self.enable_feed_assist or not self.enable_feeder_mode:
            feeds_to_remove = []
            for idx in self._active_feeds:
                if self.get_sensor_state(idx, eventtime):
                    self.send_request(
                        cmd=Cmd.STOP_FEED_OR_ROLLBACK,
                        payload=pb_uint32(1, idx),
                        callback=None)
                    feeds_to_remove.append(idx)
                elif eventtime > self._feed_start_times.get(idx, 0) + self._timeout_threshold:
                    logging.warning("ACE: Feed slot %d TIMEOUT. Stopping.", idx)
                    self.send_request(
                        cmd=Cmd.STOP_FEED_OR_ROLLBACK,
                        payload=pb_uint32(1, idx),
                        callback=None)
                    feeds_to_remove.append(idx)

            for idx in feeds_to_remove:
                self._active_feeds.discard(idx)
                self._feed_start_times.pop(idx, None)

        self.update_sensors(eventtime)

        msm = self.printer.lookup_object('machine_state_manager', None)
        fd = self.printer.lookup_object('filament_detect', None)

        if msm is not None and fd is not None:
            msm_status = msm.get_status()
            action_code = msm_status.get('action_code')

            if action_code != self._last_action:
                logging.info("ACE: Action changed to %s.", action_code)
                if self._last_action is not None:
                    self._allow_triggers = True
                self._last_action = action_code

            for obj_name, f_obj in getattr(fd, 'filament_feed_objects', []):
                for ch in range(2):
                    actual_stage = str(f_obj.channel_state[ch])
                    assigned_extruder = f_obj.filament_ch[ch]
                    state_key = f"{obj_name}_{ch}"

                    if actual_stage != self._last_f_stages.get(state_key):
                        logging.info(
                            "ACE: %s Ch %d | Stage: %s | Extruder: %s",
                            obj_name, ch, actual_stage, assigned_extruder)
                        if assigned_extruder is not None:
                            old_stage = self._last_f_stages.get(state_key)
                            old_gate_key = f"{int(assigned_extruder)}_{old_stage}"
                            self._processed_extruders.discard(old_gate_key)

                        self._last_f_stages[state_key] = actual_stage

                    is_preload = actual_stage == "preload_finish"
                    if assigned_extruder is not None and (self._allow_triggers or is_preload):
                        idx = int(assigned_extruder)
                        gate_key = f"{idx}_{actual_stage}"

                        if gate_key not in self._processed_extruders:
                            if actual_stage == "unload_finish":
                                logging.info("ACE: [SERIAL] -> UNWIND_SLOT=%d", idx)
                                self.send_request(
                                    cmd=Cmd.FEED_OR_ROLLBACK,
                                    payload=(pb_uint32(1, idx)
                                             + pb_uint32(2, self.retract_speed)
                                             + pb_uint32(3, self.retract_lengths[idx])
                                             + pb_uint32(4, FEED_MODE_ROLLBACK)),
                                    callback=None)
                                self._processed_extruders.add(gate_key)

                            elif actual_stage == "load_feeding":
                                logging.info("ACE: [SERIAL] -> LOAD_SLOT=%d", idx)
                                if self.enable_feed_assist and self.enable_feeder_mode:
                                    self.send_request(
                                        cmd=Cmd.FEED_OR_ROLLBACK,
                                        payload=(pb_uint32(1, idx)
                                                 + pb_uint32(2, self.assist_speed)
                                                 + pb_uint32(3, 0)
                                                 + pb_uint32(4, ASSIST_MODE_FORWARD)),
                                        callback=None)
                                else:
                                    if not self.get_sensor_state(idx, eventtime):
                                        self.send_request(
                                            cmd=Cmd.FEED_OR_ROLLBACK,
                                            payload=(pb_uint32(1, idx)
                                                     + pb_uint32(2, 60)
                                                     + pb_uint32(3, 3000)
                                                     + pb_uint32(4, FEED_MODE_FORWARD)),
                                            callback=None)
                                        self._feed_start_times[idx] = eventtime
                                        self._active_feeds.add(idx)
                                    else:
                                        self.send_request(
                                            cmd=Cmd.STOP_FEED_OR_ROLLBACK,
                                            payload=pb_uint32(1, idx),
                                            callback=None)
                                self._processed_extruders.add(gate_key)

                            elif actual_stage == "load_fail":
                                logging.info("ACE: [SERIAL] -> FAILED_SLOT=%d", idx)
                                self.send_request(
                                    cmd=Cmd.STOP_FEED_OR_ROLLBACK,
                                    payload=pb_uint32(1, idx),
                                    callback=None)
                                self._active_feeds.discard(idx)
                                self._feed_start_times.pop(idx, None)
                                self._processed_extruders.add(gate_key)

                            elif actual_stage == "load_finish":
                                logging.info("ACE: [SERIAL] -> LOAD_COMPLETE_SLOT=%d", idx)
                                self.send_request(
                                    cmd=Cmd.STOP_FEED_OR_ROLLBACK,
                                    payload=pb_uint32(1, idx),
                                    callback=None)
                                self._active_feeds.discard(idx)
                                self._feed_start_times.pop(idx, None)
                                self._processed_extruders.add(gate_key)

                            elif actual_stage == "preload_finish":
                                logging.info("ACE: PRELOAD_COMPLETE_SLOT=%d", idx)
                                self.set_slot_rfid_info(idx)
                                self._processed_extruders.add(gate_key)

        return eventtime + 0.25

    def is_ready(self):
        return self._connected and bool(self._info)

    def get_filament_detect(self, index):
        return 0 if self._last_filament_status[index] == 'empty' else 1

    def _parse_status_response(self, payload: bytes) -> dict:
        f = pb_decode(payload)
        slots = []
        for _, sd in f.get(9, []):
            sf = pb_decode(sd)
            slot_state     = SLOT_STATES.get(_fval(sf, 1), 'unknown').lower()
            filament_state = FILAMENT_STATES.get(_fval(sf, 2), 'unknown').lower()
            if slot_state == 'ready':
                legacy = 'ready'
            elif slot_state in ('feeding', 'assisting'):
                legacy = 'feeding'
            elif slot_state == 'preloading':
                legacy = 'preload'
            elif filament_state == 'empty':
                legacy = 'empty'
            else:
                legacy = slot_state
            slots.append({'status': legacy, 'filament': filament_state})

        dryer = {}
        if 2 in f:
            d = pb_decode(f[2][0][1])
            dryer = {
                'state':    DRY_STATES.get(_fval(d, 1), 'unknown'),
                'target':   _fval(d, 2),
                'duration': _fval(d, 3),
                'remain':   _fval(d, 4),
            }

        return {
            'slots':    slots,
            'temp':     _fval(f, 3),
            'humidity': _fval(f, 4),
            'dryer':    dryer,
        }

    def _parse_filament_info_response(self, payload: bytes) -> dict:
        f = pb_decode(payload)
        result = {}
        if 1 in f: result['index']     = _fval(f, 1)
        if 2 in f: result['sku']       = f[2][0][1].decode('utf-8', errors='ignore')
        if 3 in f: result['brand']       = f[3][0][1].decode('utf-8', errors='ignore')
        if 4 in f: result['type']      = f[4][0][1].decode('utf-8', errors='ignore')
        #if 3 in f: result['sku']       = f[3][0][1].decode('utf-8', errors='ignore')
        #if 4 in f: result['type']      = f[4][0][1].decode('utf-8', errors='ignore')
        if 8 in f: result['diameter']  = _fval(f, 8)
        if 11 in f: result['total']    = _fval(f, 11)
        if 12 in f: result['rfid']     = _fval(f, 12)
        return result

    def _check_auto_feed(self):
        slots = self._info.get('slots', [])
        if not slots:
            return
        ace_is_busy = any(s.get('status') in ('feeding', 'preload') for s in slots)

        for i in range(min(len(slots), 4)):
            current_status = slots[i].get('status')

            if current_status == 'empty':
                if self._last_filament_status[i] != 'empty':
                    self.clear_slot_rfid_info(i)
                    logging.info("ACE: Slot %d empty.", i)
                self.auto_feed_step[i] = 0
                self.feed_sent[i] = False
                self._last_filament_status[i] = 'empty'
                continue

            if self.auto_feed_step[i] == 0:
                prev_status = self._last_filament_status[i]
                if prev_status == 'preload' and current_status == 'ready':
                    if not self.enable_feeder_mode:
                        self.auto_feed_step[i] = 2
                        self.feed_sent[i] = True
                    else:
                        logging.info("ACE: Slot %d trigger. Entering Incremental Feed", i)
                        self.auto_feed_step[i] = 1
                        self.feed_sent[i] = False

            elif self.auto_feed_step[i] == 1:
                if self.port_sensor_hit[i]:
                    logging.info("ACE: Slot %d SENSOR HIT! Moving to Seating", i)
                    self.send_request(
                        cmd=Cmd.STOP_FEED_OR_ROLLBACK,
                        payload=pb_uint32(1, i),
                        callback=None)
                    self.auto_feed_step[i] = 2
                    self.feed_sent[i] = False

                elif self.feed_sent[i] and current_status == 'ready' and not ace_is_busy:
                    logging.warning("ACE: Slot %d move completed. Advancing to seating.", i)
                    self.auto_feed_step[i] = 2
                    self.feed_sent[i] = False

                elif current_status == 'ready' and not self.feed_sent[i] and not ace_is_busy:
                    logging.info("ACE: Slot %d - Motor is free. Sending feed command.", i)
                    self.send_request(
                        cmd=Cmd.FEED_OR_ROLLBACK,
                        payload=(pb_uint32(1, i)
                                 + pb_uint32(2, self.feed_speed)
                                 + pb_uint32(3, self.feed_lengths[i])
                                 + pb_uint32(4, FEED_MODE_FORWARD)),
                        callback=None)
                    if not self.enable_feeder_mode:
                        self.feed_sent[i] = True
                        ace_is_busy = True

            elif self.auto_feed_step[i] == 2:
                if not ace_is_busy:
                    logging.info("ACE: Slot %d performing final seating move.", i)
                    self._do_seating_move(i)
                    self.auto_feed_step[i] = 3
                    ace_is_busy = True

            elif self.auto_feed_step[i] == 3:
                if current_status == 'ready':
                    self.auto_feed_step[i] = 0

            self._last_filament_status[i] = current_status

    def _do_seating_move(self, i):
        if not self.enable_feeder_mode:
            logging.debug("ACE: Slot %d move finished", i)
            self.set_slot_rfid_info(i)
            return
        if self.port_sensor_hit[i]:
            logging.info(
                "ACE: Slot %d sensor hit. Sending seating move at %d mm/s.", i, self.load_speed)
            self.send_request(
                cmd=Cmd.FEED_OR_ROLLBACK,
                payload=(pb_uint32(1, i)
                         + pb_uint32(2, self.load_speed)
                         + pb_uint32(3, self.load_lengths[i])
                         + pb_uint32(4, FEED_MODE_FORWARD)),
                callback=None)
        else:
            logging.warning(
                "ACE: Slot %d primary move finished but Printer %d sensor is EMPTY.", i, i)

    def _handle_status_update(self, response_payload: bytes):
        self._info = self._parse_status_response(response_payload)
        self._check_auto_feed()

    def _writer(self):
        while self._connected:
            try:
                try:
                    cmd, payload, cb = self._queue.get_nowait()
                except queue.Empty:
                    cmd     = Cmd.GET_STATUS
                    payload = b''
                    cb      = self._handle_status_update

                seq = self._next_seq()
                with self._cb_lock:
                    self._callback_map[seq] = cb
                self._send_packet(cmd, payload, seq)
                time.sleep(0.5)
            except Exception as e:
                logging.error("ACE Writer error: %s", e)
                time.sleep(0.5)

    def _reader(self):
        buf = bytearray()
        while self._connected:
            try:
                chunk = self._serial.read(256)
                if chunk:
                    buf.extend(chunk)
                    while len(buf) > 4:
                        pkt, n = _parse_packet(buf)
                        if n > 0:
                            buf = buf[n:]
                        else:
                            break
                        if pkt and pkt.get('is_resp'):
                            seq = pkt['seq']
                            with self._cb_lock:
                                callback = self._callback_map.pop(seq, None)
                            if callback is not None:
                                callback(pkt['payload'])
            except serial.SerialException as e:
                logging.error("ACE: Serial read error: %s", e)
                self._connected = False
                break
            except Exception as e:
                logging.error("ACE: Unexpected reader error: %s", e)
                time.sleep(0.1)

    def dwell(self, delay):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(delay)

    def set_slot_rfid_info(self, index):
        def _callback(payload: bytes):
            if not payload:
                return

            s = self._parse_filament_info_response(payload)

            if s.get('rfid', 1) != 2:
                return

            sku    = s.get('sku', '')
            f_type = s.get('type', '')
            brand  = s.get('brand', '')
            
            color_list = s.get('color', [0, 0, 0])

            r, g, b = color_list[0], color_list[1], color_list[2]
            rgb_packed  = "%02X%02X%02X" % (r, g, b)
            rgb_ipacked = (r << 16) | (g << 8) | b
            colors_raw  = s.get('colors', [[0, 0, 0, 255]])
            alpha       = colors_raw[0][3] if colors_raw and len(colors_raw[0]) > 3 else 255
            alpha_hex   = "%02X" % alpha
            filament_detect = self.printer.lookup_object('filament_detect', None)

            if brand.lower() == "ac":
                filament_info = f_type.split(" ", 1)
                f_type = filament_info[0]
                brand  = filament_info[1] if len(filament_info) > 1 else ""
                sku    = "Anycubic"

            if self.force_generic or self.check_rfid_status():
                if sku.lower() != "snapmaker":
                    sku = "Generic"

            if sku.lower() != "snapmaker" and brand.lower() == "basic":
                brand = ""

            if filament_detect is None:
                command = (
                    f"SET_PRINT_FILAMENT_CONFIG "
                    f"CONFIG_EXTRUDER={index} "
                    f"VENDOR='{sku}' "
                    f"FILAMENT_TYPE='{f_type}' "
                    f"FILAMENT_SUBTYPE='{brand}' "
                    f"FILAMENT_COLOR_RGBA={rgb_packed}{alpha_hex}")
                self.gcode.run_script_from_command(command)
            else:
                length_map = {330: 1000, 247: 750, 198: 600, 165: 500, 82: 250}
                total_len  = s.get('total')
                info = copy.deepcopy(filament_protocol.FILAMENT_INFO_STRUCT)
                info['VERSION']    = 1
                info['VENDOR']     = sku
                info['MANUFACTURER'] = sku
                info['MAIN_TYPE']  = f_type
                info['SUB_TYPE']   = brand
                info['COLOR_NUMS'] = 1
                info['RGB_1']      = rgb_ipacked
                try:
                    info['ALPHA'] = max(0x00, min(0xFF, int(alpha)))
                except (ValueError, TypeError):
                    info['ALPHA'] = 0xFF
                info['ARGB_COLOR'] = info['ALPHA'] << 24 | info['RGB_1']
                info['OFFICIAL']   = True
                info['LENGTH']     = total_len
                try:
                    info['DIAMETER'] = int(round(float(s.get('diameter', 1.75)) * 100))
                except (ValueError, TypeError):
                    info['DIAMETER'] = 175
                try:
                    info['WEIGHT'] = int(length_map.get(total_len, 1000))
                except (ValueError, TypeError):
                    info['WEIGHT'] = 1000
                try:
                    info['HOTEND_MIN_TEMP'] = int(s.get('extruder_temp', {}).get('min', 190))
                    info['HOTEND_MAX_TEMP'] = int(s.get('extruder_temp', {}).get('max', 220))
                except (ValueError, TypeError):
                    info['HOTEND_MIN_TEMP'] = 0
                    info['HOTEND_MAX_TEMP'] = 0
                try:
                    bed_min_temp = int(s.get('hotbed_temp', {}).get('min', 50))
                    bed_max_temp = int(s.get('hotbed_temp', {}).get('max', 60))
                    info['BED_TEMP'] = bed_min_temp if bed_min_temp > 0 else bed_max_temp
                except (ValueError, TypeError):
                    info['BED_TEMP'] = 0
                info['FIRST_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
                info['OTHER_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
                info['MF_DATE']  = datetime.now().strftime("%Y%m%d")
                info['CARD_UID'] = [random.randint(0, 255) for _ in range(7)]
                filament_detect._filament_info_update(index, info, is_clear=True)

        self.send_request(
            cmd=Cmd.GET_FILAMENT_INFO,
            payload=pb_uint32(1, index),
            callback=_callback)

    def clear_slot_rfid_info(self, index):
        filament_detect = self.printer.lookup_object('filament_detect', None)
        if filament_detect is None:
            command = (
                f"SET_PRINT_FILAMENT_CONFIG CONFIG_EXTRUDER={index} "
                f"VENDOR='' FILAMENT_TYPE='' FILAMENT_SUBTYPE='' FILAMENT_COLOR_RGBA=000000FF")
            self.gcode.run_script_from_command(command)
        else:
            info = copy.deepcopy(filament_protocol.FILAMENT_INFO_STRUCT)
            filament_detect._filament_info_update(index, info, is_clear=True)

    cmd_ACE_START_DRYING_help = 'Start ACE filament dryer'
    def cmd_ACE_START_DRYING(self, gcmd):
        temperature = gcmd.get_int('TEMPERATURE')
        duration    = gcmd.get_int('DURATION', 240)
        if duration <= 0:
            raise gcmd.error('Wrong duration')
        if temperature <= 0 or temperature > self.max_dryer_temp:
            raise gcmd.error('Wrong temperature')
        def callback(payload: bytes):
            f = pb_decode(payload) if payload else {}
            code = _fval(f, 1, 0)
            if code != 0:
                raise gcmd.error("ACE Error code: %d" % code)
            self.gcode.respond_info(
                'Started ACE drying at %d°C for %d minutes' % (temperature, duration))
        self.send_request(
            cmd=Cmd.DRYING,
            payload=(pb_uint32(1, temperature)
                     + pb_uint32(2, duration)
                     + pb_bool(3, True)),
            callback=callback)

    cmd_ACE_STOP_DRYING_help = 'Stop ACE filament dryer'
    def cmd_ACE_STOP_DRYING(self, gcmd):
        def callback(payload: bytes):
            f = pb_decode(payload) if payload else {}
            code = _fval(f, 1, 0)
            if code != 0:
                raise gcmd.error("ACE Error code: %d" % code)
            self.gcode.respond_info('Stopped ACE drying')
        self.send_request(
            cmd=Cmd.DRYING,
            payload=pb_uint32(1, 0) + pb_uint32(2, 0),
            callback=callback)

    cmd_SET_FILAMENT_CONFIG_help = 'Set filament information'
    def cmd_SET_FILAMENT_CONFIG(self, gcmd):
        channel       = gcmd.get_int('CHANNEL')
        vendor        = gcmd.get('VENDOR', '')
        filament_type = gcmd.get('TYPE', '')
        subtype       = gcmd.get('SUBTYPE', '')
        color         = gcmd.get('COLOR', '000000')
        alpha         = gcmd.get('ALPHA', 'FF')
        official      = gcmd.get('OFFICIAL', 'false').lower()
        length        = gcmd.get_int('LENGTH', 330)
        diameter      = gcmd.get_int('DIAMETER', 175)
        weight        = gcmd.get_int('WEIGHT', 1000)
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
            raise gcmd.error("ACE channel{} is out of range[0, 3]".format(channel))

        filament_detect = self.printer.lookup_object('filament_detect', None)
        if filament_detect is None:
            command = (
                f"SET_PRINT_FILAMENT_CONFIG "
                f"CONFIG_EXTRUDER={channel} "
                f"VENDOR='{vendor}' "
                f"FILAMENT_TYPE='{filament_type}' "
                f"FILAMENT_SUBTYPE='{subtype}' "
                f"FILAMENT_COLOR_RGBA={color}{alpha}")
            self.gcode.run_script_from_command(command)
        else:
            info = copy.deepcopy(filament_protocol.FILAMENT_INFO_STRUCT)
            info['VERSION']    = 1
            info['VENDOR']     = vendor
            info['MANUFACTURER'] = vendor
            info['MAIN_TYPE']  = filament_type
            info['SUB_TYPE']   = subtype
            info['COLOR_NUMS'] = 1
            info['RGB_1']      = int(color, 16)
            info['ALPHA']      = int(alpha, 16)
            info['ARGB_COLOR'] = (int(alpha, 16) << 24) | int(color, 16)
            info['OFFICIAL']   = official in ['true', '1', 'yes']
            info['LENGTH']     = length
            info['DIAMETER']   = diameter
            info['WEIGHT']     = weight
            info['HOTEND_MIN_TEMP'] = extruder_temp_min
            info['HOTEND_MAX_TEMP'] = extruder_temp_max
            info['BED_TEMP']   = hotbed_temp_min if hotbed_temp_min > 0 else hotbed_temp_max
            info['FIRST_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
            info['OTHER_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
            info['MF_DATE']    = datetime.now().strftime("%Y%m%d")
            info['CARD_UID']   = [random.randint(0, 255) for _ in range(7)]
            filament_detect._filament_info_update(channel, info, is_clear=True)
            self.gcode.respond_info(str(info))


def load_config(config):
    return AceDevice(config)
