# https://github.com/DnG-Crafts/U1-Ace
import serial, threading, time, logging, json, struct, queue, traceback, glob, copy, random, configparser
from datetime import datetime
from . import filament_protocol
from . import ace2_protocol

ACE1_GLOB = '/dev/serial/by-id/usb-ANYCUBIC*'
ACE2_GLOB = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_*'
ACE1_DEFAULT_SERIAL = '/dev/serial/by-id/usb-ANYCUBIC_ACE_1-if00'
ACE2_DEFAULT_SERIAL = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B5F070433-if00'

# Slot status strings the v2 backend emits for hard error codes
# (FEED_ERROR / ROLLBACK_ERROR / PRELOAD_ERROR / STUCK_ERROR /
# TANGLED_ERROR / MOTOR_ERROR). 'error' is kept as a generic catch-all.
# assist_error is intentionally excluded: it is a transient ACE watchdog
# timeout handled by _handle_assist_error (log + re-arm, no print abort).
ACE_ERROR_STATUSES = {
    'feed_error', 'rollback_error', 'preload_error',
    'stuck', 'tangled', 'motor_error', 'error',
}

# Slot statuses that are mutually exclusive with FEED_ASSIST / UNWIND_ASSIST
# mode. If the device reports one of these while our _assist_active_slots
# claims the slot is still assisting, our local state is stale and must be
# cleared so the watchdog can re-arm.
_STATES_INCOMPATIBLE_WITH_ASSIST = {
    'ready', 'feeding', 'unwinding', 'preload', 'empty',
}


def _guess_version_from_path(path):
    if not path:
        return None
    if 'ANYCUBIC' in path:
        return 'ace1'
    if '1a86' in path or 'wch' in path.lower():
        return 'ace2'
    return None


class AceDevice:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.add_object('ace_device', self)
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        # device_version: 'auto' | 'ace1' | 'ace2'. Auto picks whichever
        # of the two USB IDs actually shows up under /dev/serial/by-id.
        self.device_version = config.get('device_version', 'auto').lower()
        if self.device_version not in ('auto', 'ace1', 'ace2'):
            logging.warning("ACE: invalid device_version '%s', using auto",
                            self.device_version)
            self.device_version = 'auto'

        configured_serial = config.get(
            'serial', '/dev/serial/by-id/usb-ANYCUBIC_ACE_1-if00')
        ace1_devices = glob.glob(ACE1_GLOB)
        ace2_devices = glob.glob(ACE2_GLOB)

        # noinspection PyUnusedLocal
        detected = None
        if self.device_version == 'ace1':
            detected = 'ace1'
            self.serial_name = ace1_devices[0] if ace1_devices else configured_serial
        elif self.device_version == 'ace2':
            detected = 'ace2'
            self.serial_name = ace2_devices[0] if ace2_devices else configured_serial
        else:  # auto
            if ace1_devices:
                detected = 'ace1'
                self.serial_name = ace1_devices[0]
            elif ace2_devices:
                detected = 'ace2'
                self.serial_name = ace2_devices[0]
            else:
                self.serial_name = configured_serial
                detected = _guess_version_from_path(self.serial_name) or 'ace1'

        self._is_v2 = (detected == 'ace2')
        logging.info("ACE: detected %s on %s",
                     'ACE 2 Pro' if self._is_v2 else 'ACE 1 Pro',
                     self.serial_name)

        # Baud: honor explicit config, but auto-upgrade 115200 -> 230400
        # when we detected an ACE 2 so existing configs keep working.
        configured_baud = config.getint('baud', 115200)
        if self._is_v2 and configured_baud == 115200:
            self.baud = 230400
            logging.info("ACE: overriding baud 115200 -> 230400 for ACE 2 Pro")
        else:
            self.baud = configured_baud
        self.feed_speed = config.getint('feed_speed', 90)
        self.load_speed = config.getint('load_speed', 100)
        self.retract_speed = config.getint('retract_speed', 100)
        self.feed_check_len = config.getint('feed_check_len', 100)
        self.feed_check_error_len = config.getint('feed_check_error_len', 90)
        self.max_dryer_temp = config.getint('max_dryer_temperature', 55)
        self.enable_feed_assist = config.getboolean('enable_feed_assist', True)
        # Direction-confirmation window: how long the opposite direction
        # must be sustained before we actually flip mode. Brief slicer
        # retracts (typically 50-250 ms) get filtered out and never
        # generate stop+start packets; only sustained reversals (real
        # unloads, manual long retracts) commit. The pending switch
        # is cleared if velocity returns to the original direction
        # before the window elapses.
        self.assist_mode_confirm_time = config.getfloat(
            'assist_mode_confirm_time', 1.0)
        self.assist_verbose_logging = config.getboolean(
            'assist_verbose_logging', False)
        # Stop feed assist after this many seconds without any commanded
        # extrusion. The watchdog re-arms it the moment the next move pushes
        # E forward, so this only matters between prints / during long pauses.
        # Stop assist this many seconds after live_extruder_velocity drops
        # below the threshold. ACE 2 emits ASSIST_ERROR ~1 s after the
        # toolhead stops pulling -- the motor keeps pushing filament into
        # the tube until it buckles -- so we have to stop assist faster
        # than that. 3 s is a balance: short enough to disarm before the
        # device errors, long enough to span typical layer-change pauses.
        self.feed_assist_idle_timeout = config.getfloat(
            'feed_assist_idle_timeout', 5.0)
        self.enable_feeder_mode = config.getboolean('enable_feeder_mode', False)
        self.disable_u1_rfid = config.getboolean('disable_u1_rfid', False)
        self.force_generic = config.getboolean('force_generic', False)
        

        
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
            config.getint('retract_length_slot1', 3000),
            config.getint('retract_length_slot2', 3000),
            config.getint('retract_length_slot3', 3000),
            config.getint('retract_length_slot4', 3000)
        ]


        self._connected = False
        self._serial = None
        self._request_id = 0
        self._queue = queue.Queue()
        self._callback_map = {}
        # v2 transport state (ACE 2 Pro)
        self._v2_seq = 0
        self._v2_callback_map = {}   # seq -> (callback, response_decoder)
        self._v2_rx_buf = bytearray()
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
        self._printing = False
        self._pending_start_index = -1
        self._next_cmd_time = 0
        # _assist_active_slots tracks which slots we have commanded into
        # FEED_ASSIST or UNWIND_ASSIST mode. The motion_report-driven
        # _assist_loop_eval uses it to avoid re-issuing start_*_assist on
        # every tick.
        self._assist_active_slots = set()
        # Per-slot assist mode tracking. 'feed' = FEED_ASSIST (mode 2),
        # 'unwind' = UNWIND_ASSIST (mode 3).
        self._assist_mode_per_slot = {}    # slot -> 'feed' | 'unwind'
        # Pending mode-switch confirmation: slot -> {'target', 'since'}.
        # Populated when _ensure_assist_mode wants to flip; only commits
        # the flip after the new direction has been sustained for
        # assist_mode_confirm_time seconds. Cleared when the slot's
        # observed direction returns to the current mode (e.g. after a
        # slicer retract un-retracts) or when assist is stopped.
        self._pending_mode_switch = {}
        # Per-slot idle-tracking: slot -> last eventtime we observed extruder
        # motion above the velocity threshold. After feed_assist_idle_timeout
        # seconds of quiet on a slot, _assist_loop_eval stops its assist.
        self._last_extrusion_times = {}
        # Slots that currently have a single long ROLLBACK in flight. Cleared
        # by _handle_status_update when the slot transitions to empty/ready.
        self._retract_in_progress = set()

        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('klippy:disconnect', self._handle_disconnect)
        self.printer.register_event_handler("filament_feed:port", self._feed_handler)
        self.printer.register_event_handler('print_stats:start', self._handle_start_print_job)
        self.printer.register_event_handler('print_stats:stop', self._handle_stop_print_job)
        self.printer.register_event_handler('idle_timeout:idle', self._handle_not_printing)
 
        self.gcode.register_command('ACE_START_DRYING', self.cmd_ACE_START_DRYING)
        self.gcode.register_command('ACE_STOP_DRYING', self.cmd_ACE_STOP_DRYING)
        self.gcode.register_command('ACE_GET_TEMP', self.cmd_ACE_GET_TEMP)
        self.gcode.register_command('ACE_GET_STATUS', self.cmd_ACE_GET_STATUS)
        self.gcode.register_command('SET_FILAMENT_CONFIG', self.cmd_SET_FILAMENT_CONFIG)


    def feeder_mode(self):
        return self.enable_feeder_mode

    def feed_assist(self):
        return self.enable_feed_assist

    def disable_rfid(self):
        return self.disable_u1_rfid


    def check_rfid_status(self):
        config = configparser.ConfigParser()
        try:
            config.read('/oem/printer_data/config/extended/extended2.cfg')
            rfid_value = config.get('components', 'rfid', fallback=None)
            if rfid_value == "openrfid-generic":
                return True
            else:
                return False
        except Exception:
            return False


    def _handle_start_print_job(self):
        logging.info("ACE: Print job started.")
        self._printing = True
        self._last_active_index = -1
        self._last_active_tool = None
        # Reset velocity-loop idle tracking so the new job starts fresh.
        self._last_extrusion_times.clear()



    def _handle_not_printing(self, _eventtime):
        logging.info("ACE: Printer is now IDLE/Ready.")
        self._printing = False
        if self._last_active_index != -1 and self.enable_feed_assist:
            for i in range(4):
                self._stop_feed_assist(i, why='printer_idle')
        self._last_active_index = -1
        self._last_active_tool = None
 

    def _handle_stop_print_job(self):
        logging.info("ACE: Print job stopped/completed.")
        self._printing = False
        if self._last_active_index != -1 and self.enable_feed_assist:
            for i in range(4):
                self._stop_feed_assist(i, why='print_stopped')
        self._last_active_index = -1
        self._last_active_tool = None


    # ---- Assist helpers (always log; callback surfaces device ack/err) ----

    def _assist_log_cb(self, label, idx):
        def _cb(_ace_ref, resp):
            if not resp:
                return
            code = resp.get('code')
            if code:
                logging.warning(
                    "ACE: %s slot=%d FAILED code=%s msg=%s",
                    label, idx, code, resp.get('msg', ''))
            else:
                logging.info("ACE: %s slot=%d ack", label, idx)
        return _cb

    def _start_feed_assist(self, idx, why=''):
        logging.info("ACE: -> start_feed_assist slot=%d (%s)", idx, why)
        self._assist_active_slots.add(idx)
        self._assist_mode_per_slot[idx] = 'feed'
        def _cb(_ace_ref, resp):
            if not resp:
                return
            code = resp.get('code')
            if code:
                logging.warning(
                    "ACE: start_feed_assist slot=%d FAILED code=%s msg=%s",
                    idx, code, resp.get('msg', ''))
            else:
                logging.info("ACE: start_feed_assist slot=%d ack", idx)
        self.send_request(
            request={'method': 'start_feed_assist', 'params': {'index': idx}},
            callback=_cb)

    def _stop_feed_assist(self, idx, why=''):
        logging.info("ACE: -> stop_feed_assist slot=%d (%s)", idx, why)
        self._clear_assist_tracking(idx)
        self.send_request(
            request={'method': 'stop_feed_assist', 'params': {'index': idx}},
            callback=self._assist_log_cb('stop_feed_assist', idx))


    def _clear_assist_tracking(self, idx):
        """Drop local feed-assist bookkeeping for a slot without issuing
        another stop_feed_assist on the wire. Use this when something we
        already sent -- stop_feed_filament, feed_filament, unwind_filament,
        or an observed device-side mode change -- has implicitly cancelled
        assist mode for the slot. stop_feed_filament and stop_feed_assist
        share the same CMD_STOP_FEED_OR_ROLLBACK opcode, so any
        stop_feed_filament also stops assist on the device."""
        self._assist_active_slots.discard(idx)
        self._assist_mode_per_slot.pop(idx, None)
        self._pending_mode_switch.pop(idx, None)
        self._last_extrusion_times.pop(idx, None)


    def _start_unwind_assist(self, idx, why=''):
        logging.info("ACE: -> start_unwind_assist slot=%d (%s)", idx, why)
        self._assist_active_slots.add(idx)
        self._assist_mode_per_slot[idx] = 'unwind'
        def _cb(_ace_ref, resp):
            if not resp:
                return
            code = resp.get('code')
            if code:
                logging.warning(
                    "ACE: unwind_assist slot=%d FAILED code=%s msg=%s",
                    idx, code, resp.get('msg', ''))
            else:
                logging.info("ACE: unwind_assist slot=%d ack", idx)
        self.send_request(
            request={'method': 'unwind_assist', 'params': {'index': idx}},
            callback=_cb)


    def _resolve_active_extruder_index(self, eventtime):
        """Best-effort: figure out which extruder index (0-3) is active."""
        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is None:
            return None
        try:
            active_name = toolhead.get_status(eventtime).get('extruder', '')
        except Exception:
            return None
        if not active_name or not active_name.startswith('extruder'):
            return None
        suffix = active_name[8:]
        if suffix == '':
            return 0
        if suffix.isdigit():
            return int(suffix)
        return None


    # Below this absolute velocity (mm/s) we treat motion_report as
    # "not moving". Anything below is floating-point noise / pressure-
    # advance trail-off rather than a real extrusion command. The
    # watchdog will not arm or update the loop's last-extrusion time
    # for sub-threshold velocities.
    _VELOCITY_IDLE_THRESHOLD = 0.001


    # filament_feed stages that count as "in-flight load/unload activity"
    # for the per-slot _filament_feed_active() check. Anything starting
    # with these prefixes AND not ending in _finish/_fail is active.
    _FF_ACTIVE_PREFIXES = ('load_', 'unload_', 'preload_', 'manual_sta_')


    def _filament_feed_active(self, idx=None):
        """True if filament_feed has an in-progress load / unload /
        preload / manual operation.

        When ``idx`` is None, returns True if ANY channel is active.
        When ``idx`` is a slot index (0-3), returns True only if the
        channel mapped to that extruder is active. The per-slot form
        is used by the idle-timeout check so that, e.g., a transient
        port-sensor blip on slot 2 doesn't prevent the timeout from
        stopping a stale assist on slot 1.

        Terminal stages (*_finish, *_fail) and idle stages (none,
        inited, wait_insert, test) do NOT count as active.
        """
        fd = self.printer.lookup_object('filament_detect', None)
        if fd is None:
            return False
        feed_objs = getattr(fd, 'filament_feed_objects', None) or []
        for _, f_obj in feed_objs:
            states = getattr(f_obj, 'channel_state', None) or []
            ch_count = len(states)
            for ch in range(ch_count):
                try:
                    if (idx is not None
                            and f_obj.filament_ch[ch] != idx):
                        continue
                    stage_str = str(states[ch])
                except Exception:
                    continue
                if not any(stage_str.startswith(p)
                           for p in self._FF_ACTIVE_PREFIXES):
                    continue
                if (stage_str.endswith('_finish')
                        or stage_str.endswith('_fail')):
                    continue
                return True
        return False


    def _calc_crc(self, buffer):
        _crc = 0xffff
        for byte in buffer:
            data = byte
            data ^= _crc & 0xff
            data ^= (data & 0x0f) << 4
            _crc = ((data << 8) | (_crc >> 8)) ^ (data >> 4) ^ (data << 3)
        return _crc & 0xffff


    def _send_request(self, request):
        if not self._connected or not self._serial:
            return
        try:
            payload = json.dumps(request).encode('utf-8')
            data = b"\xFF\xAA" + struct.pack('@H', len(payload)) + payload + \
                   struct.pack('@H', self._calc_crc(payload)) + b"\xFE"
            self._serial.write(data)
        except Exception as e:
            logging.error("ACE: Serial write failed, disconnecting: %s" % str(e))
            self._connected = False


    def _next_v2_seq(self):
        self._v2_seq = (self._v2_seq % 0xFFFE) + 1
        return self._v2_seq


    def _send_request_v2(self, request, callback):
        """Send a v1-style request through the ACE 2 Pro protobuf transport.

        The callback, if set, receives a v1-shaped response dict so that
        code paths shared with ACE 1 (e.g. _handle_status_update, the drying
        callbacks, set_slot_rfid_info) do not need to change.
        """
        if not self._connected or not self._serial:
            return
        encoded = ace2_protocol.encode_v1_request(request)
        if encoded is None:
            return
        cmd, payload, decoder = encoded
        seq = self._next_v2_seq()
        self._v2_callback_map[seq] = (callback, decoder)
        # Hex-dump every feed/assist command so we can see exactly what hits
        # the wire when something goes wrong. Status polls are skipped to
        # avoid drowning the log.
        if cmd in (ace2_protocol.CMD_FEED_OR_ROLLBACK,
                   ace2_protocol.CMD_STOP_FEED_OR_ROLLBACK):
            logging.info(
                "ACE2: tx cmd=0x%02X seq=%d method=%s params=%s payload=%s",
                cmd, seq, request.get('method'), request.get('params'),
                payload.hex() or '<empty>')
        try:
            self._serial.write(
                ace2_protocol.build_packet(cmd, payload, seq=seq))
        except Exception as e:
            logging.error("ACE: v2 serial write failed, disconnecting: %s",
                          str(e))
            self._v2_callback_map.pop(seq, None)
            self._connected = False


    def _v2_handshake(self):
        """Optional ACE 2 Pro discover + assign-device-id before polling.

        The ACE 2 Pro happily answers status requests without assignment in
        single-unit setups, but the official driver always discovers +
        assigns device id 1 first, so we mirror that to stay on the
        well-tested path. Failures here are non-fatal.
        """
        try:
            self._serial.reset_input_buffer()
            self._serial.write(ace2_protocol.build_discover_packet(seq=1))
            self._serial.flush()
            buf = bytearray()
            deadline = time.time() + 1.0
            uids = None
            while time.time() < deadline and uids is None:
                n = self._serial.in_waiting
                if n:
                    buf.extend(self._serial.read(n))
                    packets, buf = ace2_protocol.parse_stream(buf)
                    for p in packets:
                        if (p['is_resp']
                                and p['cmd'] == ace2_protocol.CMD_DISCOVER_DEVICE
                                and p['payload']):
                            uids = ace2_protocol.parse_discover_response(
                                p['payload'])
                            break
                else:
                    time.sleep(0.02)
            if uids is not None:
                logging.info(
                    "ACE: ACE 2 discovered uid 0x%08X-0x%08X-0x%08X",
                    uids[0], uids[1], uids[2])
                self._serial.write(ace2_protocol.build_assign_id_packet(
                    uids[0], uids[1], uids[2], dev_id=1, seq=2))
                self._serial.flush()
                time.sleep(0.2)
            else:
                logging.info(
                    "ACE: no ACE 2 discover response; proceeding anyway")
            self._serial.reset_input_buffer()
        except Exception as e:
            logging.warning("ACE: v2 handshake failed (continuing): %s", e)


    def send_request(self, request, callback):
        self._queue.put([request, callback])


    def _handle_ready(self):
        self.connection_timer = self.reactor.register_timer(
            self._connection_timer, self.reactor.NOW)
        logging.info("ACE: Connection monitor started")
        # Velocity-driven assist control loop.
        self._assist_loop_timer = self.reactor.register_timer(
            self._assist_loop_eval, self.reactor.NOW)


    def _get_extruder_velocity(self):
        """Return current commanded extruder velocity in mm/s.

        Sourced from motion_report's live_extruder_velocity, which is the
        instantaneous interpolated rate including pressure advance and any
        other extruder-axis modifiers. Sign is meaningful: positive =
        extrusion, negative = retract. Returns 0.0 if motion_report is
        unavailable or raises.
        """
        motion_report = self.printer.lookup_object('motion_report', None)
        if motion_report is None:
            return 0.0
        try:
            status = motion_report.get_status(self.reactor.monotonic())
        except Exception as e:
            logging.error("ACE: motion_report.get_status failed: %s", e)
            return 0.0
        v = status.get('live_extruder_velocity', 0.0)
        try:
            return float(v) if v is not None else 0.0
        except (TypeError, ValueError):
            return 0.0


    def _assist_loop_eval(self, eventtime):
        """Velocity-driven feed-assist control loop.

        Each tick:
          1. Read motion_report.live_extruder_velocity (signed mm/s).
          2. Resolve the active extruder slot.
          3. Decide target mode (feed when v>0, unwind when v<0); below
             the velocity threshold we leave assist alone and stop it
             after feed_assist_idle_timeout.
          4. Make sure the slot is in the target mode (with throttle so
             slicer retracts don't flip the mode every segment).
        """
        if not self._connected or not self.enable_feed_assist:
            return eventtime + 0.25

        velocity = self._get_extruder_velocity()
        abs_vel = abs(velocity)
        idx = self._resolve_active_extruder_index(eventtime)

        if idx is None or idx < 0 or idx > 3:
            return eventtime + 0.25

        # Per-slot idle timeout: check every tick so each slot times out on
        # its own clock regardless of which extruder is currently active.
        for s in list(self._assist_active_slots):
            last_t = self._last_extrusion_times.get(s, 0.0)
            if (not self._printing and last_t > 0
                    and eventtime - last_t > self.feed_assist_idle_timeout
                    and not self._filament_feed_active(s)):
                self._stop_feed_assist(
                    s, why='no_extrusion_%ds' % int(self.feed_assist_idle_timeout))

        if abs_vel < self._VELOCITY_IDLE_THRESHOLD:
            # If a direction-flip was pending, the blip that triggered
            # it has clearly resolved -- cancel it so it can't confirm
            # on the next non-zero tick that happens to be in the same
            # direction (but may be unrelated).
            if idx in self._pending_mode_switch:
                pending = self._pending_mode_switch.pop(idx)
                if self.assist_verbose_logging:
                    logging.info(
                        "ACE: pending switch slot=%d %s -> %s cancelled "
                        "(velocity dropped to zero after %.2fs)",
                        idx,
                        self._assist_mode_per_slot.get(idx),
                        pending['target'],
                        eventtime - pending['since'])
            return eventtime + 0.25


        self._last_extrusion_times[idx] = eventtime
        target_mode = 'feed' if velocity > 0 else 'unwind'
        # Unwind assist is suppressed during an active print job.
        if target_mode == 'unwind' and self._printing:
            target_mode = 'feed'

        self._ensure_assist_mode(idx, target_mode, abs_vel, eventtime)

        return eventtime + 0.25


    def _ensure_assist_mode(self, idx, target, target_velocity, eventtime):
        """Make sure slot ``idx`` is currently asserting in ``target`` mode
        ('feed' or 'unwind').

        Once we've sent start_*_assist and got an ack, we trust the device
        until either:
          - we see the slot enter an explicit error state (handled by
            _handle_slot_error elsewhere -- it stops the assist and pulls
            the slot out of _assist_active_slots, which makes the next
            tick re-arm cleanly), or
          - we want to flip to the other direction (feed <-> unwind).
        We do NOT re-arm just because the device reports the slot as
        'ready'. The ACE 2 toggles slot status 'assisting' <-> 'ready'
        depending on whether the toolhead is actively pulling on the
        filament right now -- 'ready' here means "armed but idle", and
        re-arming on that fires start_*_assist 4x/sec which the device
        eventually rejects with FORBIDDEN (code=2).
        """
        slots = (self._info or {}).get('slots') or []
        slot_status = (slots[idx].get('status')
                       if idx < len(slots) else None)
        # Don't touch assist while the device has an explicit feed/rollback
        # in flight, and don't try to assist an empty slot. Errors are
        # handled by _handle_slot_error -- nothing to do here.
        if slot_status in ('feeding', 'rollback', 'empty'):
            return
        if slot_status in ACE_ERROR_STATUSES:
            return

        current_mode = self._assist_mode_per_slot.get(idx)
        in_set = idx in self._assist_active_slots

        if in_set and current_mode == target:
            # Already armed in the right mode. If we had a pending
            # opposite-direction switch in flight (slicer retract
            # blip), the velocity has returned to the current mode,
            # so abandon it -- no flip needed.
            if idx in self._pending_mode_switch:
                if self.assist_verbose_logging:
                    pending = self._pending_mode_switch[idx]
                    held = eventtime - pending['since']
                    logging.info(
                        "ACE: pending switch slot=%d %s -> %s "
                        "abandoned after %.2fs (direction returned to "
                        "%s before confirm window of %.2fs elapsed)",
                        idx, current_mode, pending['target'], held,
                        current_mode, self.assist_mode_confirm_time)
                self._pending_mode_switch.pop(idx, None)
            return

        if in_set and current_mode is not None and current_mode != target:
            # Wrong mode -- normally we'd flip, but first gate on the
            # direction-confirmation window so brief reversals (slicer
            # retracts) don't generate spurious stop+start packets.
            pending = self._pending_mode_switch.get(idx)
            if pending is None or pending['target'] != target:
                # Start (or restart with the new target) the pending
                # timer. Don't flip yet -- wait for confirmation.
                self._pending_mode_switch[idx] = {
                    'target': target,
                    'since': eventtime,
                }
                if self.assist_verbose_logging:
                    logging.info(
                        "ACE: pending switch slot=%d %s -> %s started "
                        "(velocity=%.1f mm/s, will commit after %.2fs)",
                        idx, current_mode, target, target_velocity,
                        self.assist_mode_confirm_time)
                return
            held = eventtime - pending['since']
            if held < self.assist_mode_confirm_time:
                # Still inside the confirmation window; keep waiting.
                return
            logging.info(
                "ACE: pending switch slot=%d %s -> %s confirmed "
                "after %.2fs; flipping",
                idx, current_mode, target, held)
            self._stop_feed_assist(
                idx, why='switch_%s_to_%s' % (current_mode, target))
            self._pending_mode_switch.pop(idx, None)

        if target == 'feed':
            self._start_feed_assist(
                idx, why='loop_v=+%.1fmms' % target_velocity)
        else:
            self._start_unwind_assist(
                idx, why='loop_v=-%.1fmms' % target_velocity)


    def _start_retract(self, idx):
        """Issue a single long ROLLBACK for slot ``idx``.

        Length comes from retract_length_slot* (default 3 m -- enough to
        clear any reasonable PTFE tube in one shot). Completion is inferred
        in _handle_status_update when the slot transitions to empty/ready.
        """
        logging.info(
            "ACE: starting retract on slot %d (%dmm @ %dmm/s)",
            idx, self.retract_lengths[idx], self.retract_speed)
        self._retract_in_progress.add(idx)
        self.send_request(
            request={'method': 'unwind_filament',
                     'params': {'index': idx,
                                'length': self.retract_lengths[idx],
                                'speed': self.retract_speed}},
            callback=None)
        self._clear_assist_tracking(idx)


    def _connection_timer(self, eventtime):
        if not self._connected:
            self._attempt_connection()
        return eventtime + (5.0 if not self._connected else 20.0)


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
                is_detected = status.get('filament_detected', False)
                self.port_sensor_hit[i] = is_detected
            else:
                self.port_sensor_hit[i] = False


    def get_sensor_state(self, index, eventtime):
        sensor_name = 'filament_motion_sensor e%d_filament' % index
        s_obj = self.printer.lookup_object(sensor_name, None)
        if s_obj is not None:
            status = s_obj.get_status(eventtime)
            is_detected = status.get('filament_detected', False)
            return is_detected
        else:
            return False


    def _attempt_connection(self):
        try:
            if self._serial:
                try: self._serial.close()
                except: pass

            # If device_version is forced to 'ace2' but serial still points at
            # the ACE 1 default (user left the config untouched), auto-correct
            # to the ACE 2 path so the connection actually succeeds.
            if (self.device_version == 'ace2'
                    and self.serial_name == ACE1_DEFAULT_SERIAL):
                ace2_devices = glob.glob(ACE2_GLOB)
                self.serial_name = ace2_devices[0] if ace2_devices else ACE2_DEFAULT_SERIAL
                logging.info(
                    "ACE: device_version=ace2 with ACE 1 default serial; "
                    "auto-corrected to %s", self.serial_name)

            self._serial = serial.Serial(port=self.serial_name, baudrate=self.baud, timeout=0.2)
            if self._serial.isOpen():
                self._connected = True
                # The device forgets any in-flight assist on a reconnect, so
                # drop our local belief that those slots are still assisting.
                self._assist_active_slots.clear()
                self._assist_mode_per_slot.clear()
                self._pending_mode_switch.clear()
                self._last_extrusion_times.clear()
                self._retract_in_progress.clear()
                if self._is_v2:
                    self._v2_rx_buf = bytearray()
                    self._v2_callback_map = {}
                    self._v2_seq = 0
                    # Run the ACE 2 handshake synchronously before the reader
                    # and writer threads start, so neither of them contends
                    # with it on the serial port.
                    self._v2_handshake()
                    self.send_request(
                        request={'method': 'set_feed_check',
                                 'params': {'check_len': self.feed_check_len,
                                            'error_len': self.feed_check_error_len}},
                        callback=None)

                if not hasattr(self, '_writer_thread') or not self._writer_thread.is_alive():
                    self._writer_thread = threading.Thread(target=self._writer)
                    self._writer_thread.setDaemon(True)
                    self._writer_thread.start()
                
                if not hasattr(self, '_reader_thread') or not self._reader_thread.is_alive():
                    self._reader_thread = threading.Thread(target=self._reader)
                    self._reader_thread.setDaemon(True)
                    self._reader_thread.start()

                self.main_timer = self.reactor.register_timer(self._main_eval, self.reactor.NOW)
                logging.info("ACE: Connected successfully")
        except Exception as e:
            self._connected = False
            logging.error("ACE: Connection attempt failed: %s" % str(e))


    def _handle_disconnect(self):
        self._connected = False
        if self._serial:
            try: self._serial.close()
            except: pass
        self._serial = None
        try: self.reactor.unregister_timer(self.main_timer)
        except: pass


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

            if current_state == "printing" and (current_layer > 0 or progress > 0.00):
                # Track the active extruder so other handlers
                # (_handle_not_printing, _handle_stop_print_job) know which
                # slot was last used. Assist itself is driven by the
                # velocity loop, not by swap detection.
                toolhead = self.printer.lookup_object('toolhead', None)
                if toolhead:
                    th_status = toolhead.get_status(eventtime)
                    active_name = th_status.get('extruder')
                    new_idx = 0
                    if active_name and active_name.startswith('extruder'):
                        num_part = active_name[8:]
                        new_idx = int(num_part) if num_part.isdigit() else 0
                    if active_name != self._last_active_tool:
                        logging.info(
                            "ACE: Active extruder %s (Idx %s) -> %s (Idx %s)",
                            self._last_active_tool,
                            self._last_active_index,
                            active_name, new_idx)
                        self._last_active_tool = active_name
                        self._last_active_index = new_idx
                        if self.enable_feed_assist:
                            for s in list(self._assist_active_slots):
                                if s != new_idx:
                                    self._stop_feed_assist(
                                        s, why='extruder_switch_to_%d' % new_idx)
                            if new_idx not in self._assist_active_slots:
                                self._start_feed_assist(
                                    new_idx, why='extruder_switch')

                return eventtime + 0.25



        if not self.enable_feed_assist or not self.enable_feeder_mode:
            feeds_to_remove = []
            for idx in self._active_feeds:
                if self.get_sensor_state(idx, eventtime):
                    logging.info("ACE: sensor hit, slot %d" % idx)
                    self.send_request(request={"method": "stop_feed_filament", "params": {"index": idx}}, callback=None)
                    self._clear_assist_tracking(idx)
                    feeds_to_remove.append(idx)
                elif eventtime > self._feed_start_times.get(idx, 0) + self._timeout_threshold:
                    logging.warning("ACE: Feed slot %d TIMEOUT. Stopping." % idx)
                    self.send_request(request={"method": "stop_feed_filament", "params": {"index": idx}}, callback=None)
                    self._clear_assist_tracking(idx)
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
            
            if not hasattr(self, '_processed_extruders'):
                self._processed_extruders = set()
                self._last_f_stages = {}
                # Per-channel snapshot of the stage we observed
                # immediately BEFORE the current one. Used to detect
                # races like the U1's port event handler overwriting
                # UNLOAD_FINISH with PRELOAD_FINISH faster than our
                # _main_eval poll catches it (see the unload_doing ->
                # preload_finish branch below).
                self._prev_channel_stages = {}
                self._last_action = action_code
                self._allow_triggers = False
                logging.info("ACE: System initialized. Triggers locked until action change.")

            if action_code != self._last_action:
                logging.info("ACE: Action changed to %s.", action_code)
                self._allow_triggers = True
                self._last_action = action_code

            for obj_name, f_obj in getattr(fd, 'filament_feed_objects', []):
                for ch in range(2):
                    actual_stage = str(f_obj.channel_state[ch])
                    assigned_extruder = f_obj.filament_ch[ch]
                    state_key = f"{obj_name}_{ch}"

                    last_stage = self._last_f_stages.get(state_key)
                    if actual_stage != last_stage:
                        logging.info(
                            "ACE: %s Ch %d | Stage: %s -> %s | Extruder: %s",
                            obj_name, ch, last_stage, actual_stage,
                            assigned_extruder)
                        if assigned_extruder is not None:
                            old_gate_key = f"{int(assigned_extruder)}_{last_stage}"
                            if old_gate_key in self._processed_extruders:
                                self._processed_extruders.remove(old_gate_key)

                        # Snapshot the prior stage so the action handler
                        # below can detect cross-stage races such as
                        # unload_doing -> preload_finish.
                        self._prev_channel_stages[state_key] = last_stage
                        self._last_f_stages[state_key] = actual_stage
                    is_preload = actual_stage in ["preload_finish"]
                    if assigned_extruder is not None and (self._allow_triggers or is_preload):
                        idx = int(assigned_extruder)
                        gate_key = f"{idx}_{actual_stage}"

                        if gate_key not in self._processed_extruders:
                            if actual_stage == "unload_finish":
                                logging.info("ACE: [SERIAL] -> UNWIND_SLOT=%d", idx)
                                # Toolhead-side unwind is done;
                                # fire one long ACE-driven retract.
                                if idx in self._assist_active_slots:
                                    self._stop_feed_assist(
                                        idx, why='unload_finish')
                                self._start_retract(idx)
                                self._processed_extruders.add(gate_key)

                            elif actual_stage == "unload_fail":
                                logging.info(
                                    "ACE: [SERIAL] -> UNLOAD_FAIL_SLOT=%d",
                                    idx)
                                if idx in self._assist_active_slots:
                                    self._stop_feed_assist(
                                        idx, why='unload_fail')
                                self._retract_in_progress.discard(idx)
                                self._processed_extruders.add(gate_key)
                                
                            elif actual_stage == "load_feeding":
                                logging.info("ACE: [SERIAL] -> LOAD_SLOT=%d", idx)
                                if self.enable_feed_assist and self.enable_feeder_mode:
                                    self._start_feed_assist(idx, why='load_feeding')
                                else:
                                    if not self.get_sensor_state(idx, eventtime):
                                        self.send_request(request = {"method": "feed_filament", "params": {"index": idx, "length": 3000, "speed": 60}}, callback = None)
                                        self._clear_assist_tracking(idx)
                                        self._feed_start_times[idx] = eventtime
                                        self._active_feeds.add(idx)
                                    else:
                                        logging.info("ACE: sensor hit %d (load feeding stage)" % idx)
                                        self.send_request(request = {"method": "stop_feed_filament", "params": {"index": idx}}, callback = None)
                                        self._clear_assist_tracking(idx)
                                self._processed_extruders.add(gate_key)

                            elif actual_stage == "load_fail":
                                logging.info("ACE: [SERIAL] -> FAILED_SLOT=%d", idx)
                                if self.enable_feed_assist and self.enable_feeder_mode:
                                    self._stop_feed_assist(idx, why='load_fail')
                                else:
                                    self.send_request(request = {"method": "stop_feed_filament", "params": {"index": idx}}, callback = None)
                                    self._clear_assist_tracking(idx)
                                    self._active_feeds.discard(idx)
                                    self._feed_start_times.pop(idx, None)
                                self._processed_extruders.add(gate_key)

                            elif actual_stage == "load_finish":
                                logging.info("ACE: [SERIAL] -> LOAD_COMPLETE_SLOT=%d", idx)
                                if self.enable_feed_assist and self.enable_feeder_mode:
                                    self._stop_feed_assist(idx, why='load_finish')
                                else:
                                    self.send_request(request = {"method": "stop_feed_filament", "params": {"index": idx}}, callback = None)
                                    self._clear_assist_tracking(idx)
                                    self._active_feeds.discard(idx)
                                    self._feed_start_times.pop(idx, None)
                                self._processed_extruders.add(gate_key)
                                
                            elif actual_stage == "preload_finish":
                                # Race detection: U1's port event handler can
                                # fire FEED_ACT_REMOVE_FILAMENT immediately
                                # after the unload routine completes. That
                                # handler overwrites UNLOAD_FINISH with
                                # PRELOAD_FINISH at event-queue speed --
                                # faster than our 0.25 s _main_eval poll
                                # catches the intermediate state. So when
                                # we see preload_finish following an in-
                                # progress unload_* stage, treat it as a
                                # post-unload transition and run the chunked
                                # retract that unload_finish would have.
                                prev = (self._prev_channel_stages.get(
                                            state_key) or '')
                                is_post_unload = (
                                    prev.startswith('unload_')
                                    and not prev.endswith('_finish')
                                    and not prev.endswith('_fail'))
                                if is_post_unload:
                                    logging.info(
                                        "ACE: post-unload preload_finish "
                                        "(prev=%s) -> retract slot=%d",
                                        prev, idx)
                                    if idx in self._assist_active_slots:
                                        self._stop_feed_assist(
                                            idx, why='post_unload_preload')
                                    self._start_retract(idx)
                                else:
                                    logging.info(
                                        "ACE: PRELOAD_COMPLETE_SLOT=%d "
                                        "(prev=%s)", idx, prev)
                                    self.set_slot_rfid_info(idx)
                                self._processed_extruders.add(gate_key)
                                
        return eventtime + 0.25


    def is_ready(self):
        return self._connected and bool(self._info)


    def get_filament_detect(self, index):
        if self._last_filament_status[index] == 'empty':
            return 0
        else:
            return 1


    def _check_auto_feed(self):
        slots = self._info.get('slots', [])
        if not slots: return
        ace_is_busy = any(s.get('status') in ['feeding', 'preload'] for s in slots)

        for i in range(min(len(slots), 4)):
            current_status = slots[i].get('status')
            
            if current_status == 'empty':
                if self._last_filament_status[i] != 'empty':
                    self.clear_slot_rfid_info(i)
                    logging.info("ACE: Slot %d empty." % i)
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
                        logging.info("ACE: Slot %d trigger. Entering Incremental Feed" % i)
                        self.auto_feed_step[i] = 1
                        self.feed_sent[i] = False

            elif self.auto_feed_step[i] == 1:
                if self.port_sensor_hit[i]:
                    logging.info("ACE: Slot %d SENSOR HIT! Moving to Seating" % i)
                    self.send_request(request = {"method": "stop_feed_filament", "params": {"index": i}}, callback = None)
                    self._clear_assist_tracking(i)
                    self.auto_feed_step[i] = 2
                    self.feed_sent[i] = False

                elif self.feed_sent[i] and current_status == 'ready' and not ace_is_busy:
                    logging.warning("ACE: Slot %d move completed. Advancing to seating." % i)
                    self.auto_feed_step[i] = 2
                    self.feed_sent[i] = False

                else:
                    if current_status == 'ready' and not self.feed_sent[i] and not ace_is_busy:
                        logging.info("ACE: Slot %d - Motor is free. Sending feed command." % i)
                        self.send_request(request = {"method": "feed_filament", "params": {"index": i, "length": self.feed_lengths[i], "speed": self.feed_speed}}, callback = None)
                        self._clear_assist_tracking(i)

                        if not self.enable_feeder_mode:
                            self.feed_sent[i] = True
                            ace_is_busy = True

            elif self.auto_feed_step[i] == 2:
                if not ace_is_busy:
                    logging.info("ACE: Slot %d performing final seating move." % i)
                    self._do_seating_move(i)
                    self.auto_feed_step[i] = 3
                    ace_is_busy = True

            elif self.auto_feed_step[i] == 3:
                if current_status == 'ready':
                     self.auto_feed_step[i] = 0
            
            self._last_filament_status[i] = current_status


    def _do_seating_move(self, i):
        if not self.enable_feeder_mode:
            logging.debug("ACE: Slot %d move finished" % i)
            self.set_slot_rfid_info(i)
            return
        if self.port_sensor_hit[i]:
            logging.info("ACE: Slot %d sensor hit. Sending seating move at %d mm/s." % (i, self.load_speed))
            self.send_request(request = {"method": "feed_filament", "params": {"index": i, "length":  self.load_lengths[i], "speed": self.load_speed}}, callback = None)
            self._clear_assist_tracking(i)
        else:
            logging.warning("ACE: Slot %d primary move finished but Printer %d sensor is EMPTY." % (i, i))


    def _handle_status_update(self, _printer_instance, response):
        if 'result' in response:
            self._info = response['result']
            # Log slot status transitions so it's clear when a slot enters or
            # leaves 'assisting' / 'feeding' / 'preload' / 'ready' on the wire.
            slots = self._info.get('slots') or []
            if not hasattr(self, '_last_logged_slot_status'):
                self._last_logged_slot_status = [None, None, None, None]
            for i, slot in enumerate(slots[:4]):
                st = slot.get('status')
                prev = self._last_logged_slot_status[i]
                if st != prev:
                    logging.info(
                        "ACE: slot[%d] status %s -> %s (rfid=%s)",
                        i, prev, st, slot.get('rfid'))
                    self._last_logged_slot_status[i] = st
                    if (i in self._retract_in_progress
                            and st in ('empty', 'ready')):
                        logging.info(
                            "ACE: slot %d retract complete (status=%s)",
                            i, st)
                        self._retract_in_progress.discard(i)
                    if st == 'assist_error' and prev != 'assist_error':
                        self._handle_assist_error(i)
                    elif (st in ACE_ERROR_STATUSES
                            and prev not in ACE_ERROR_STATUSES):
                        self._handle_slot_error(i, st)
                    elif (i in self._assist_active_slots
                            and st in _STATES_INCOMPATIBLE_WITH_ASSIST):
                        # Device-side mode change we didn't initiate
                        # (explicit feed/unwind, slot empty, ...).
                        # Drop stale assist bookkeeping so the watchdog
                        # can re-arm cleanly on the next tick.
                        logging.info(
                            "ACE: slot[%d] dropped assist (status=%s); "
                            "clearing local assist tracking", i, st)
                        self._clear_assist_tracking(i)
            self._check_auto_feed()


    def _handle_assist_error(self, idx):
        """assist_error is a transient ACE watchdog timeout.

        Restart assist immediately -- sending stop_feed_assist to a slot in
        assist_error does not clear the error; only a new start_*_assist does.
        If printing, always restart feed assist. Otherwise pick the mode based
        on current extruder velocity: positive -> feed, negative -> unwind,
        zero -> feed.
        """
        self._clear_assist_tracking(idx)

        if self._printing:
            mode = 'feed'
        else:
            velocity = self._get_extruder_velocity()
            mode = 'unwind' if velocity < 0 else 'feed'

        logging.info("ACE: slot %d assist_error -- restarting as %s "
                     "(printing=%s)", idx, mode, self._printing)
        if mode == 'feed':
            self._start_feed_assist(idx, why='assist_error_restart')
        else:
            self._start_unwind_assist(idx, why='assist_error_restart')


    def _handle_slot_error(self, idx, kind):
        """React to FEED/ROLLBACK/ASSIST/PRELOAD/STUCK/TANGLED/MOTOR errors.

        Drops local assist/feed bookkeeping on the slot, tells the device to
        stop, and hops onto the reactor to abort the matching filament_feed
        channel and surface the error. Runs from the reader thread, so any
        printer-state mutation is deferred to register_async_callback.
        """
        msg = "ACE slot %d %s -- aborting operation" % (idx, kind)
        logging.error(msg)

        # Local bookkeeping: the device is errored, drop our cached "this
        # slot is assisting / feeding" state so the watchdog doesn't think
        # everything is fine.
        self._clear_assist_tracking(idx)
        self._active_feeds.discard(idx)
        self._feed_start_times.pop(idx, None)
        self._retract_in_progress.discard(idx)

        # Tell the device to stop whatever it was doing on this slot. These
        # are queued through the writer thread, so they're safe from here.
        self.send_request(
            request={'method': 'stop_feed_filament',
                     'params': {'index': idx}},
            callback=None)
        self.send_request(
            request={'method': 'stop_feed_assist',
                     'params': {'index': idx}},
            callback=None)

        # Anything below mutates gcode / filament_feed state, so it must run
        # on the reactor thread.
        self.reactor.register_async_callback(
            lambda e, m=msg, i=idx, k=kind:
                self._raise_ace_error_async(m, i, k))


    _FILAMENT_FAIL_PREFIXES = (
        ('load_',        'load_fail'),
        ('unload_',      'unload_fail'),
        ('preload_',     'preload_fail'),
        ('manual_sta_',  'manual_sta_fail'),
    )

    def _abort_filament_op(self, idx):
        """Force any in-flight load/unload/preload/manual operation on this
        slot's extruder into the matching _FAIL state, and pre-arm the
        filament_feed channel_error so cmd_FEED_AUTO surfaces the touch UI
        popup.

        Mechanism (verified against Snapmaker u1-klipper filament_feed.py):
          - cmd_FEED_AUTO wraps _do_feed in try/except; on except it calls
            ``gcmd.error(msg, action='pause', id=525)`` which is the U1
            fork's gcode-error format that the touch UI watches for popups.
          - _do_feed has a checkpoint at the end of the LOAD_FEEDING phase:
            ``if channel_error[ch] != FEED_OK: raise ValueError(...)``.
            Pre-setting channel_error from the outside is what makes
            _do_feed self-raise so cmd_FEED_AUTO's except block runs and
            the popup appears. (LOAD_EXTRUDING / LOAD_FLUSHING /
            UNLOAD_DOING phases have no such checkpoint, so for those we
            still rely on _do_feed's own natural failure -- which will
            happen because the ACE can't deliver filament -- but the
            popup still ends up correctly populated because we've staged
            channel_error / channel_error_state ahead of time.)
          - get_status() exposes channel_error / channel_error_state /
            channel_state to the UI, so polling-based listeners also see
            the failure even before the popup fires.
        """
        fd = self.printer.lookup_object('filament_detect', None)
        if fd is None:
            return False
        feed_objs = getattr(fd, 'filament_feed_objects', None) or []
        aborted = False
        for obj_name, f_obj in feed_objs:
            ch_count = len(getattr(f_obj, 'channel_state', []) or [])
            for ch in range(ch_count):
                try:
                    if f_obj.filament_ch[ch] != idx:
                        continue
                    stage = str(f_obj.channel_state[ch])
                except Exception:
                    continue
                fail_state = None
                for prefix, target in self._FILAMENT_FAIL_PREFIXES:
                    if stage.startswith(prefix) and not stage.endswith('_fail'):
                        fail_state = target
                        break
                if fail_state is None:
                    continue
                try:
                    # 1. Stage channel_error FIRST. _do_feed's LOAD_FEEDING
                    #    checkpoint reads it; cmd_FEED_AUTO's except path
                    #    preserves any non-OK value (only overwrites it
                    #    with 'general' when it was still 'ok').
                    if hasattr(f_obj, 'channel_error'):
                        cur_err = f_obj.channel_error[ch]
                        if cur_err in (None, '', 'ok'):
                            # 'general' is FEED_ERR -- the catch-all
                            # the UI knows how to render.
                            f_obj.channel_error[ch] = 'general'
                    # 2. Snapshot the state at which the error happened.
                    if hasattr(f_obj, 'channel_error_state'):
                        f_obj.channel_error_state[ch] = f_obj.channel_state[ch]
                    # 3. Optional: bump exception_code so the UI gets
                    #    something other than 0 if it cares.
                    if hasattr(f_obj, 'exception_code'):
                        try:
                            f_obj.exception_code[ch] = 99  # ACE-side error
                        except Exception:
                            pass
                    # 4. Flip the channel into the matching _fail state.
                    if hasattr(f_obj, '_set_channel_state'):
                        f_obj._set_channel_state(ch, fail_state, True)
                    else:
                        f_obj.channel_state[ch] = fail_state
                    aborted = True
                    ce = (f_obj.channel_error[ch]
                          if hasattr(f_obj, 'channel_error') else 'n/a')
                    logging.info(
                        "ACE: forced %s ch=%d into %s (was %s) for slot %d "
                        "channel_error=%s",
                        obj_name, ch, fail_state, stage, idx, ce)
                except Exception as e:
                    logging.error(
                        "ACE: failed forcing fail state on slot %d: %s",
                        idx, e)
        return aborted


    def _raise_ace_error_async(self, msg, idx, kind):
        """Reactor-thread half of slot error handling.

        Order matters: cancel the in-progress load/unload first (so the
        state machine stops driving the printer), then text overlays for
        the touch UI and the web UI, then a print pause if we were running
        a job.
        """
        # 1. Cancel any matching filament_feed operation. This is what makes
        #    the U1 touch screen pop the load-failed dialog.
        self._abort_filament_op(idx)

        # 2. Touchscreen-friendly short message via display_status.
        short = "ACE slot %d %s" % (idx, kind)
        ds = self.printer.lookup_object('display_status', None)
        if ds is not None:
            try:
                ds.message = short
            except Exception as e:
                logging.error("ACE: display_status.message set failed: %s", e)
        # M117 as a backup -- some UIs only watch the gcode stream.
        try:
            self.gcode.run_script_from_command("M117 %s" % short)
        except Exception:
            pass

        # 3. mainsail/fluidd error toast.
        try:
            self.gcode.respond_raw("!! %s" % msg)
        except Exception as e:
            logging.error("ACE: respond_raw failed: %s", e)

        # 4. Pause an active print so the operator can recover.
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is not None:
            try:
                state = print_stats.get_status(
                    self.reactor.monotonic()).get('state')
            except Exception:
                state = None
            if state == 'printing':
                try:
                    self.gcode.run_script_from_command('PAUSE')
                    logging.info("ACE: paused print due to %s", msg)
                except Exception as e:
                    logging.error("ACE: failed to pause print: %s", e)


    def _writer(self):
        if self._is_v2:
            self._writer_v2()
            return
        while self._connected:
            try:
                if not self._queue.empty():
                    task = self._queue.get()
                    if task:
                        req, cb = task
                        msg_id = self._request_id
                        self._request_id = (self._request_id + 1) % 300000
                        self._callback_map[msg_id] = cb
                        req['id'] = msg_id
                        self._send_request(req)
                else:
                    msg_id = self._request_id
                    self._request_id = (self._request_id + 1) % 300000
                    self._callback_map[msg_id] = self._handle_status_update
                    self._send_request({"id": msg_id, "method": "get_status"})

                time.sleep(0.5)
            except Exception as e:
                logging.error("ACE Writer error: %s" % str(e))
                time.sleep(0.5)


    def _writer_v2(self):
        while self._connected:
            try:
                if not self._queue.empty():
                    task = self._queue.get()
                    if task:
                        req, cb = task
                        self._send_request_v2(req, cb)
                else:
                    self._send_request_v2(
                        {'method': 'get_status'},
                        self._handle_status_update)
                time.sleep(0.5)
            except Exception as e:
                logging.error("ACE v2 Writer error: %s", str(e))
                time.sleep(0.5)


    def _reader(self):
        if self._is_v2:
            self._reader_v2()
            return
        while self._connected:
            try:
                header = self._serial.read(2)
                if header != b"\xFF\xAA": continue
                len_bytes = self._serial.read(2)
                if len(len_bytes) < 2: continue
                payload_len = struct.unpack('@H', len_bytes)[0]
                full_payload = self._serial.read(payload_len + 3)
                json_str = full_payload[:payload_len].decode('utf-8', errors='ignore')

                response = json.loads(json_str)
                msg_id = response.get('id')
                if msg_id in self._callback_map:
                    callback = self._callback_map.pop(msg_id)
                    callback(self, response)
            except:
                time.sleep(0.1)


    def _reader_v2(self):
        while self._connected:
            try:
                n = self._serial.in_waiting
                if n:
                    self._v2_rx_buf.extend(self._serial.read(n))
                else:
                    time.sleep(0.02)
                if not self._v2_rx_buf:
                    continue
                packets, self._v2_rx_buf = ace2_protocol.parse_stream(
                    self._v2_rx_buf)
                for p in packets:
                    if not p['is_resp']:
                        continue
                    entry = self._v2_callback_map.pop(p['seq'], None)
                    if not entry:
                        continue
                    callback, decoder = entry
                    try:
                        response = (decoder(p['payload'])
                                    if decoder else {'code': 0, 'msg': ''})
                    except Exception as e:
                        logging.error(
                            "ACE: v2 decode error for cmd 0x%02X: %s",
                            p['cmd'], e)
                        response = {'code': 0, 'msg': ''}
                    response['id'] = p['seq']
                    if callback:
                        try:
                            callback(self, response)
                        except Exception as e:
                            logging.error(
                                "ACE: v2 callback error: %s\n%s",
                                e, traceback.format_exc())
            except Exception as e:
                logging.error("ACE: v2 reader error: %s", e)
                time.sleep(0.1)


    def dwell(self, delay):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(delay)


    def set_slot_rfid_info(self, index):
        def _callback(_ace_ref, resp):
            if not resp or 'result' not in resp:
                return

            s = resp.get('result', {})

            # Surface the raw fields the MMU reported so users can sanity-check
            # them in klippy.log without enabling debug logging. Diameter is
            # the most common thing to verify (should read ~1.75 mm).
            logging.info(
                "ACE: filament_info slot=%d sku=%s type=%s brand=%s "
                "diameter=%s color=%s total=%s remainder=%s rfid=%s",
                index, s.get('sku'), s.get('type'), s.get('brand'),
                s.get('diameter'), s.get('color'), s.get('total'),
                s.get('remainder'), s.get('rfid'))

            if s.get('rfid', 1) != 2:
                return

            sku = s.get('sku')
            f_type = s.get('type')
            brand = s.get('brand')
            color_list = s.get('color', [0, 0, 0])
        
            r, g, b = color_list[0], color_list[1], color_list[2]
            rgb_packed = "%02X%02X%02X" % (r, g, b)
            rgb_ipacked = (r << 16) | (g << 8) | b
            colors_raw = s.get('colors', [[0, 0, 0, 255]])
            alpha = colors_raw[0][3] if colors_raw and len(colors_raw[0]) > 3 else 255
            alpha_hex = "%02X" % alpha
            filament_detect = self.printer.lookup_object('filament_detect', None)

            if brand.lower() == "ac":
                filament_info = f_type.split(" ", 1)
                f_type = filament_info[0]
                brand = filament_info[1] if len(filament_info) > 1 else ""
                sku = "Anycubic"

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
                length_map = {330:1000, 247:750, 198:600, 165:500, 82:250}
                total_len = s.get('total')
                info = copy.deepcopy(filament_protocol.FILAMENT_INFO_STRUCT)
                info['VERSION'] = 1
                info['VENDOR'] = sku
                info['MANUFACTURER'] = sku
                info['MAIN_TYPE'] = f_type
                info['SUB_TYPE'] = brand
                info['COLOR_NUMS'] = 1
                info['RGB_1'] = rgb_ipacked
                try:
                    info['ALPHA'] = max(0x00, min(0xFF, int(alpha)))
                except (ValueError, TypeError):
                    info['ALPHA'] = 0xFF
                info['ARGB_COLOR'] = info['ALPHA'] << 24 | info['RGB_1']
                info['OFFICIAL'] = True
                info['LENGTH'] = total_len
                try:
                    info['DIAMETER'] = int(round(float(s.get('diameter', 1.75)) * 100))
                except:
                    info['DIAMETER'] = 175
                try:
                    info['WEIGHT'] = int(length_map.get(total_len, 1000))
                except:
                    info['WEIGHT'] = 1000
                try:
                    info['HOTEND_MIN_TEMP'] = int(s.get('extruder_temp', {}).get('min', 190))
                    info['HOTEND_MAX_TEMP'] = int(s.get('extruder_temp', {}).get('max', 220))
                except:
                    info['HOTEND_MIN_TEMP'] = 0
                    info['HOTEND_MAX_TEMP'] = 0
                try:
                    bed_min_temp = int(s.get('hotbed_temp', {}).get('min', 50))
                    bed_max_temp = int(s.get('hotbed_temp', {}).get('max', 60))
                    info['BED_TEMP'] = bed_min_temp if bed_min_temp > 0 else bed_max_temp
                except:
                    info['BED_TEMP'] = 0
                info['FIRST_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
                info['OTHER_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
                info['MF_DATE'] = datetime.now().strftime("%Y%m%d")
                info['CARD_UID'] = [random.randint(0, 255) for _ in range(7)]
                filament_detect._filament_info_update(index, info, is_clear=True)

        self.send_request(request={"method": "get_filament_info", "params": {"index": index}}, callback=_callback)
        

    def clear_slot_rfid_info(self, index):
        filament_detect = self.printer.lookup_object('filament_detect', None)
        if filament_detect is None:
            command = f"SET_PRINT_FILAMENT_CONFIG CONFIG_EXTRUDER={index} VENDOR='' FILAMENT_TYPE='' FILAMENT_SUBTYPE='' FILAMENT_COLOR_RGBA=000000FF"
            self.gcode.run_script_from_command(command)
        else:
            info = copy.deepcopy(filament_protocol.FILAMENT_INFO_STRUCT)
            filament_detect._filament_info_update(index, info, is_clear=True)


    cmd_ACE_START_DRYING_help = 'Start ACE filament dryer'
    def cmd_ACE_START_DRYING(self, gcmd):
        temperature = gcmd.get_int('TEMPERATURE')
        duration = gcmd.get_int('DURATION', 240)
        if duration <= 0:
            raise gcmd.error('Wrong duration')
        if temperature <= 0 or temperature > self.max_dryer_temp:
            raise gcmd.error('Wrong temperature')
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise gcmd.error("ACE Error: " + response['msg'])
            self.gcode.respond_info('Started ACE drying at %d°C for %d minutes' % (temperature, duration))
        self.send_request(request = {"method": "drying", "params": {"temp":temperature, "fan_speed": 7000, "duration": duration}}, callback = callback)


    cmd_ACE_STOP_DRYING_help = 'Stop ACE filament dryer'
    def cmd_ACE_STOP_DRYING(self, gcmd):
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise gcmd.error("ACE Error: " + response['msg'])
            self.gcode.respond_info('Stopped ACE drying')
        self.send_request(request = {"method":"drying_stop"}, callback = callback)


    cmd_ACE_GET_TEMP_help = ('Read ACE 2 Pro internal temperatures and '
                             'humidity (ACE 1 Pro: not available)')
    def cmd_ACE_GET_TEMP(self, gcmd):
        if not self._is_v2:
            gcmd.respond_info("ACE_GET_TEMP: Not available on ACE 1 Pro")
            return
        if not self._connected:
            gcmd.respond_info("ACE_GET_TEMP: ACE not connected")
            return

        def callback(_ace, response):
            if not response or 'result' not in response:
                self.gcode.respond_info(
                    "ACE_GET_TEMP: no response from device")
                return
            r = response['result']
            self.gcode.respond_info(
                "ACE 2 Pro temperatures:\n"
                "  Box 1: %.1f C    Box 2: %.1f C\n"
                "  PTC 1: %.1f C    PTC 2: %.1f C\n"
                "  Env:   %.1f C    Humidity: %.1f %%"
                % (r.get('box1_temp', 0.0), r.get('box2_temp', 0.0),
                   r.get('ptc1_temp', 0.0), r.get('ptc2_temp', 0.0),
                   r.get('env_temp', 0.0), r.get('env_humidity', 0.0)))

        self.send_request(request={'method': 'get_temp'}, callback=callback)


    cmd_ACE_GET_STATUS_help = ('Snapshot of ACE connection / device / '
                               'slot / dryer / feed-assist state')
    def cmd_ACE_GET_STATUS(self, gcmd):
        # Header and connection state.
        version = 'ACE 2 Pro' if self._is_v2 else 'ACE 1 Pro'
        lines = [
            "ACE Status",
            "  Device:    %s on %s @ %d baud"
            % (version, self.serial_name, self.baud),
            "  Connected: %s" % ('yes' if self._connected else 'no'),
        ]

        if not self._connected:
            gcmd.respond_info("\n".join(lines))
            return

        # _info is populated by the regular get_status polling
        # (every ~0.5 s) via _handle_status_update; reading the cache
        # is fine here -- no need for an async round-trip.
        info = self._info or {}
        if not info:
            lines.append("  (no status data yet -- polling may "
                         "not have completed first round)")
            gcmd.respond_info("\n".join(lines))
            return

        # Work state + temps.
        lines.append("  Work:      %s" % info.get('status', '?'))
        temp = info.get('temp')
        humidity = info.get('humidity')
        lines.append(
            "  Box temp:  %s C    Humidity: %s %%"
            % (temp if temp is not None else '?',
               humidity if humidity is not None else '?'))

        # Dryer.
        dryer = info.get('dryer_status') or info.get('dryer') or {}
        if dryer:
            def _fmt_sec(val):
                try:
                    s = int(val)
                except (TypeError, ValueError):
                    return str(val)
                h, rem = divmod(s, 3600)
                m, sec = divmod(rem, 60)
                return "%02d:%02d:%02d" % (h, m, sec)
            lines.append(
                "  Dryer:     %s (target=%s C, duration=%s, remain=%s)"
                % (dryer.get('status', '?'),
                   dryer.get('target_temp', '?'),
                   _fmt_sec(dryer.get('duration', '?')),
                   _fmt_sec(dryer.get('remain_time',
                                      dryer.get('remain', '?')))))

        # Slot table.
        rfid_names = {
            0: 'empty', 1: 'unknown',
            2: 'identified', 3: 'identifying',
        }
        slots = info.get('slots') or []
        lines.append("")
        lines.append("Slots:")
        for i in range(4):
            slot = slots[i] if i < len(slots) else {}
            slot_status = slot.get('status', '-')
            rfid = slot.get('rfid')
            rfid_label = rfid_names.get(rfid, str(rfid))
            row = ("  %d: status=%-12s filament=%s"
                   % (i, slot_status, rfid_label))
            if i in self._assist_active_slots:
                row += "  assist=%s" % self._assist_mode_per_slot.get(i, '?')
            lines.append(row)

        # filament_feed channel state (the U1's per-channel state machine).
        fd = self.printer.lookup_object('filament_detect', None)
        feed_objs = (getattr(fd, 'filament_feed_objects', None) or []
                     if fd is not None else [])
        if feed_objs:
            lines.append("")
            lines.append("filament_feed channels:")
            for obj_name, f_obj in feed_objs:
                states = getattr(f_obj, 'channel_state', None) or []
                fchans = getattr(f_obj, 'filament_ch', None) or []
                for ch in range(len(states)):
                    extruder = (fchans[ch] if ch < len(fchans) else '?')
                    lines.append(
                        "  %s ch=%d  extruder=%s  stage=%s"
                        % (obj_name, ch, extruder, states[ch]))

        # Feed-assist control state.
        eventtime = self.reactor.monotonic()
        try:
            cur_velocity = self._get_extruder_velocity()
        except Exception:
            cur_velocity = 0.0
        lines.append("")
        lines.append("Feed assist:")
        lines.append(
            "  enabled=%s  idle_timeout=%ss  active_slots=%s"
            % (self.enable_feed_assist,
               self.feed_assist_idle_timeout,
               sorted(self._assist_active_slots) or '[]'))
        slot_ages = {
            s: '%.2fs' % (eventtime - t)
            for s, t in self._last_extrusion_times.items()
        }
        lines.append(
            "  toolhead_velocity=%+.3f mm/s  last_extrusion_ages=%s"
            % (cur_velocity, slot_ages or 'n/a'))

        gcmd.respond_info("\n".join(lines))


    cmd_SET_FILAMENT_CONFIG_help = 'Set filament information'
    def cmd_SET_FILAMENT_CONFIG(self, gcmd):
        channel = gcmd.get_int('CHANNEL')
        vendor = gcmd.get('VENDOR', '')
        filament_type = gcmd.get('TYPE', '')
        subtype = gcmd.get('SUBTYPE', '')
        color = gcmd.get('COLOR', '000000')
        alpha = gcmd.get('ALPHA', 'FF')
        official = gcmd.get('OFFICIAL', 'false').lower()
        length = gcmd.get_int('LENGTH', 330)
        diameter = gcmd.get_int('DIAMETER', 175)
        weight = gcmd.get_int('WEIGHT', 1000)
        extruder_temp_min = gcmd.get_int('EXT_TEMP_MIN', 190)
        extruder_temp_max = gcmd.get_int('EXT_TEMP_MAX', 220)
        hotbed_temp_min = gcmd.get_int('BED_TEMP_MIN', 50)
        hotbed_temp_max = gcmd.get_int('BED_TEMP_MAX', 60)
        
        if self.force_generic or self.check_rfid_status():
            if vendor.lower() != "snapmaker":
                vendor = "Generic"
        
        if vendor.lower() != "snapmaker" and subtype.lower() == "basic":
            subtype = ""

        if channel < 0 or channel >= 4:
            raise gcmd.error("ACE channel{} is out of range[0, 3]".format(channel))
 
        filament_detect = self.printer.lookup_object('filament_detect')
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
            info['VERSION'] = 1
            info['VENDOR'] = vendor
            info['MANUFACTURER'] = vendor
            info['MAIN_TYPE'] = filament_type
            info['SUB_TYPE'] = subtype
            info['COLOR_NUMS'] = 1
            info['RGB_1'] = int(color, 16)
            info['ALPHA'] = int(alpha, 16)
            info['ARGB_COLOR'] = (int(alpha, 16) << 24) | int(color, 16)
            info['OFFICIAL'] = official in ['true', '1', 'yes']
            info['LENGTH'] = length
            info['DIAMETER'] = diameter
            info['WEIGHT'] = weight
            info['HOTEND_MIN_TEMP'] = extruder_temp_min
            info['HOTEND_MAX_TEMP'] = extruder_temp_max
            info['BED_TEMP'] = hotbed_temp_min if hotbed_temp_min > 0 else hotbed_temp_max
            info['FIRST_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
            info['OTHER_LAYER_TEMP'] = info['HOTEND_MIN_TEMP']
            info['MF_DATE'] = datetime.now().strftime("%Y%m%d")
            info['CARD_UID'] = [random.randint(0, 255) for _ in range(7)]
            filament_detect._filament_info_update(channel, info, is_clear=True)
            self.gcode.respond_info(str(info))


def load_config(config):
    return AceDevice(config)
    