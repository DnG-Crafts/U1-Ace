# https://github.com/DnG-Crafts/U1-Ace
import serial, threading, time, logging, json, struct, queue, traceback, glob
from datetime import datetime

class AceDevice:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.add_object('ace_device', self)
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        all_devices = glob.glob('/dev/serial/by-id/usb-ANYCUBIC*')
        if all_devices:
            self.serial_name = all_devices[0]
            logging.info("ACE: Found serial port: %s", self.serial_name)
        else:
            self.serial_name = config.get('serial', '/dev/serial/by-id/usb-ANYCUBIC_ACE_1-if00')
            logging.info("ACE: No devices auto detected, Using config serial path %s", self.serial_name)

        self.baud = config.getint('baud', 115200)
        self.feed_speed = config.getint('feed_speed', 90)
        self.load_speed = config.getint('load_speed', 100)
        self.retract_speed = config.getint('retract_speed', 25)
        self.max_dryer_temp = config.getint('max_dryer_temperature', 55)
        self.enable_feed_assist = config.getboolean('enable_feed_assist', True)
        self.enable_feeder_mode = config.getboolean('enable_feeder_mode', False)

        
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
            config.getint('retract_length_slot1', 1150),
            config.getint('retract_length_slot2', 1150),
            config.getint('retract_length_slot3', 1150),
            config.getint('retract_length_slot4', 1150)
        ]


        self._connected = False
        self._serial = None
        self._request_id = 0
        self._queue = queue.Queue()
        self._callback_map = {}
        self._info = {}
        self._feed_assist_index = -1
        self._last_filament_data = [("", "", [0, 0, 0])] * 4 
        self._virtual_uids = [[0]*7 for _ in range(4)]
        self._initialized = False
        self.port_sensor_hit = [False, False, False, False]
        self._last_filament_status = ['empty', 'empty', 'empty', 'empty']
        self.auto_feed_step = [0, 0, 0, 0]
        self._last_active_tool = None
        self._last_active_index = -1
        self._pending_start_index = -1
        self._next_cmd_time = 0


        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('klippy:disconnect', self._handle_disconnect)
        self.printer.register_event_handler("filament_feed:port", self._feed_handler)
        self.printer.register_event_handler('print_stats:start', self._handle_start_print_job)
        self.printer.register_event_handler('print_stats:stop', self._handle_stop_print_job)
        self.printer.register_event_handler('idle_timeout:idle', self._handle_not_printing)
 
        self.gcode.register_command('ACE_START_DRYING', self.cmd_ACE_START_DRYING)
        self.gcode.register_command('ACE_STOP_DRYING', self.cmd_ACE_STOP_DRYING)


    def feeder_mode(self):
        return self.enable_feeder_mode


    def _handle_start_print_job(self):
        logging.info("ACE: Print job started.")


    def _handle_not_printing(self, eventtime):
        logging.info("ACE: Printer is now IDLE/Ready.")
        if self._last_active_index != -1 and self.enable_feed_assist:
            for i in range(4):
                self.send_request(request = {"method": "stop_feed_assist", "params": {"index": i}}, callback = None)
            self._last_active_index = -1

    def _handle_stop_print_job(self):
        logging.info("ACE: Print job stopped/completed.")
        if self._last_active_index != -1 and self.enable_feed_assist:
            for i in range(4):
                self.send_request(request = {"method": "stop_feed_assist", "params": {"index": i}}, callback = None)
            self._last_active_index = -1

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

    def send_request(self, request, callback):
        self._queue.put([request, callback])

    def _handle_ready(self):
        self.connection_timer = self.reactor.register_timer(
            self._connection_timer, self.reactor.NOW)    
        logging.info("ACE: Connection monitor started")

    def _connection_timer(self, eventtime):
        if not self._connected:
            self._attempt_connection()
        return eventtime + (5.0 if not self._connected else 20.0)

    def _feed_handler(self, channel, detect):
        self.port_sensor_hit[channel] = detect


    def update_sensors(self, eventtime):
        if not self.enable_feeder_mode:
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


    def _attempt_connection(self):
        try:
            if self._serial:
                try: self._serial.close()
                except: pass
            
            self._serial = serial.Serial(port=self.serial_name, baudrate=self.baud, timeout=0.2)
            if self._serial.isOpen():
                self._connected = True
                
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
                        logging.info("ACE: Swap detected: %s (Idx %s) -> %s (Idx %s)", self._last_active_tool, old_idx, active_name, new_idx)

                        self._last_active_tool = active_name
                        self._last_active_index = new_idx
                        
                        if self.enable_feed_assist:
                            if old_idx != -1:
                                self.send_request(request = {"method": "start_feed_assist", "params": {"index": new_idx}}, callback = None)
                            else:
                                self.send_request(request = {"method": "start_feed_assist", "params": {"index": new_idx}}, callback = None)

                return eventtime + 0.25



        self.update_sensors(eventtime)

        msm = self.printer.lookup_object('machine_state_manager', None)
        fd = self.printer.lookup_object('filament_detect', None)

        if msm is not None and fd is not None:
            msm_status = msm.get_status()
            action_code = msm_status.get('action_code') 
            
            if not hasattr(self, '_processed_extruders'):
                self._processed_extruders = set()
                self._last_f_stages = {}
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

                    if actual_stage != self._last_f_stages.get(state_key):
                        logging.info("ACE: %s Ch %d | Stage: %s | Extruder: %s", obj_name, ch, actual_stage, assigned_extruder)
                        if assigned_extruder is not None:
                            old_stage = self._last_f_stages.get(state_key)
                            old_gate_key = f"{int(assigned_extruder)}_{old_stage}"
                            if old_gate_key in self._processed_extruders:
                                self._processed_extruders.remove(old_gate_key)
                        
                        self._last_f_stages[state_key] = actual_stage
                    is_preload = actual_stage in ["preload_finish"]
                    if assigned_extruder is not None and (self._allow_triggers or is_preload):
                        idx = int(assigned_extruder)
                        gate_key = f"{idx}_{actual_stage}"

                        if gate_key not in self._processed_extruders:
                            if actual_stage == "unload_finish":
                                logging.info("ACE: [SERIAL] -> UNWIND_SLOT=%d", idx)
                                self.send_request(request = {"method": "unwind_filament", "params": {"index": idx, "length": self.retract_lengths[idx], "speed": self.retract_speed}}, callback = None)
                                self._processed_extruders.add(gate_key)

                            elif actual_stage == "load_feeding":
                                logging.info("ACE: [SERIAL] -> LOAD_SLOT=%d", idx)
                                if self.enable_feed_assist:
                                    self.send_request(request = {"method": "start_feed_assist", "params": {"index": idx}}, callback = None)
                                self._processed_extruders.add(gate_key)
                                
                            elif actual_stage == "load_fail":
                                logging.info("ACE: [SERIAL] -> FAILED_SLOT=%d", idx)
                                if self.enable_feed_assist:
                                    self.send_request(request = {"method": "stop_feed_assist", "params": {"index": idx}}, callback = None)
                                self._processed_extruders.add(gate_key)                                
                                
                            elif actual_stage == "load_finish":
                                logging.info("ACE: [SERIAL] -> LOAD_COMPLETE_SLOT=%d", idx)
                                if self.enable_feed_assist:
                                    self.send_request(request = {"method": "stop_feed_assist", "params": {"index": idx}}, callback = None)
                                self._processed_extruders.add(gate_key)
                                
                            elif actual_stage == "preload_finish":
                                logging.info("ACE: PRELOAD_COMPLETE_SLOT=%d", idx)
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

        for i in range(min(len(slots), 4)):
            current_status = slots[i].get('status')
            
            if current_status == 'empty':
                if self._last_filament_status[i] != 'empty':
                    self.clear_slot_rfid_info(i)
                    logging.info("ACE: Slot %d empty." % i)
                self.auto_feed_step[i] = 0
                self._last_filament_status[i] = 'empty'
                continue

            if self.auto_feed_step[i] == 0:
                prev_status = self._last_filament_status[i]
                if prev_status == 'preload' and current_status == 'ready':
                    logging.info("ACE: Slot %d trigger. Entering Incremental Feed (Step 1)." % i)
                    self.auto_feed_step[i] = 1

            elif self.auto_feed_step[i] == 1:
                if self.port_sensor_hit[i]:
                    logging.info("ACE: Slot %d SENSOR HIT! Moving to Seating (Step 2)." % i)
                    self.send_request(request = {"method": "stop_feed_filament", "params": {"index": i}}, callback = None)
                    self.auto_feed_step[i] = 2
                else:
                    if current_status != 'feeding':
                        logging.debug("ACE: Slot %d feeding... (Sensor Empty)" % i)
                        self.send_request(request = {"method": "feed_filament", "params": {"index": i, "length":  self.feed_lengths[i], "speed": self.feed_speed}}, callback = None)


            elif self.auto_feed_step[i] == 2:
                logging.info("ACE: Slot %d performing final seating move." % i)
                self._do_seating_move(i)
                self.auto_feed_step[i] = 3

            elif self.auto_feed_step[i] == 3:
                if current_status == 'ready':
                     self.auto_feed_step[i] = 0
            
            self._last_filament_status[i] = current_status

    def _do_seating_move(self, i):
        if self.enable_feeder_mode:
            logging.debug("ACE: Slot %d move finished" % i)
            self.set_slot_rfid_info(i)
            return
        if self.port_sensor_hit[i]:
            logging.info("ACE: Slot %d sensor hit. Sending seating move at %d mm/s." % (i, self.load_speed))
            self.send_request(request = {"method": "feed_filament", "params": {"index": i, "length":  self.load_lengths[i], "speed": self.load_speed}}, callback = None)
        else:
            logging.warning("ACE: Slot %d primary move finished but Printer %d sensor is EMPTY." % (i, i))

    def _handle_status_update(self, printer_instance, response):
        if 'result' in response: 
            self._info = response['result']
            self._check_auto_feed()

    def _writer(self):
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

    def _reader(self):
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

    def dwell(self, delay):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(delay)

    def set_slot_rfid_info(self, index):
        def _callback(ace_ref, resp):
            if not resp or 'result' not in resp:
                return
                
            s = resp.get('result', {})
            
            if s.get('rfid', 1) != 2:
                return

            sku = s.get('sku')
            f_type = s.get('type')
            brand = s.get('brand')
            color_list = s.get('color', [0, 0, 0])
        
            r, g, b = color_list[0], color_list[1], color_list[2]
            rgb_packed = "%02X%02X%02X" % (r, g, b)
            colors_raw = s.get('colors', [[0, 0, 0, 255]])
            alpha = colors_raw[0][3] if colors_raw and len(colors_raw[0]) > 3 else 255
            alpha_hex = "%02X" % alpha

            command = (
                f"SET_PRINT_FILAMENT_CONFIG "
                f"CONFIG_EXTRUDER={index} "
                f"VENDOR='{sku}' "
                f"FILAMENT_TYPE='{f_type}' "
                f"FILAMENT_SUBTYPE='{brand}' "
                f"FILAMENT_COLOR_RGBA={rgb_packed}{alpha_hex}")

            self.gcode.run_script_from_command(command)

        self.send_request(request={"method": "get_filament_info", "params": {"index": index}}, callback=_callback)


    def clear_slot_rfid_info(self, index):
        command = (f"SET_PRINT_FILAMENT_CONFIG CONFIG_EXTRUDER={index} VENDOR='' FILAMENT_TYPE='' FILAMENT_SUBTYPE='' FILAMENT_COLOR_RGBA=000000FF")
        self.gcode.run_script_from_command(command)


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
            self.gcode.respond_info('Started ACE drying')
        self.send_request(request = {"method": "drying", "params": {"temp":temperature, "fan_speed": 7000, "duration": duration}}, callback = callback)

    cmd_ACE_STOP_DRYING_help = 'Stop ACE filament dryer'
    def cmd_ACE_STOP_DRYING(self, gcmd):
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise gcmd.error("ACE Error: " + response['msg'])
            self.gcode.respond_info('Stopped ACE drying')
        self.send_request(request = {"method":"drying_stop"}, callback = callback)


def load_config(config):
    return AceDevice(config)