#Version 3
# https://github.com/DnG-Crafts/U1-Ace
import serial, threading, time, logging, json, struct, queue, traceback, glob, copy, random
from datetime import datetime
from . import filament_protocol

class AceDevice:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.add_object('ace_device', self)
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        
        self.serial_name = config.get('serial', '/dev/serial/by-id/usb-ANYCUBIC_ACE_1-if00')
        all_devices = glob.glob('/dev/serial/by-id/usb-ANYCUBIC*')
        if all_devices:
            self.serial_name = all_devices[0]
            logging.info("ACE: Found serial port: %s", self.serial_name)
        else:
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
            config.getint('retract_length_slot1', 100),
            config.getint('retract_length_slot2', 100),
            config.getint('retract_length_slot3', 100),
            config.getint('retract_length_slot4', 100)
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
        self._active_feeds = set()
        self._feed_start_times = {}
        self._timeout_threshold = 60.0
        self.auto_feed_step = [0, 0, 0, 0]
        self.feed_sent = [False, False, False, False]
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
        self.gcode.register_command('SET_FILAMENT_CONFIG', self.cmd_SET_FILAMENT_CONFIG)


    def feeder_mode(self):
        return self.enable_feeder_mode

    def feed_assist(self):
        return self.enable_feed_assist

    def _handle_start_print_job(self):
        logging.info("ACE: Print job started.")
        self._last_active_index = -1
        self._last_active_tool = None
         


    def _handle_not_printing(self, eventtime):
        logging.info("ACE: Printer is now IDLE/Ready.")
        if self._last_active_index != -1 and self.enable_feed_assist:
            for i in range(4):
                self.send_request(request = {"method": "stop_feed_assist", "params": {"index": i}}, callback = None)
        self._last_active_index = -1
        self._last_active_tool = None
 

    def _handle_stop_print_job(self):
        logging.info("ACE: Print job stopped/completed.")
        if self._last_active_index != -1 and self.enable_feed_assist:
            for i in range(4):
                self.send_request(request = {"method": "stop_feed_assist", "params": {"index": i}}, callback = None)
        self._last_active_index = -1
        self._last_active_tool = None
  

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



        if not self.enable_feed_assist:
            feeds_to_remove = []
            for idx in self._active_feeds:
                if self.get_sensor_state(idx, eventtime):
                    self.send_request(request={"method": "stop_feed_filament", "params": {"index": idx}}, callback=None)
                    feeds_to_remove.append(idx)
                elif eventtime > self._feed_start_times.get(idx, 0) + self._timeout_threshold:
                    logging.warning("ACE: Feed slot %d TIMEOUT. Stopping." % idx)
                    self.send_request(request={"method": "stop_feed_filament", "params": {"index": idx}}, callback=None)
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
                                else:
                                    if not self.get_sensor_state(idx, eventtime):
                                        self.send_request(request = {"method": "feed_filament", "params": {"index": idx, "length": 3000, "speed": 50}}, callback = None)
                                        self._feed_start_times[idx] = eventtime
                                        self._active_feeds.add(idx) 
                                    else:
                                        self.send_request(request = {"method": "stop_feed_filament", "params": {"index": idx}}, callback = None)
                                self._processed_extruders.add(gate_key)
                                
                            elif actual_stage == "load_fail":
                                logging.info("ACE: [SERIAL] -> FAILED_SLOT=%d", idx)
                                if self.enable_feed_assist:
                                    self.send_request(request = {"method": "stop_feed_assist", "params": {"index": idx}}, callback = None)
                                else:
                                    self.send_request(request = {"method": "stop_feed_filament", "params": {"index": idx}}, callback = None)
                                    self._active_feeds.discard(idx)
                                    self._feed_start_times.pop(idx, None)
                                self._processed_extruders.add(gate_key)                                
                                
                            elif actual_stage == "load_finish":
                                logging.info("ACE: [SERIAL] -> LOAD_COMPLETE_SLOT=%d", idx)
                                if self.enable_feed_assist:
                                    self.send_request(request = {"method": "stop_feed_assist", "params": {"index": idx}}, callback = None)
                                else:
                                    self.send_request(request = {"method": "stop_feed_filament", "params": {"index": idx}}, callback = None)
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
                    logging.info("ACE: Slot %d trigger. Entering Incremental Feed" % i)
                    self.auto_feed_step[i] = 1
                    self.feed_sent[i] = False

            elif self.auto_feed_step[i] == 1:
                if self.port_sensor_hit[i]:
                    logging.info("ACE: Slot %d SENSOR HIT! Moving to Seating" % i)
                    self.send_request(request = {"method": "stop_feed_filament", "params": {"index": i}}, callback = None)
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
            rgb_ipacked = (r << 16) | (g << 8) | b
            colors_raw = s.get('colors', [[0, 0, 0, 255]])
            alpha = colors_raw[0][3] if colors_raw and len(colors_raw[0]) > 3 else 255
            alpha_hex = "%02X" % alpha
            filament_detect = self.printer.lookup_object('filament_detect', None)
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
            command = (f"SET_PRINT_FILAMENT_CONFIG CONFIG_EXTRUDER={index} VENDOR='' FILAMENT_TYPE='' FILAMENT_SUBTYPE='' FILAMENT_COLOR_RGBA=000000FF")
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
            self.gcode.respond_info('Started ACE drying')
        self.send_request(request = {"method": "drying", "params": {"temp":temperature, "fan_speed": 7000, "duration": duration}}, callback = callback)


    cmd_ACE_STOP_DRYING_help = 'Stop ACE filament dryer'
    def cmd_ACE_STOP_DRYING(self, gcmd):
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise gcmd.error("ACE Error: " + response['msg'])
            self.gcode.respond_info('Stopped ACE drying')
        self.send_request(request = {"method":"drying_stop"}, callback = callback)


    cmd_SET_FILAMENT_CONFIG_help = 'Set filament information'
    def cmd_SET_FILAMENT_CONFIG(self, gcmd):
        channel = gcmd.get_int('CHANNEL')
        vendor = gcmd.get('VENDOR', '')
        type = gcmd.get('TYPE', '')
        subtype = gcmd.get('SUBTYPE', '')
        color = gcmd.get('COLOR', '000000')
        alpha = gcmd.get('ALPHA', 'FF')
        official = gcmd.get('OFFICIAL', False)
        length = gcmd.get_int('LENGTH', 330)
        diameter = gcmd.get_int('DIAMETER', 175)
        weight = gcmd.get_int('WEIGHT', 1000)
        extruder_temp_min = gcmd.get_int('EXT_TEMP_MIN', 190)
        extruder_temp_max = gcmd.get_int('EXT_TEMP_MAX', 220)
        hotbed_temp_min = gcmd.get_int('BED_TEMP_MIN', 50)
        hotbed_temp_max = gcmd.get_int('BED_TEMP_MAX', 60)
        
        if channel < 0 or channel >= 4:
            raise gcmd.error("ACE channel{} is out of range[0, 3]".format(channel))
 
        filament_detect = self.printer.lookup_object('filament_detect')
        if filament_detect is None:
            command = (
                f"SET_PRINT_FILAMENT_CONFIG "
                f"CONFIG_EXTRUDER={channel} "
                f"VENDOR='{vendor}' "
                f"FILAMENT_TYPE='{type}' "
                f"FILAMENT_SUBTYPE='{subtype}' "
                f"FILAMENT_COLOR_RGBA={color}{alpha}")
            self.gcode.run_script_from_command(command)
        else:
            info = copy.deepcopy(filament_protocol.FILAMENT_INFO_STRUCT)
            info['VERSION'] = 1
            info['VENDOR'] = vendor
            info['MANUFACTURER'] = vendor
            info['MAIN_TYPE'] = type
            info['SUB_TYPE'] = subtype
            info['COLOR_NUMS'] = 1
            info['RGB_1'] = int(color, 16)
            info['ALPHA'] = int(alpha, 16)
            info['ARGB_COLOR'] = (int(alpha, 16) << 24) | int(color, 16)
            info['OFFICIAL'] = official
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