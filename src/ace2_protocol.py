# https://github.com/DnG-Crafts/U1-Ace
#
# ACE 2 Pro protocol helpers.
#
# Physical : CH343 USB-UART, 230400 baud, 8N1
# Frame    : [FF AA][flags][seq_lo seq_hi][cmd][len][protobuf][CRC16_lo CRC16_hi][FE]
# CRC      : CRC-16/KERMIT (poly 0x8408), init 0xFFFF, over flags..payload
# Payload  : Protocol Buffers proto3, package ace_com
#
# This module is pure data: no serial, no threading. It only knows how to
# build/parse packets and how to translate between the v1 JSON dict API
# used by ace_device.py and the v2 protobuf wire format.
import logging
import struct
from typing import Any, cast

# ---- Wire constants ---------------------------------------------------------

BAUD = 230400
PREAMBLE = b'\xff\xaa'
END_MARKER = 0xFE
FLAG_REQUEST = 0x00
FLAG_RESPONSE = 0x80


# ---- Commands (subset we use) ----------------------------------------------

CMD_DISCOVER_DEVICE       = 0
CMD_ASSIGN_DEVICE_ID      = 1
CMD_GET_STATUS            = 6
CMD_GET_INFO              = 7
CMD_FEED_OR_ROLLBACK      = 8
CMD_STOP_FEED_OR_ROLLBACK = 9
CMD_UPDATE_SPEED          = 10
CMD_DRYING                = 11
CMD_GET_FILAMENT_INFO     = 13
CMD_SET_FEED_CHECK        = 19
CMD_GET_TEMP              = 64


# FeedOrRollbackRequest.mode values
FEED_MODE_FEED          = 0
FEED_MODE_ROLLBACK      = 1
FEED_MODE_FEED_ASSIST   = 2
FEED_MODE_UNWIND_ASSIST = 3

# Static assist speeds sent with every start_feed_assist / unwind_assist.
FEED_ASSIST_SPEED   = 10
UNWIND_ASSIST_SPEED = 0


# SlotState / FilamentState values (from ace-2-pro-proto.proto)
SLOT_READY              = 0
SLOT_FEEDING            = 1
SLOT_ROLLBACK           = 2
SLOT_ASSISTING          = 3
SLOT_ROLLBACK_ASSISTING = 4
SLOT_PRELOADING         = 5
SLOT_UPGRADING          = 6
SLOT_FEED_ERROR         = 129

FILAMENT_EMPTY       = 0
FILAMENT_UNKNOWN     = 1
FILAMENT_IDENTIFIED  = 2
FILAMENT_IDENTIFYING = 3

DRY_STATE_NAMES = {
    0: 'free', 1: 'starting', 2: 'keeping', 3: 'stopping',
    4: 'ptc_error', 5: 'ntc_error',
}


# ---- CRC --------------------------------------------------------------------

def crc16_kermit(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


# ---- Minimal protobuf encode/decode ----------------------------------------

def pb_varint(value):
    r = bytearray()
    while value > 0x7F:
        r.append((value & 0x7F) | 0x80)
        value >>= 7
    r.append(value & 0x7F)
    return bytes(r)


def pb_uint32(field, value):
    return pb_varint((field << 3) | 0) + pb_varint(int(value) & 0xFFFFFFFF)


def pb_bool(field, value):
    return pb_varint((field << 3) | 0) + pb_varint(1 if value else 0)


def pb_string(field, value):
    data = value.encode('utf-8') if isinstance(value, str) else bytes(value)
    return pb_varint((field << 3) | 2) + pb_varint(len(data)) + data


def _pb_decode_varint(data, pos):
    result, shift = 0, 0
    while pos < len(data):
        b = data[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, pos
        shift += 7
    return result, pos


def pb_decode(data):
    fields = {}
    pos = 0
    n = len(data)
    while pos < n:
        tag, pos = _pb_decode_varint(data, pos)
        fnum = tag >> 3
        wtype = tag & 7
        if wtype == 0:
            val, pos = _pb_decode_varint(data, pos)
        elif wtype == 1:
            if pos + 8 > n:
                break
            val = struct.unpack_from('<d', data, pos)[0]
            pos += 8
        elif wtype == 2:
            ln, pos = _pb_decode_varint(data, pos)
            if pos + ln > n:
                break
            val = bytes(data[pos:pos + ln])
            pos += ln
        elif wtype == 5:
            if pos + 4 > n:
                break
            val = struct.unpack_from('<f', data, pos)[0]
            pos += 4
        else:
            break
        fields.setdefault(fnum, []).append((wtype, val))
    return fields


def pb_first(fields, num, default: int | str = 0) -> int | str:
    lst = fields.get(num)
    if not lst:
        return default
    return lst[0][1]


def _pb_first_str(fields, num, default=''):
    val = pb_first(fields, num, default)
    if isinstance(val, (bytes, bytearray)):
        try:
            return val.decode('utf-8', errors='ignore')
        except Exception:
            return default
    return val


# ---- Packet build / parse ---------------------------------------------------

MAX_PAYLOAD = 100


def build_packet(cmd, payload=b'', seq=1, flags=FLAG_REQUEST):
    plen = min(len(payload), MAX_PAYLOAD)
    inner = bytearray([
        flags & 0xFF,
        seq & 0xFF, (seq >> 8) & 0xFF,
        cmd & 0xFF,
        plen & 0xFF,
    ])
    inner.extend(payload[:plen])
    crc = crc16_kermit(bytes(inner))
    return (PREAMBLE
            + bytes(inner)
            + bytes([crc & 0xFF, (crc >> 8) & 0xFF, END_MARKER]))


def parse_stream(buf):
    """Pull all complete frames out of ``buf`` (a bytearray).

    Returns ``(packets, buf)``, where ``buf`` is the (possibly shorter)
    leftover. Each packet is a dict:
        {'cmd', 'seq', 'flags', 'is_resp', 'payload'}
    Frames with bad CRC or desync are discarded.
    """
    packets = []
    while True:
        if len(buf) < 10:  # preamble(2)+header(5)+crc(2)+end(1)
            break
        idx = buf.find(PREAMBLE)
        if idx < 0:
            # preserve trailing byte in case next chunk starts mid-preamble
            del buf[:-1]
            break
        if idx > 0:
            del buf[:idx]
            continue
        plen = buf[6]
        if plen > MAX_PAYLOAD:
            del buf[:2]
            continue
        total = 2 + 5 + plen + 2 + 1
        if len(buf) < total:
            break
        if buf[total - 1] != END_MARKER:
            del buf[:2]
            continue
        inner = bytes(buf[2:7 + plen])
        crc_r = buf[7 + plen] | (buf[8 + plen] << 8)
        if crc_r != crc16_kermit(inner):
            del buf[:2]
            continue
        flags = buf[2]
        seq = buf[3] | (buf[4] << 8)
        cmd = buf[5]
        packets.append({
            'cmd': cmd,
            'seq': seq,
            'flags': flags,
            'is_resp': bool(flags & 0x80),
            'payload': bytes(buf[7:7 + plen]),
        })
        del buf[:total]
    return packets, buf


# ---- v1-dict → v2 command translation --------------------------------------
#
# The rest of the mod speaks the v1 JSON dialect ({'method': ..., 'params': ...})
# and expects v1-shaped responses ({'result': {...}} or {'code': N, 'msg': ...}).
# These helpers convert between the two worlds so higher-level logic in
# ace_device.py does not need to know which protocol is on the wire.

SLOT_ERROR_STATUS_BY_RAW = {
    SLOT_FEED_ERROR: 'feed_error',         # 0x81
    130:             'rollback_error',     # 0x82
    131:             'assist_error',       # 0x83
    132:             'preload_error',      # 0x84
    133:             'stuck',              # 0x85
    134:             'tangled',            # 0x86
    135:             'motor_error',        # 0x87
}


def _slot_status_to_v1(slot_state, fil_state):
    if fil_state == FILAMENT_EMPTY:
        return 'empty'
    if slot_state == SLOT_FEEDING:
        return 'feeding'
    if slot_state == SLOT_PRELOADING:
        return 'preload'
    if slot_state == SLOT_ROLLBACK:
        return 'unwinding'
    if slot_state in (SLOT_ASSISTING, SLOT_ROLLBACK_ASSISTING):
        return 'assisting'
    err = SLOT_ERROR_STATUS_BY_RAW.get(slot_state)
    if err is not None:
        return err
    if slot_state >= SLOT_FEED_ERROR:
        return 'error'
    return 'ready'


def _decode_status(payload):
    f = pb_decode(payload)
    slots = []
    for _, sd in f.get(9, []):
        sf = pb_decode(sd)
        state = pb_first(sf, 1, 0)
        fil = pb_first(sf, 2, 0)
        slots.append({
            'status': _slot_status_to_v1(state, fil),
            'rfid': 2 if fil == FILAMENT_IDENTIFIED
                      else (1 if fil != FILAMENT_EMPTY else 0),
        })
    result: dict[str, str | list[Any] | int | dict[str, int | str]] = {
        'status': 'ready',
        'slots': slots,
        'temp': pb_first(f, 3, 0),
        'humidity': pb_first(f, 4, 0),
        'feed_assist_count': pb_first(f, 7, 0),
    }
    if 2 in f:
        d = pb_decode(f[2][0][1])
        dry_state_index = cast(int, pb_first(d, 1, 0))
        result['dryer_status'] = {
            'status': DRY_STATE_NAMES.get(dry_state_index, 'free'),
            'target_temp': pb_first(d, 2, 0),
            'duration': pb_first(d, 3, 0),
            'remain_time': pb_first(d, 4, 0),
        }
    return {'result': result}


def _decode_info(payload):
    f = pb_decode(payload)
    return {'result': {
        'version': _pb_first_str(f, 1, ''),
        'boot_version': _pb_first_str(f, 2, ''),
        'first_request': bool(pb_first(f, 3, 0)),
    }}


def _decode_generic(payload):
    f = pb_decode(payload)
    return {'code': pb_first(f, 1, 0), 'msg': ''}


def _decode_temp(payload):
    """Decode GetTempResponse — six 32-bit float fields."""
    f = pb_decode(payload)

    def _flt(num):
        lst = f.get(num)
        if not lst:
            return 0.0
        _, val = lst[0]
        try:
            return float(val)
        except (TypeError, ValueError):
            return 0.0

    return {'result': {
        'box1_temp':    _flt(1),
        'box2_temp':    _flt(2),
        'ptc1_temp':    _flt(3),
        'ptc2_temp':    _flt(4),
        'env_temp':     _flt(5),
        'env_humidity': _flt(6),
    }}


def _decode_filament_info(payload):
    f = pb_decode(payload)
    sku = _pb_first_str(f, 3, '')
    typ = _pb_first_str(f, 4, '')
    colors = []
    primary_rgb = [0, 0, 0]
    for i, (_, c_data) in enumerate(f.get(5, [])):
        cf = pb_decode(c_data)
        rgba = pb_first(cf, 1, 0) & 0xFFFFFFFF
        r = (rgba >> 24) & 0xFF
        g = (rgba >> 16) & 0xFF
        b = (rgba >> 8) & 0xFF
        a = rgba & 0xFF
        colors.append([r, g, b, a])
        if i == 0:
            primary_rgb = [r, g, b]
    if not colors:
        colors = [[0, 0, 0, 255]]

    ex_min = ex_max = 0
    if 6 in f:
        ef = pb_decode(f[6][0][1])
        ex_min = pb_first(ef, 1, 0)
        ex_max = pb_first(ef, 2, 0)
    bd_min = bd_max = 0
    if 7 in f:
        hf = pb_decode(f[7][0][1])
        bd_min = pb_first(hf, 1, 0)
        bd_max = pb_first(hf, 2, 0)

    # v2 diameter is uint32 in 0.01 mm (175 -> 1.75 mm);
    diam_raw = cast(int, pb_first(f, 8, 175))
    try:
        diameter = diam_raw / 100.0
    except Exception:
        diameter = 1.75

    result = {
        'index': pb_first(f, 1, 0),
        'sku': sku,
        'type': typ,
        'brand': '',
        'color': primary_rgb,
        'colors': colors,
        'extruder_temp': {'min': ex_min, 'max': ex_max},
        'hotbed_temp': {'min': bd_min, 'max': bd_max},
        'diameter': diameter,
        'total': pb_first(f, 9, 0),
        'remainder': pb_first(f, 11, 0),
        'code': pb_first(f, 12, 0),
        # v2 does not return a separate rfid flag; we use the presence of an
        # sku/type string as the "identified" signal (v1 uses rfid==2 here).
        'rfid': 2 if (sku or typ) else 1,
    }
    return {'result': result}


def encode_v1_request(request):
    """Translate a v1-style JSON request dict to a v2 wire request.

    Returns (cmd, payload_bytes, response_decoder) or None if the method
    has no v2 equivalent (caller should log and drop it).
    ``response_decoder`` takes a raw protobuf payload and returns a
    v1-shaped response dict (or None if the command expects no answer).
    """
    method = request.get('method')
    params = request.get('params') or {}

    if method == 'get_status':
        return CMD_GET_STATUS, b'', _decode_status

    if method == 'get_info':
        return CMD_GET_INFO, b'', _decode_info

    if method == 'feed_filament':
        idx = int(params.get('index', 0))
        length = int(params.get('length', 0))
        speed = int(params.get('speed', 50))
        payload = (pb_uint32(1, idx) + pb_uint32(2, speed)
                   + pb_uint32(3, length) + pb_uint32(4, FEED_MODE_FEED))
        return CMD_FEED_OR_ROLLBACK, payload, _decode_generic

    if method == 'unwind_filament':
        # ACE-driven phase: a fixed-length ROLLBACK that retracts filament
        # back into the spool after the printer's own extruder has finished
        # pushing it out of the hot end.
        idx = int(params.get('index', 0))
        length = int(params.get('length', 0))
        speed = int(params.get('speed', 25))
        payload = (pb_uint32(1, idx) + pb_uint32(2, speed)
                   + pb_uint32(3, length)
                   + pb_uint32(4, FEED_MODE_ROLLBACK))
        return CMD_FEED_OR_ROLLBACK, payload, _decode_generic

    if method == 'unwind_assist':
        idx = int(params.get('index', 0))
        payload = (pb_uint32(1, idx) + pb_uint32(2, UNWIND_ASSIST_SPEED)
                   + pb_uint32(3, 0)
                   + pb_uint32(4, FEED_MODE_UNWIND_ASSIST))
        return CMD_FEED_OR_ROLLBACK, payload, _decode_generic

    if method == 'stop_feed_filament':
        idx = int(params.get('index', 0))
        return CMD_STOP_FEED_OR_ROLLBACK, pb_uint32(1, idx), _decode_generic

    if method == 'start_feed_assist':
        idx = int(params.get('index', 0))
        payload = (pb_uint32(1, idx) + pb_uint32(2, FEED_ASSIST_SPEED)
                   + pb_uint32(3, 0)
                   + pb_uint32(4, FEED_MODE_FEED_ASSIST))
        return CMD_FEED_OR_ROLLBACK, payload, _decode_generic

    if method == 'stop_feed_assist':
        idx = int(params.get('index', 0))
        return CMD_STOP_FEED_OR_ROLLBACK, pb_uint32(1, idx), _decode_generic

    if method == 'update_speed':
        idx = int(params.get('index', 0))
        speed = int(params.get('speed', 50))
        payload = pb_uint32(1, idx) + pb_uint32(2, speed)
        return CMD_UPDATE_SPEED, payload, _decode_generic

    if method == 'get_temp':
        # GET_TEMP returns GetTempResponse with six float fields:
        #   box1_temp, box2_temp, ptc1_temp, ptc2_temp,
        #   env_temp, env_humidity
        return CMD_GET_TEMP, b'', _decode_temp

    if method == 'get_filament_info':
        idx = int(params.get('index', 0))
        return CMD_GET_FILAMENT_INFO, pb_uint32(1, idx), _decode_filament_info

    if method == 'drying':
        temp = int(params.get('temp', 0))
        duration = int(params.get('duration', 0))
        auto_roll = bool(params.get('auto_roll', False))
        payload = (pb_uint32(1, temp) + pb_uint32(2, duration)
                   + pb_bool(3, auto_roll))
        return CMD_DRYING, payload, _decode_generic

    if method == 'drying_stop':
        # Stop drying by sending CMD_DRYING with temp=0 and duration=0
        payload = pb_uint32(1, 0) + pb_uint32(2, 0)
        return CMD_DRYING, payload, _decode_generic

    if method == 'set_feed_check':
        # Defaults match gklib initializeDevice (0x6292B0): check_len=100, error_len=90.
        # Both are stored as single bytes in the ACE MCU (byte_20000096/97), range 1-255.
        check_len = int(params.get('check_len', 100))
        error_len = int(params.get('error_len', 90))
        payload = pb_uint32(1, check_len) + pb_uint32(2, error_len)
        return CMD_SET_FEED_CHECK, payload, _decode_generic

    logging.info("ACE2: unsupported v1 method '%s' (ignored)", method)
    return None


# ---- Discovery helper -------------------------------------------------------

def build_discover_packet(seq=1):
    return build_packet(CMD_DISCOVER_DEVICE, b'', seq=seq)


def build_assign_id_packet(uid1, uid2, uid3, dev_id=1, seq=2):
    payload = (pb_uint32(1, uid1) + pb_uint32(2, uid2)
               + pb_uint32(3, uid3) + pb_uint32(4, dev_id))
    return build_packet(CMD_ASSIGN_DEVICE_ID, payload, seq=seq)


def parse_discover_response(payload):
    f = pb_decode(payload)
    return (pb_first(f, 1, 0), pb_first(f, 2, 0), pb_first(f, 3, 0))
