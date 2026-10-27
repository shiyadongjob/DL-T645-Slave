# -*- coding: utf-8 -*-
"""
DL/T 645-2007 多功能电能表通信协议 调试工具 (Slave模拟器)
==================================================================
类似 Redisant DL/T645 Slave 功能，本地模拟串口采集。
特性:
  * 忽略表号（响应任意地址）
  * 无需登录/密码校验
  * 支持读数据 (0x01)、读后续数据 (0x02)、写数据 (0x04)、广播校时 (0x0C) 等
  * 内置常用数据标识字典，数值可在线编辑
  * 实时显示收发帧 HEX 日志
"""

import threading
import time
import struct
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox


# ============================================================================
# 1. 协议常量
# ============================================================================

HEADER = 0x68
FOOTER = 0x16
OFFSET = 0x33  # 数据域字节偏移量 (33H)

# 控制码 (主站请求)
CMD_READ_DATA       = 0x01   # 读数据
CMD_READ_DATA_EXT   = 0x11   # 读数据(扩展码, 部分主站使用)
CMD_READ_NEXT       = 0x02   # 读后续数据
CMD_READ_BLOCK      = 0x03   # 读数据块
CMD_WRITE_DATA      = 0x04   # 写数据
CMD_WRITE_BLOCK     = 0x05   # 写数据块
CMD_BROADCAST_TIME  = 0x0C   # 广播校时
CMD_RESET           = 0x10   # 复位
CMD_READ_ADDR       = 0x13   # 读通信地址

# 控制码 (从站应答 = 请求 | 0x80)
RESP_READ_DATA      = 0x81
RESP_READ_DATA_EXT  = 0x91
RESP_WRITE_DATA     = 0x84
RESP_BROADCAST_TIME = 0x8C
RESP_RESET          = 0x90
RESP_READ_ADDR      = 0x93

# 异常应答标志位 (数据域首字节)
ERR_NORMAL          = 0x00   # 正常
ERR_COMMAND         = 0x01   # 非法命令
ERR_CHECKSUM        = 0x02   # 校验错误
ERR_TIMEOUT         = 0x04   # 过载
ERR_LENGTH          = 0x08   # 长度错误
ERR_SEQUENCE        = 0x10   # 密码错误
ERR_NOT_SUPPORT     = 0x20   # 不支持

# 广播/通配地址
BROADCAST_ADDR = bytes([0x99] * 6)
BROADCAST_ADDR_AA = bytes([0xAA] * 6)

# 固定地址（忽略表号，响应该地址和任何地址）
DEFAULT_ADDR = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x01])


# ============================================================================
# 2. 数据字典 —— 常用数据标识 (DI)
#    DL/T 645-2007 中 DI 写作 DI3 DI2 DI1 DI0 (数值), 帧内按 DI0 DI1 DI2 DI3 传输
#    (即低字节在前), 所以帧内字节 = 数值字节倒序
# ============================================================================

def _di_frame(di_hex: str) -> bytes:
    """DI 数值 (如 '02030000') -> 帧内字节 (00 00 03 02)"""
    return bytes.fromhex(di_hex)[::-1]


# (DI数值, 名称, 单位, 字节数, 默认值字符串, 小数位, 类型['BCD'|'ASCII'])
DATA_DICT: List[Tuple[str, str, str, int, str, int, str]] = [
    # ---- 电能量 (BCD 4字节, 2位小数) ----
    ("00000000", "组合有功总电能",   "kWh",    4, "00120100", 2, "BCD"),
    ("00010000", "正向有功总电能",   "kWh",    4, "00123556", 2, "BCD"),
    ("00020000", "反向有功总电能",   "kWh",    4, "00003456", 2, "BCD"),
    ("00020300", "反向有功平电能",   "kWh",    4, "00000000", 2, "BCD"),
    ("00030000", "组合无功1总电能",  "kvarh",  4, "00056789", 2, "BCD"),
    ("00040000", "组合无功2总电能",  "kvarh",  4, "00034567", 2, "BCD"),
    ("00010100", "正向有功尖电能",   "kWh",    4, "00034500", 2, "BCD"),
    ("00010200", "正向有功峰电能",   "kWh",    4, "00045600", 2, "BCD"),
    ("00010300", "正向有功平电能",   "kWh",    4, "00023400", 2, "BCD"),
    ("00010400", "正向有功谷电能",   "kWh",    4, "00020056", 2, "BCD"),

    # ---- 电压 (BCD 3字节, 1位小数) ----
    ("02010100", "A相电压",         "V",      3, "02200",    1, "BCD"),
    ("02010200", "B相电压",         "V",      3, "02205",    1, "BCD"),
    ("02010300", "C相电压",         "V",      3, "02198",    1, "BCD"),

    # ---- 电流 (BCD 3字节, 3位小数) ----
    ("02020100", "A相电流",         "A",      3, "005000",   3, "BCD"),
    ("02020200", "B相电流",         "A",      3, "004980",   3, "BCD"),
    ("02020300", "C相电流",         "A",      3, "005020",   3, "BCD"),

    # ---- 功率因数 (BCD 2字节, 3位小数) —— 源数据, 负值表示功率为负 ----
    ("02060100", "A相功率因数",      "",       2, "1000",     3, "BCD"),
    ("02060200", "B相功率因数",      "",       2, "0980",     3, "BCD"),
    ("02060300", "C相功率因数",      "",       2, "0990",     3, "BCD"),

    # ---- 频率 (BCD 2字节, 2位小数) ----
    ("02800002", "电网频率",         "Hz",     2, "5000",     2, "BCD"),

    # ---- 其他 ----
    ("04000101", "日期及星期",       "",       4, "26102606", 0, "BCD"),
    ("04000102", "时间",             "",       3, "095442",   0, "BCD"),
    ("04000401", "通信地址",         "",       6, "000000000001", 0, "BCD"),
]

# 计算量: 由 电压/电流/分相功率因数 实时算出, 不可直接编辑
# 注意: 3字节BCD × 2位小数 = 可表示 ±9999.99 kW, 避免大功率时整数溢出
# (规约通常4位小数, 但调试场景下调为2位保证 ±10000kW 范围内都能正确表示)
COMPUTED_DICT: List[Tuple[str, str, str, int, int]] = [
    ("02030000", "瞬时总有功功率",  "kW",   3, 2),
    ("02030100", "A相有功功率",     "kW",   3, 2),
    ("02030200", "B相有功功率",     "kW",   3, 2),
    ("02030300", "C相有功功率",     "kW",   3, 2),
    ("02040000", "瞬时总无功功率",  "kvar", 3, 2),
    ("02050000", "瞬时总视在功率",  "kVA",  3, 2),
    ("02060000", "总功率因数",      "",     2, 3),
]

# 源数据 DI (参与功率计算)
DI_UA, DI_UB, DI_UC = "02010100", "02010200", "02010300"
DI_IA, DI_IB, DI_IC = "02020100", "02020200", "02020300"
DI_PFA, DI_PFB, DI_PFC = "02060100", "02060200", "02060300"


# ============================================================================
# 3. 协议引擎
# ============================================================================

def compute_checksum(data: bytes) -> int:
    """计算校验和 (模 256 加)"""
    return sum(data) & 0xFF


def encode_offset(data: bytes) -> bytes:
    """数据域 +33H 编码"""
    return bytes((b + OFFSET) & 0xFF for b in data)


def decode_offset(data: bytes) -> bytes:
    """数据域 -33H 解码"""
    return bytes((b - OFFSET) & 0xFF for b in data)


def build_frame(addr: bytes, ctrl: int, data: bytes) -> bytes:
    """
    构建 DLT 645-2007 帧
    结构: 68H | A5..A0 | 68H | C | L | DATA | CS | 16H
    data: 原始数据域 (自动 +33H 编码)
    """
    encoded_data = encode_offset(data)
    length = len(encoded_data)
    # 长度字节最高位: 是否跟时钟标 (这里不跟时钟, 置0)
    length_byte = length & 0x7F

    # 校验范围: 从帧起始符 68H 开始, 到数据域最后一字节 (含两个 68H)
    # 按规约: CS = (68H + A5..A0 + 68H + C + L + DATA[0] + ... + DATA[n]) mod 256
    check_range = bytes([HEADER]) + addr + bytes([HEADER, ctrl, length_byte]) + encoded_data
    cs = compute_checksum(check_range)

    frame = bytes([HEADER]) + addr + bytes([HEADER, ctrl, length_byte]) + encoded_data + bytes([cs, FOOTER])
    return frame


def parse_frame(raw: bytes) -> Optional[Dict]:
    """
    解析 DLT 645-2007 帧
    返回 dict: {addr, ctrl, length, data_raw, cs_ok, hex} 或 None
    """
    # 找帧头
    if len(raw) < 12:
        return None
    try:
        h1 = raw.index(HEADER)
    except ValueError:
        return None
    sub = raw[h1:]
    if len(sub) < 12:
        return None
    # 找第二个帧头（地址域6字节后）
    if sub[7] != HEADER:
        return None
    if sub[8] == FOOTER:
        return None

    addr = sub[1:7]
    ctrl = sub[8]
    length_byte = sub[9]
    length = length_byte & 0x7F
    has_time_flag = bool(length_byte & 0x80)

    # 数据域
    data_start = 10
    data_end = data_start + length
    if len(sub) < data_end + 2:  # +CS + FOOTER
        return None
    encoded_data = sub[data_start:data_end]
    cs = sub[data_end]
    footer = sub[data_end + 1]

    if footer != FOOTER:
        return None

    # 校验和 (含两个 68H 帧起始符)
    check_range = bytes([HEADER]) + addr + bytes([HEADER, ctrl, length_byte]) + encoded_data
    expected_cs = compute_checksum(check_range)
    cs_ok = (cs == expected_cs)

    data_raw = decode_offset(encoded_data)

    return {
        "addr": addr,
        "ctrl": ctrl,
        "length": length,
        "data_raw": data_raw,
        "cs_ok": cs_ok,
        "has_time_flag": has_time_flag,
        "cs_actual": cs,
        "cs_expected": expected_cs,
        "hex": sub[:data_end + 2].hex().upper(),
    }


def find_frames(buffer: bytes) -> Tuple[List[Dict], bytes]:
    """
    从字节流中切出完整帧，返回 (帧列表, 剩余缓冲)
    剩余缓冲保留所有非完整帧字节 (前导/间隙/尾部垃圾 + 不完整帧)
    """
    # 逐字节扫描, 找出所有完整帧的 (start, end)
    i = 0
    n = len(buffer)
    frame_ranges: List[Tuple[int, int]] = []
    while i < n:
        if buffer[i] == HEADER and i + 12 <= n:
            if buffer[i + 7] == HEADER:
                length = buffer[i + 9] & 0x7F
                total_len = 1 + 6 + 1 + 1 + 1 + length + 1 + 1
                if i + total_len <= n:
                    sub = buffer[i:i + total_len]
                    if sub[-1] == FOOTER and parse_frame(sub):
                        frame_ranges.append((i, i + total_len))
                        i += total_len
                        continue
        i += 1

    if not frame_ranges:
        return [], buffer

    # 从 buffer 中移除完整帧, 保留所有其他字节
    remaining = bytearray()
    last_end = 0
    for start, end in frame_ranges:
        remaining.extend(buffer[last_end:start])
        last_end = end
    remaining.extend(buffer[last_end:])

    # 解析每个完整帧
    parsed_frames = []
    for start, end in frame_ranges:
        pf = parse_frame(buffer[start:end])
        if pf:
            parsed_frames.append(pf)

    return parsed_frames, bytes(remaining)


# ============================================================================
# 4. 数据存储 (可编辑)
# ============================================================================

class MeterDataStore:
    """从设备数据存储"""

    def __init__(self):
        # 键为帧内字节顺序 (DI0 DI1 DI2 DI3)
        self._store: Dict[bytes, Tuple[str, str, int, str, int, str]] = {}
        for entry in DATA_DICT:
            di_hex, name, unit, length, val, dec, vtype = entry
            self._store[_di_frame(di_hex)] = (name, unit, length, val, dec, vtype)
        self._computed = {_di_frame(d[0]): d[1:] for d in COMPUTED_DICT}

    def is_computed(self, di: bytes) -> bool:
        return di in self._computed

    def _num(self, di_hex: str, dec: int) -> float:
        """取源数据的数值 (如 '02200', 1位小数 -> 220.0)"""
        info = self._store.get(_di_frame(di_hex))
        if not info:
            return 0.0
        val = info[3]
        neg = val.strip().startswith("-")
        digits = ''.join(c for c in val if c.isdigit()) or "0"
        n = int(digits) / (10 ** dec)
        return -n if neg else n

    def compute_value(self, di: bytes, deci: int) -> str:
        """按 电压×电流×功率因数 实时计算功率/功率因数"""
        ua, ub, uc = (self._num(DI_UA, 1), self._num(DI_UB, 1), self._num(DI_UC, 1))
        ia, ib, ic = (self._num(DI_IA, 3), self._num(DI_IB, 3), self._num(DI_IC, 3))
        pfa, pfb, pfc = (self._num(DI_PFA, 3), self._num(DI_PFB, 3), self._num(DI_PFC, 3))

        def sin_of(pf: float) -> float:
            s = (max(0.0, 1.0 - pf * pf)) ** 0.5
            return -s if pf < 0 else s

        pa, pb, pc = ua * ia * pfa, ub * ib * pfb, uc * ic * pfc          # W
        qa, qb, qc = ua * ia * sin_of(pfa), ub * ib * sin_of(pfb), uc * ic * sin_of(pfc)
        sa, sb, sc = ua * ia, ub * ib, uc * ic

        di_hex = di[::-1].hex().upper()
        if di_hex == "02030100":
            val = pa / 1000.0
        elif di_hex == "02030200":
            val = pb / 1000.0
        elif di_hex == "02030300":
            val = pc / 1000.0
        elif di_hex == "02030000":
            val = (pa + pb + pc) / 1000.0
        elif di_hex == "02040000":
            val = (qa + qb + qc) / 1000.0
        elif di_hex == "02050000":
            val = (sa + sb + sc) / 1000.0
        else:  # 02060000 总功率因数
            s_total = sa + sb + sc
            val = ((pa + pb + pc) / s_total) if s_total else 0.0

        n = int(round(val * (10 ** deci)))
        neg = n < 0
        length = self._computed[di][2]
        width = length * 2
        digits = str(abs(n)).rjust(width, "0")[-width:]
        return ("-" + digits) if neg else digits

    def get(self, di: bytes) -> Optional[str]:
        if di in self._store:
            return self._store[di][3]
        if di in self._computed:
            return self.compute_value(di, self._computed[di][3])
        return None

    def get_info(self, di: bytes):
        if di in self._store:
            return self._store[di]  # (name, unit, length, value_str, dec, vtype)
        if di in self._computed:
            name, unit, length, deci = self._computed[di]
            return (name, unit, length, self.compute_value(di, deci), deci, "BCD")
        return None

    def get_display(self, di_hex: str) -> str:
        """按 DI 数值 hex 取十进制显示字符串, 不存在返回 ''"""
        info = self.get_info(_di_frame(di_hex))
        if not info:
            return ""
        return self.dec_to_display(info[3], info[4])

    def set(self, di: bytes, value_str: str) -> bool:
        if di in self._store:
            name, unit, length, _, dec, vtype = self._store[di]
            self._store[di] = (name, unit, length, value_str, dec, vtype)
            return True
        return False

    # 走字目标 (组合, 总, 平): 用电走正向, 放电走反向 (光伏上网)
    WALK_FORWARD = ("00000000", "00010000", "00010300")
    WALK_REVERSE = ("00000000", "00020000", "00020300")

    def compute_total_power_kw(self) -> float:
        """总有功功率 (kW), 负值 = 放电/上网"""
        val = self.compute_value(_di_frame("02030000"), 4)
        neg = val.strip().startswith("-")
        digits = ''.join(c for c in val if c.isdigit()) or "0"
        p = int(digits) / 10000.0
        return -p if neg else p

    def add_energy(self, delta: float, forward: bool = True) -> None:
        """累加电量 (kWh), delta 取正值。
        forward=True  用电: 正向/平 +delta, 组合 +delta
        forward=False 放电: 反向/平 +delta, 组合 -delta (真实表正向不会倒退)"""
        dis = self.WALK_FORWARD if forward else self.WALK_REVERSE
        for idx, di_hex in enumerate(dis):
            info = self._store.get(_di_frame(di_hex))
            if not info:
                continue
            name, unit, length, _, dec, vtype = info
            cur = self._num(di_hex, dec)
            # 组合(idx==0): 用电 +delta / 放电 -delta; 其余总是 +delta
            d = (delta if forward else -delta) if idx == 0 else delta
            n = int(round((cur + d) * (10 ** dec)))
            if n < 0 and idx != 0:      # 正/反向寄存器不允许为负
                n = 0
            neg = n < 0
            width = length * 2
            s = str(abs(n)).rjust(width, "0")[-width:]
            if neg:
                s = "-" + s
            self.set(_di_frame(di_hex), s)

    def dec_to_display(self, val: str, dec: int) -> str:
        """内部 BCD 字符串 -> 带小数位的十进制显示 (如 '02200', 1 -> '220.0')"""
        neg = val.strip().startswith("-")
        digits = ''.join(c for c in val if c.isdigit()) or "0"
        if dec > 0 and len(digits) > dec:
            i = len(digits) - dec
            s = digits[:i] + "." + digits[i:]
        else:
            s = digits
        # 去尾零, 保留小数点后至少 dec 位的精度显示
        if "." in s:
            s = s.rstrip("0").rstrip(".")
        return ("-" + s) if neg else s

    def display_to_val(self, display: str, length: int, dec: int) -> str:
        """十进制输入 -> 内部 BCD 字符串 (如 '220.0', 3字节, 1位 -> '02200')"""
        neg = display.strip().startswith("-")
        # 去掉小数点, 凑齐小数位
        clean = ''.join(c for c in display if c.isdigit() or c == ".")
        if "." in clean:
            int_part, frac_part = clean.split(".", 1)
            frac_part = (frac_part + "0" * dec)[:dec]
        else:
            int_part, frac_part = clean, "0" * dec
        digits = int_part + frac_part
        # 凑齐总位数 (length * 2)
        width = length * 2
        digits = digits.rjust(width, "0")[-width:]
        return ("-" + digits) if neg else digits

    def all_items(self):
        for di, (name, unit, length, val, dec, vtype) in self._store.items():
            yield di, name, unit, length, val, dec, vtype
        for di, (name, unit, length, deci) in self._computed.items():
            yield di, name + "(计算)", unit, length, self.compute_value(di, deci), deci, "BCD"

    def keys(self):
        return list(self._store.keys())


# ============================================================================
# 5. 响应生成器
# ============================================================================

class SlaveResponder:
    """从站响应逻辑"""

    def __init__(self, store: MeterDataStore):
        self.store = store
        self._addr = DEFAULT_ADDR
        self._cur_addr = DEFAULT_ADDR   # 本次应答使用的地址
        self.on_store_changed = lambda: None  # 写数据后通知 GUI 刷新

    def set_addr(self, addr: bytes):
        self._addr = addr

    def handle(self, req: Dict) -> bytes:
        ctrl = req["ctrl"] & 0x7F  # 去掉非法位
        data = req["data_raw"]
        addr = req["addr"]

        # 应答地址: 非广播地址时原样回送请求地址(等效忽略表号); 广播时用本机地址
        if addr in (BROADCAST_ADDR, BROADCAST_ADDR_AA):
            self._cur_addr = self._addr
        else:
            self._cur_addr = addr

        try:
            return self._dispatch(ctrl, data, addr, req)
        except Exception:
            return self._build_error(req["ctrl"], addr, ERR_COMMAND, req["data_raw"])

    def _dispatch(self, ctrl: int, data: bytes, addr: bytes, req: Dict) -> bytes:
        if ctrl == CMD_READ_DATA:        # 0x01 读数据
            return self._cmd_read_data(req, data, RESP_READ_DATA)
        elif ctrl == CMD_READ_DATA_EXT:  # 0x11 主站请求读电能表数据(扩展码)
            return self._cmd_read_data(req, data, RESP_READ_DATA_EXT)
        elif ctrl == CMD_READ_NEXT:      # 0x02
            return self._cmd_read_next(req, data)
        elif ctrl == CMD_READ_BLOCK:     # 0x03
            return self._cmd_read_block(req, data)
        elif ctrl == CMD_WRITE_DATA:     # 0x04
            return self._cmd_write_data(req, data)
        elif ctrl == CMD_WRITE_BLOCK:    # 0x05
            return self._cmd_write_block(req, data)
        elif ctrl == CMD_BROADCAST_TIME:  # 0x0C
            return self._cmd_broadcast_time(req, data)
        elif ctrl == CMD_RESET:          # 0x10
            return self._cmd_reset(req, data)
        elif ctrl == CMD_READ_ADDR:      # 0x13 读通信地址
            return self._cmd_read_address(req)
        else:
            return self._build_error(ctrl | 0x80, addr, ERR_COMMAND, data)

    # ---------- 读数据 (0x01 / 0x11) ----------
    def _cmd_read_data(self, req: Dict, data: bytes, resp_ctrl: int = RESP_READ_DATA) -> bytes:
        if len(data) < 4:
            return self._build_error(resp_ctrl, req["addr"], ERR_LENGTH, data)
        di = data[:4]
        extra = data[4:]  # 附加数据 (需原样返回)

        info = self.store.get_info(di)
        if info is None:
            return self._build_error(resp_ctrl, req["addr"], ERR_NOT_SUPPORT, data)
        name, unit, length, val_str, dec, vtype = info

        payload = di + self._value_to_bytes(val_str, length, vtype) + extra
        return build_frame(self._cur_addr, resp_ctrl, payload)

    # ---------- 读通信地址 0x13 ----------
    def _cmd_read_address(self, req: Dict) -> bytes:
        # 应答: 本机通信地址 6 字节 (BCD)
        info = self.store.get_info(_di_frame("04000401"))
        val_str = info[3] if info else ""
        payload = self._value_to_bytes(val_str, 6, "BCD")
        return build_frame(self._cur_addr, RESP_READ_ADDR, payload)

    # ---------- 读后续数据 0x02 ----------
    def _cmd_read_next(self, req: Dict, data: bytes) -> bytes:
        # 简化处理：返回空数据
        payload = bytes()
        return build_frame(self._cur_addr, RESP_READ_DATA, payload)

    # ---------- 读数据块 0x03 ----------
    def _cmd_read_block(self, req: Dict, data: bytes) -> bytes:
        if len(data) < 2:
            return self._build_error(0x83, req["addr"], ERR_LENGTH, data)
        count = data[0]
        payload = bytes([count])
        pos = 1
        for _ in range(count):
            if pos + 4 > len(data):
                break
            di = data[pos:pos + 4]
            pos += 4
            info = self.store.get_info(di)
            if info:
                name, unit, length, val_str, dec, vtype = info
                payload += di + self._value_to_bytes(val_str, length, vtype)
            else:
                payload += di + bytes(4)  # 未知标识填0
        return build_frame(self._cur_addr, 0x83, payload)

    # ---------- 写数据 0x04 ----------
    def _cmd_write_data(self, req: Dict, data: bytes) -> bytes:
        if len(data) < 10:
            return self._build_error(RESP_WRITE_DATA, req["addr"], ERR_LENGTH, data)
        # 数据域: 密码(6) + DI(4) + VALUE
        di = data[6:10]
        value_raw = data[10:]
        info = self.store.get_info(di)
        if info:
            name, unit, length, _, dec, vtype = info
            if len(value_raw) == length:
                new_val = self._bytes_to_value(value_raw, vtype)
                self.store.set(di, new_val)
                try:
                    self.on_store_changed()  # 通知 GUI 刷新
                except Exception:
                    pass
        # 原样回送 (规约要求)
        payload = data
        return build_frame(self._cur_addr, RESP_WRITE_DATA, payload)

    @staticmethod
    def _bytes_to_value(raw: bytes, vtype: str = "BCD") -> str:
        """将收到的字节数据解码为值字符串 (带符号: 最高字节 D7 为符号位)"""
        if vtype == "ASCII":
            # ASCII 字符串, 去掉尾部 0x00
            return raw.decode("ascii", errors="replace").rstrip("\x00")
        else:
            # BCD: 每字节高低4位组合; 最高字节(帧内最后一字节)的 D7 为符号位
            negative = bool(raw) and bool(raw[-1] & 0x80)
            digits = ""
            for i, b in enumerate(raw):
                if i == len(raw) - 1:
                    b &= 0x7F
                digits += str((b >> 4) & 0x0F)
                digits += str(b & 0x0F)
            digits = digits.lstrip("0") or "0"
            return ("-" + digits) if negative else digits

    # ---------- 写数据块 0x05 ----------
    def _cmd_write_block(self, req: Dict, data: bytes) -> bytes:
        payload = data
        return build_frame(self._cur_addr, 0x85, payload)

    # ---------- 广播校时 0x0C ----------
    def _cmd_broadcast_time(self, req: Dict, data: bytes) -> bytes:
        # 广播校时如果DITime=1 (从站需应答)，则应答; DITime=0 无应答
        # 简化：始终应答
        return build_frame(self._cur_addr, RESP_BROADCAST_TIME, data)

    # ---------- 复位 0x10 ----------
    def _cmd_reset(self, req: Dict, data: bytes) -> bytes:
        return build_frame(self._cur_addr, RESP_RESET, data)

    # ---------- 异常应答 ----------
    def _build_error(self, ctrl: int, addr: bytes, err_code: int, echo: bytes) -> bytes:
        # 数据域: err_code + 原请求数据域
        payload = bytes([err_code]) + echo
        return build_frame(self._cur_addr, ctrl, payload)

    # ---------- 值转换 ----------
    @staticmethod
    def _value_to_bytes(val_str: str, length: int, vtype: str = "BCD") -> bytes:
        """
        将值字符串转为定长字节数据
        val_str: 值字符串
        length: 期望字节数
        vtype:  'BCD' 或 'ASCII'
        """
        if vtype == "ASCII":
            # ASCII 编码, 截断或右侧补 0x00
            raw = val_str.encode("ascii", errors="replace")
            if len(raw) < length:
                raw = raw + bytes(length - len(raw))
            elif len(raw) > length:
                raw = raw[:length]
            return raw
        else:
            # BCD 编码: 支持前导 '-' 表示负数 (最高字节 D7 置 1)
            negative = val_str.strip().startswith("-")
            digits = ''.join(c for c in val_str if c.isdigit())
            need_digits = length * 2
            if len(digits) < need_digits:
                digits = '0' * (need_digits - len(digits)) + digits
            elif len(digits) > need_digits:
                digits = digits[-need_digits:]
            result = bytearray()
            for i in range(0, len(digits), 2):
                high = int(digits[i])
                low = int(digits[i + 1])
                result.append((high << 4) | low)
            if negative and result:
                result[-1] |= 0x80   # 最高字节(帧内最后一字节) D7 = 符号位
            return bytes(result)


# ============================================================================
# 6. 串口监听器
# ============================================================================

class SerialListener(threading.Thread):
    def __init__(self, port: str, baud: int, responder: SlaveResponder,
                 on_rx, on_tx, on_status):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.responder = responder
        self.on_rx = on_rx      # callback(parsed_frame_dict, raw_bytes)
        self.on_tx = on_tx      # callback(response_bytes)
        self.on_status = on_status  # callback(msg)
        self._stop = threading.Event()
        self.ser: Optional[serial.Serial] = None
        self.buffer = b""

    def run(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.3,
                write_timeout=1.0,
            )
        except Exception as e:
            self.on_status(f"[错误] 打开串口失败: {e}")
            return

        self.on_status(f"[就绪] 已打开 {self.port} @ {self.baud}bps")

        try:
            while not self._stop.is_set():
                try:
                    if self.ser.is_open and self.ser.in_waiting > 0:
                        chunk = self.ser.read(self.ser.in_waiting)
                        if chunk:
                            self.buffer += chunk
                            self.on_rx(None, chunk)
                            frames, self.buffer = find_frames(self.buffer)
                            for f in frames:
                                self._handle_frame(f)
                    else:
                        # 短 sleep, 同时能被 stop 快速打断
                        time.sleep(0.02)
                except serial.SerialException:
                    # 串口被关闭或出错, 退出循环
                    break
                except OSError:
                    break
                except Exception as e:
                    self.on_status(f"[异常] {e}")
                    time.sleep(0.1)
        finally:
            self._close()

    def _handle_frame(self, parsed: Dict):
        self.on_rx(parsed, None)
        if not parsed["cs_ok"]:
            self.on_status(f"[警告] 校验失败 (计算={parsed['cs_expected']:02X} 收到={parsed['cs_actual']:02X})")
        try:
            resp = self.responder.handle(parsed)
        except Exception as e:
            self.on_status(f"[错误] 处理异常: {e}")
            return
        if resp and self.ser and self.ser.is_open:
            try:
                self.ser.write(resp)
                self.ser.flush()
                self.on_tx(resp)
            except Exception as e:
                self.on_status(f"[错误] 发送失败: {e}")

    def stop(self):
        """请求停止并等待线程退出 (最多 2 秒)"""
        self._stop.set()
        # 由 run() 的 finally 负责关闭串口, 不在此直接 close
        if self.is_alive():
            self.join(timeout=2.0)
        # 兜底: 如果线程还没退出, 强制关闭
        self._close()

    def _close(self):
        """关闭串口 (幂等)"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.on_status(f"[关闭] 串口 {self.port} 已关闭")
            except Exception:
                pass
        self.ser = None


# ============================================================================
# 7. Tkinter GUI
# ============================================================================

class SlaveApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("DL/T 645-2007 调试工具 - Slave模拟器")
        self.root.geometry("960x720")

        self.store = MeterDataStore()
        self.responder = SlaveResponder(self.store)
        self.listener: Optional[SerialListener] = None

        # 写数据后刷新表格
        self.responder.on_store_changed = self._refresh_tree

        self._build_ui()
        self._refresh_port_list()

    # ---------- UI 构建 ----------
    def _build_ui(self):
        # 顶部: 串口配置
        cfg = ttk.LabelFrame(self.root, text="串口配置", padding=8)
        cfg.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(8, 4))

        ttk.Label(cfg, text="端口:").grid(row=0, column=0, sticky=tk.E)
        self.cmb_port = ttk.Combobox(cfg, width=10, state="readonly")
        self.cmb_port.grid(row=0, column=1, padx=4)
        ttk.Button(cfg, text="刷新", command=self._refresh_port_list).grid(row=0, column=2, padx=4)

        ttk.Label(cfg, text="波特率:").grid(row=0, column=3, sticky=tk.E)
        self.cmb_baud = ttk.Combobox(cfg, width=8, state="readonly",
                                     values=["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"])
        self.cmb_baud.set("9600")
        self.cmb_baud.grid(row=0, column=4, padx=4)

        ttk.Label(cfg, text="数据位:").grid(row=0, column=5, sticky=tk.E)
        ttk.Label(cfg, text="8").grid(row=0, column=6, padx=4)
        ttk.Label(cfg, text="校验位:").grid(row=0, column=7, sticky=tk.E)
        ttk.Label(cfg, text="无").grid(row=0, column=8, padx=4)
        ttk.Label(cfg, text="停止位:").grid(row=0, column=9, sticky=tk.E)
        ttk.Label(cfg, text="1").grid(row=0, column=10, padx=4)

        self.btn_open = ttk.Button(cfg, text="打开串口", command=self._toggle_port)
        self.btn_open.grid(row=0, column=11, padx=12)

        ttk.Label(cfg, text="  表地址:").grid(row=0, column=12, sticky=tk.E)
        self.var_addr = tk.StringVar(value="000000000001")
        ent_addr = ttk.Entry(cfg, textvariable=self.var_addr, width=14)
        ent_addr.grid(row=0, column=13, padx=4)
        ent_addr.bind("<KeyRelease>", self._on_addr_change)

        # 第二行: 模拟走字
        self.var_auto_walk = tk.BooleanVar(value=False)
        ttk.Checkbutton(cfg, text="自动走字", variable=self.var_auto_walk,
                        command=self._on_auto_walk).grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=(6, 0))

        ttk.Label(cfg, text="每次(kWh):").grid(row=1, column=2, sticky=tk.E, pady=(6, 0))
        self.var_walk_step = tk.StringVar(value="0.01")
        ttk.Entry(cfg, textvariable=self.var_walk_step, width=10).grid(row=1, column=3, padx=4, pady=(6, 0))

        ttk.Label(cfg, text="间隔(s):").grid(row=1, column=4, sticky=tk.E, pady=(6, 0))
        self.var_walk_interval = tk.StringVar(value="1.0")
        ttk.Entry(cfg, textvariable=self.var_walk_interval, width=8).grid(row=1, column=5, padx=4, pady=(6, 0))

        ttk.Button(cfg, text="走一次", command=self._walk_once).grid(row=1, column=6, padx=8, pady=(6, 0))
        self.var_pv_mode = tk.BooleanVar(value=False)
        ttk.Checkbutton(cfg, text="光伏模式", variable=self.var_pv_mode).grid(row=1, column=7, sticky=tk.W, pady=(6, 0))
        ttk.Label(cfg, text="用电走正向, 放电走反向",
                  foreground="#7f8c8d").grid(row=1, column=8, columnspan=6, sticky=tk.W, pady=(6, 0))

        # 中间: Notebook (数据/日志 与 电表面板)
        nb = ttk.Notebook(self.root)
        nb.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        # 页1: 数据字典 + 日志
        page1 = ttk.Frame(nb)
        nb.add(page1, text=" 数据配置 ")
        mid = ttk.Panedwindow(page1, orient=tk.HORIZONTAL)
        mid.pack(fill=tk.BOTH, expand=True)

        # 左: 数据字典表格
        dict_frame = ttk.LabelFrame(mid, text="数据表 (双击数值可编辑)", padding=4)
        mid.add(dict_frame, weight=3)

        cols = ("di", "name", "unit", "bytes", "value", "dec")
        self.tree = ttk.Treeview(dict_frame, columns=cols, show="headings", height=14)
        headings = [("di", "数据标识 DI", 110), ("name", "名称", 160),
                    ("unit", "单位", 60), ("bytes", "字节数", 60),
                    ("value", "BCD字符串", 120), ("dec", "十进制值(双击可改)", 130)]
        for c, text, w in headings:
            self.tree.heading(c, text=text)
            self.tree.column(c, width=w, anchor=tk.W)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb = ttk.Scrollbar(dict_frame, orient=tk.VERTICAL, command=self.tree.yview)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.bind("<Double-1>", self._on_tree_double)

        # 右: 日志
        log_frame = ttk.LabelFrame(mid, text="通信日志 (HEX)", padding=4)
        mid.add(log_frame, weight=2)

        btn_row = ttk.Frame(log_frame)
        btn_row.pack(fill=tk.X)
        ttk.Button(btn_row, text="清空日志", command=self._clear_log).pack(side=tk.LEFT)
        self.var_autoscroll = tk.BooleanVar(value=True)
        ttk.Checkbutton(btn_row, text="自动滚动", variable=self.var_autoscroll).pack(side=tk.LEFT, padx=8)

        self.txt_log = scrolledtext.ScrolledText(log_frame, height=14, font=("Consolas", 10))
        self.txt_log.pack(fill=tk.BOTH, expand=True)
        self.txt_log.tag_config("rx", foreground="#c0392b")
        self.txt_log.tag_config("tx", foreground="#27ae60")
        self.txt_log.tag_config("info", foreground="#7f8c8d")
        self.txt_log.tag_config("err", foreground="#c0392b", font=("Consolas", 10, "bold"))

        # 页2: 电表面板
        page2 = ttk.Frame(nb)
        nb.add(page2, text=" 电表面板 ")
        self._build_meter_panel(page2)

        # 底部: 状态栏
        self.var_status = tk.StringVar(value="就绪")
        status_bar = ttk.Label(self.root, textvariable=self.var_status, anchor=tk.W, relief=tk.SUNKEN)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(4, 8))

        # 填充数据字典
        self._fill_tree()
        self._panel_tick()

    def _fill_tree(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for di, name, unit, length, val, dec, vtype in self.store.all_items():
            di_str = di[::-1].hex().upper()
            dec_str = self.store.dec_to_display(val, dec)
            self.tree.insert("", tk.END, iid=di_str,
                             values=(di_str, name, unit, length, val, dec_str))

    def _refresh_tree(self):
        for di, name, unit, length, val, dec, vtype in self.store.all_items():
            di_str = di[::-1].hex().upper()
            if self.tree.exists(di_str):
                dec_str = self.store.dec_to_display(val, dec)
                self.tree.set(di_str, "value", val)
                self.tree.set(di_str, "dec", dec_str)

    def _clear_log(self):
        self.txt_log.delete("1.0", tk.END)

    # ---------- 电表面板 (LCD 仿真) ----------
    LCD_BG = "#9fb3a3"          # 液晶屏底色 (灰绿)
    LCD_FG = "#1a2b20"          # 字符色 (深绿黑)
    LCD_FRAME = "#3a4a41"       # 屏幕边框

    def _build_meter_panel(self, parent):
        """仿真电表 LCD 显示页: 上半 LCD 屏, 下半分组卡片"""
        wrap = tk.Frame(parent, bg="#e8ecef")
        wrap.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # ============ 上半: LCD 屏 ============
        lcd_outer = tk.Frame(wrap, bg=self.LCD_FRAME, bd=0)
        lcd_outer.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))

        # LCD 主屏 (大数字 + 单位 + 标签)
        lcd = tk.Frame(lcd_outer, bg=self.LCD_BG)
        lcd.pack(fill=tk.X, padx=10, pady=10)

        # 顶行小字
        top_row = tk.Frame(lcd, bg=self.LCD_BG)
        top_row.pack(fill=tk.X, padx=12, pady=(10, 0))
        self.lcd_lbl_rate = tk.Label(top_row, text="● 当前费率: 平", bg=self.LCD_BG, fg=self.LCD_FG,
                                     font=("Microsoft YaHei", 10, "bold"))
        self.lcd_lbl_rate.pack(side=tk.LEFT)
        self.lcd_lbl_dir = tk.Label(top_row, text="◆ 用电", bg=self.LCD_BG, fg=self.LCD_FG,
                                    font=("Microsoft YaHei", 10, "bold"))
        self.lcd_lbl_dir.pack(side=tk.RIGHT)

        # 大数字
        num_row = tk.Frame(lcd, bg=self.LCD_BG)
        num_row.pack(fill=tk.X, padx=12, pady=(4, 0))
        self.lcd_lbl_main = tk.Label(num_row, text="0.00", bg=self.LCD_BG, fg=self.LCD_FG,
                                     font=("Courier New", 48, "bold"))
        self.lcd_lbl_main.pack(side=tk.LEFT)
        tk.Label(num_row, text="kWh", bg=self.LCD_BG, fg=self.LCD_FG,
                 font=("Microsoft YaHei", 16, "bold")).pack(side=tk.LEFT, padx=(8, 0), pady=(16, 0))
        # 点击大数字区域 → 翻到下一费率
        self.lcd_lbl_main.bind("<Button-1>", lambda e: self._next_rate())

        # 副显示: 瞬时功率 / 电压 / 电流 / 功率因数
        sub_row = tk.Frame(lcd, bg=self.LCD_BG)
        sub_row.pack(fill=tk.X, padx=12, pady=(6, 4))
        self.lcd_lbl_power = tk.Label(sub_row, text="P = 0.000 kW",
                                      bg=self.LCD_BG, fg=self.LCD_FG,
                                      font=("Courier New", 14, "bold"))
        self.lcd_lbl_power.pack(side=tk.LEFT)

        # 费率按钮 (总量 + 尖峰平谷), 点击切换主屏显示
        rate_row = tk.Frame(lcd, bg=self.LCD_BG)
        rate_row.pack(fill=tk.X, padx=12, pady=(0, 8))
        self.rate_order = ["总", "尖", "峰", "平", "谷"]
        self.rate_di_map = {
            "总": "00000000",   # 组合有功总电能
            "尖": "00010100",   # 正向有功尖电能
            "峰": "00010200",   # 正向有功峰电能
            "平": "00010300",   # 正向有功平电能
            "谷": "00010400",   # 正向有功谷电能
        }
        self.current_rate = "总"
        self._cycle_pause_until = 0.0
        self._cycle_count = 0
        self.rate_btns = {}
        for tag in self.rate_order:
            lbl = tk.Label(rate_row, text=f" {tag} ", bg=self.LCD_BG, fg=self.LCD_FG,
                           font=("Microsoft YaHei", 10, "bold"),
                           relief=tk.RIDGE, bd=1, padx=8, pady=2, cursor="hand2")
            lbl.pack(side=tk.LEFT, padx=2)
            lbl.bind("<Button-1>", lambda e, t=tag: self._select_rate(t))
            self.rate_btns[tag] = lbl
        self._apply_rate_highlight()

        # 通信状态条: 在此页也能看到是否收到/发出报文
        self.rx_count = 0
        self.tx_count = 0
        self._last_comm = "等待主站报文..."
        self.lbl_comm = tk.Label(wrap, text="RX: 0 帧   TX: 0 帧   等待主站报文...",
                                 bg="#e8ecef", fg="#7f8c8d",
                                 font=("Microsoft YaHei", 9), anchor=tk.W)
        self.lbl_comm.pack(side=tk.BOTTOM, fill=tk.X, pady=(6, 0))

        # ============ 下半: 分组卡片 ============
        cards = tk.Frame(wrap, bg="#e8ecef")
        cards.pack(fill=tk.BOTH, expand=True)
        # 左列: 电能组
        left = tk.Frame(cards, bg="#e8ecef")
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))
        self.grp_energy = self._make_group(left, "电能量 (kWh)")
        self.energy_lbls = {}
        for key, di in (("组合有功总电能", "00000000"),
                        ("正向有功总电能", "00010000"),
                        ("反向有功总电能", "00020000"),
                        ("正向有功尖电能", "00010100"),
                        ("正向有功峰电能", "00010200"),
                        ("正向有功平电能", "00010300"),
                        ("正向有功谷电能", "00010400"),
                        ("反向有功平电能", "00020300")):
            self.energy_lbls[di] = self._make_row(self.grp_energy, key, di, "kWh")

        # 右列: 电气量组
        right = tk.Frame(cards, bg="#e8ecef")
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(6, 0))
        self.grp_elec = self._make_group(right, "电气量")
        self.elec_lbls = {}
        for key, di, unit in (("A相电压", "02010100", "V"),
                              ("B相电压", "02010200", "V"),
                              ("C相电压", "02010300", "V"),
                              ("A相电流", "02020100", "A"),
                              ("B相电流", "02020200", "A"),
                              ("C相电流", "02020300", "A"),
                              ("A相功率因数", "02060100", ""),
                              ("B相功率因数", "02060200", ""),
                              ("C相功率因数", "02060300", ""),
                              ("瞬时总有功功率", "02030000", "kW"),
                              ("瞬时总无功功率", "02040000", "kvar"),
                              ("瞬时总视在功率", "02050000", "kVA")):
            self.elec_lbls[di] = self._make_row(self.grp_elec, key, di, unit)

    def _make_group(self, parent, title):
        g = tk.LabelFrame(parent, text=title, bg="#ffffff",
                          font=("Microsoft YaHei", 10, "bold"),
                          padx=8, pady=6, bd=1, relief=tk.GROOVE)
        g.pack(fill=tk.BOTH, expand=True)
        return g

    def _make_row(self, parent, label, di, unit):
        row = tk.Frame(parent, bg="#ffffff")
        row.pack(fill=tk.X, pady=1)
        tk.Label(row, text=label, bg="#ffffff", fg="#2c3e50",
                 font=("Microsoft YaHei", 9), width=14, anchor=tk.W).pack(side=tk.LEFT)
        val_lbl = tk.Label(row, text="--", bg="#ffffff", fg="#16a085",
                           font=("Courier New", 11, "bold"), width=14, anchor=tk.E)
        val_lbl.pack(side=tk.LEFT, padx=4)
        tk.Label(row, text=unit, bg="#ffffff", fg="#7f8c8d",
                 font=("Microsoft YaHei", 9), width=5, anchor=tk.W).pack(side=tk.LEFT)
        return val_lbl

    def _select_rate(self, tag):
        """点击费率按钮: 切换主屏显示, 并暂停自动循环 10 秒"""
        self.current_rate = tag
        self._cycle_pause_until = time.time() + 10
        self._cycle_count = 0
        self._apply_rate_highlight()
        self._update_panel()

    def _next_rate(self):
        """翻到下一费率 (点击屏幕大数字时触发)"""
        idx = (self.rate_order.index(self.current_rate) + 1) % len(self.rate_order)
        self._select_rate(self.rate_order[idx])

    def _apply_rate_highlight(self):
        """高亮当前选中的费率按钮"""
        for tag, lbl in self.rate_btns.items():
            if tag == self.current_rate:
                lbl.config(bg="#d5e6d8", fg=self.LCD_FG, relief=tk.SUNKEN, bd=2)
            else:
                lbl.config(bg=self.LCD_BG, fg=self.LCD_FG, relief=tk.RIDGE, bd=1)

    def _panel_tick(self):
        """每秒刷新一次 LCD 面板, 每 5 秒自动循环切换费率"""
        if time.time() >= self._cycle_pause_until:
            self._cycle_count += 1
            if self._cycle_count >= 5:
                self._cycle_count = 0
                idx = (self.rate_order.index(self.current_rate) + 1) % len(self.rate_order)
                self.current_rate = self.rate_order[idx]
                self._apply_rate_highlight()
        try:
            self._update_panel()
        except Exception as e:
            # 不再静默吞掉异常, 写入日志便于排查
            try:
                self._on_status(f"[面板刷新异常] {e}")
            except Exception:
                pass
        self.root.after(1000, self._panel_tick)

    def _update_panel(self):
        # 主屏: 当前选中费率的电能值
        di = self.rate_di_map.get(self.current_rate, "00000000")
        self.lcd_lbl_main.config(text=self.store.get_display(di) or "0.00")
        self.lcd_lbl_rate.config(text=f"● 当前显示: {self.current_rate}")

        # 瞬时功率
        p = self.store.compute_total_power_kw()
        self.lcd_lbl_power.config(text=f"P = {p:+.3f} kW")

        # 用电/放电方向
        if self.var_pv_mode.get():
            self.lcd_lbl_dir.config(text="◆ 放电 (光伏上网)")
        else:
            self.lcd_lbl_dir.config(text="◆ 用电")

        # 电能组
        for di, lbl in self.energy_lbls.items():
            lbl.config(text=self.store.get_display(di) or "--")
        # 电气量组
        for di, lbl in self.elec_lbls.items():
            lbl.config(text=self.store.get_display(di) or "--")

    def _refresh_port_list(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cmb_port["values"] = ports
        if ports and not self.cmb_port.get():
            self.cmb_port.set(ports[0])

    # ---------- 串口控制 ----------
    def _toggle_port(self):
        if self.listener and self.listener.is_alive():
            self.listener.stop()
            self.listener = None
            self.btn_open.config(text="打开串口")
        else:
            port = self.cmb_port.get()
            if not port:
                messagebox.showwarning("提示", "请选择串口")
                return
            baud = int(self.cmb_baud.get())
            self.btn_open.config(text="关闭串口")
            self._update_addr_from_entry()
            self.listener = SerialListener(
                port, baud, self.responder,
                on_rx=self._on_rx, on_tx=self._on_tx, on_status=self._on_status,
            )
            self.listener.start()

    # ---------- 模拟走字 ----------
    def _walk_once(self):
        if self.var_pv_mode.get():
            # 光伏模式: 电量 = 实际功率 x 间隔时间 (真实电表行为)
            try:
                interval = float(self.var_walk_interval.get())
            except ValueError:
                interval = 1.0
            p = self.store.compute_total_power_kw()   # kW, 负 = 放电
            delta = abs(p) * max(interval, 0.1) / 3600.0
            forward = p >= 0
            self.store.add_energy(delta, forward=forward)
            self._refresh_tree()
            fw = int(self.store.get(_di_frame("00010000")) or "0") / 100
            rv = int(self.store.get(_di_frame("00020000")) or "0") / 100
            state = "用电→正向累加" if forward else "放电→反向累加"
            self._on_status(f"[走字] P={p:.4f} kW {state}, 正向={fw:.2f} 反向={rv:.2f} kWh")
        else:
            try:
                step = float(self.var_walk_step.get())
            except ValueError:
                step = 0.01
            self.store.add_energy(step, forward=True)
            self._refresh_tree()
            ei = self.store.get(_di_frame("00000000"))
            self._on_status(f"[走字] +{step} kWh, 组合有功总 = {int(ei) / 100:.2f} kWh")

    def _on_auto_walk(self):
        if self.var_auto_walk.get():
            self._walk_once()          # 立即走一步, 之后按间隔循环
            self._schedule_walk()
        else:
            self._on_status("[走字] 已停止自动走字")

    def _schedule_walk(self):
        if not self.var_auto_walk.get():
            return
        try:
            interval = float(self.var_walk_interval.get())
        except ValueError:
            interval = 1.0
        interval = max(0.1, min(interval, 3600.0))
        self.root.after(int(interval * 1000), self._walk_tick)

    def _walk_tick(self):
        if not self.var_auto_walk.get():
            return
        self._walk_once()
        self._schedule_walk()

    # ---------- 地址 ----------
    def _on_addr_change(self, _evt=None):
        self._update_addr_from_entry()

    def _update_addr_from_entry(self):
        hex_str = self.var_addr.get().strip()
        # 补零或截断到6字节(12 hex字符)
        hex_str = ''.join(c for c in hex_str if c in '0123456789abcdefABCDEF')
        if len(hex_str) < 12:
            hex_str = '0' * (12 - len(hex_str)) + hex_str
        elif len(hex_str) > 12:
            hex_str = hex_str[:12]
        try:
            addr = bytes.fromhex(hex_str)
            self.responder.set_addr(addr)
        except Exception:
            pass

    # ---------- 数据编辑 ----------
    def _on_tree_double(self, event):
        item = self.tree.identify_row(event.y)
        col = self.tree.identify_column(event.x)
        if not item or col not in ("#5", "#6"):  # 第5列 BCD / 第6列 十进制
            return
        if self.store.is_computed(bytes.fromhex(item)[::-1]):
            self._on_status("[提示] 该项由 电压×电流×功率因数 实时计算, 请修改源数据")
            return
        bbox = self.tree.bbox(item, col)
        if not bbox:
            return
        x, y, w, h = bbox
        # 从 tree 里取 length/dec 用于转换
        vals = self.tree.item(item, "values")   # (di, name, unit, bytes, value, dec)
        length = int(vals[3])
        cur_val = vals[4]
        cur_dec = vals[5]
        # 取源数据的真实小数位和类型
        info = self.store.get_info(bytes.fromhex(item)[::-1])
        dec = info[4] if info else 0
        vtype = info[5] if info else "BCD"

        use_dec_col = (col == "#6")   # 双击十进制列?
        old_display = cur_dec if use_dec_col else cur_val
        entry = ttk.Entry(self.tree)
        entry.place(x=x, y=y, width=w, height=h)
        entry.insert(0, old_display)
        entry.focus_set()

        def commit(_evt=None):
            new_raw = entry.get().strip()
            try:
                di = bytes.fromhex(item)[::-1]
                if use_dec_col:
                    # 十进制输入 -> BCD 字符串 (仅 BCD 类型支持)
                    if vtype != "BCD":
                        self._on_status(f"[提示] {info[0] if info else ''} 为 ASCII 类型, 请改 BCD 字符串列")
                        entry.destroy()
                        return
                    new_val = self.store.display_to_val(new_raw, length, dec)
                else:
                    new_val = new_raw
                self.store.set(di, new_val)
                self._refresh_tree()
            except Exception as e:
                self._on_status(f"[错误] 设置值失败: {e}")
            entry.destroy()

        entry.bind("<Return>", commit)
        entry.bind("<FocusOut>", commit)
        entry.bind("<Escape>", lambda e: entry.destroy())

    # ---------- 日志回调 ----------
    def _on_rx(self, parsed: Optional[Dict], raw: Optional[bytes]):
        ts = datetime.now().strftime("%H:%M:%S.") + f"{int(time.time()*1000)%1000:03d}"
        if parsed:
            ctrl_name = self._ctrl_name(parsed["ctrl"])
            addr_str = parsed["addr"].hex().upper()
            data_str = parsed["data_raw"].hex().upper() if parsed["data_raw"] else ""
            cs_mark = "✓" if parsed["cs_ok"] else "✗"
            line = f"[{ts}] RX  {parsed['hex']}  | 控制={ctrl_name} 地址={addr_str} 数据={data_str} CS{cs_mark}\n"
        else:
            line = f"[{ts}] RX  (原始) {raw.hex().upper()}\n"
        self.rx_count += 1
        self._last_comm = f"最后收: {ts}"
        self.root.after(0, lambda: self._append_log(line, "rx"))
        self.root.after(0, self._refresh_comm_label)

    def _on_tx(self, data: bytes):
        ts = datetime.now().strftime("%H:%M:%S.") + f"{int(time.time()*1000)%1000:03d}"
        line = f"[{ts}] TX  {data.hex().upper()}\n"
        self.tx_count += 1
        self._last_comm = f"最后发: {ts}"
        self.root.after(0, lambda: self._append_log(line, "tx"))
        self.root.after(0, self._refresh_comm_label)

    def _refresh_comm_label(self):
        """刷新电表面板页的收发状态条"""
        state = "正常应答中" if self.tx_count else ("已收到请求, 但未发出应答" if self.rx_count else "等待主站报文...")
        color = "#27ae60" if self.tx_count else ("#c0392b" if self.rx_count else "#7f8c8d")
        self.lbl_comm.config(
            text=f"RX: {self.rx_count} 帧   TX: {self.tx_count} 帧   {state}   {self._last_comm}",
            fg=color)

    def _on_status(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.root.after(0, lambda: self._append_log(f"[{ts}] {msg}\n", "info"))
        self.root.after(0, lambda: self.var_status.set(msg))

    def _append_log(self, text: str, tag: str):
        self.txt_log.insert(tk.END, text, tag)
        if self.var_autoscroll.get():
            self.txt_log.see(tk.END)
        # 限制行数
        lines = int(self.txt_log.index("end-1c").split(".")[0])
        if lines > 1000:
            self.txt_log.delete("1.0", "200.0")

    @staticmethod
    def _ctrl_name(ctrl: int) -> str:
        names = {
            0x01: "读数据", 0x02: "读后续", 0x03: "读块", 0x04: "写数据",
            0x05: "写块", 0x0C: "广播校时", 0x10: "复位",
            0x11: "读数据(扩展)", 0x13: "读通信地址",
            0x81: "读数据应答", 0x82: "读后续应答", 0x83: "读块应答",
            0x84: "写数据应答", 0x85: "写块应答", 0x8C: "校时应答",
            0x90: "复位应答", 0x91: "读数据应答", 0x93: "读地址应答",
        }
        return f"0x{ctrl:02X}({names.get(ctrl, '?')})"


# ============================================================================
# 8. 入口
# ============================================================================

def main():
    root = tk.Tk()
    try:
        root.iconbitmap(default="")
    except Exception:
        pass
    app = SlaveApp(root)

    def on_closing():
        if app.listener and app.listener.is_alive():
            app.listener.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
