# -*- coding: utf-8 -*-
""" Modbus RTU 指令生成器 v4.0 """

def modbus_crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc

SEP = "─" * 52

CONFIG_REGS = [
    (0x2710, "设备地址",       None,         "1~247"),
    (0x2714, "电压上限",       None,         "0.1V"),
    (0x2715, "电压下限",       None,         "0.1V"),
    (0x2716, "电流上限",       None,         "mA"),
    (0x271C, "关窗极限角度",   None,         "0.1°"),
    (0x271D, "开窗极限角度",   None,         "0.1°"),
    (0x271E, "电流检测时间",   None,         "ms"),
    (0x3710, "霍尔方向",       ["正常", "反转"], ""),
    (0x3711, "电机方向",       ["正常", "反转"], ""),
    (0x3712, "减速比",         None,         "默认1183"),
    (0x3713, "电机极对数",     None,         "默认3"),
]

REALTIME_REGS = [
    (0x2730, "实时转速",   "r/min"),
    (0x2731, "实时角度",   "0.1°"),
    (0x2732, "实时电压",   "0.1V"),
    (0x2733, "实时电流",   "mA"),
    (0x2737, "实时方向",   ""),
    (0x3714, "霍尔脉冲累计", ""),
]

FAULT_BITS = [
    (0x01, "过压"), (0x02, "过流"), (0x40, "欠压"),
]

def ask_node():
    s = input("设备地址 [1]: ").strip()
    return int(s) if s else 1

def ask_value():
    s = input("值: ").strip()
    if s.startswith("0x") or s.startswith("0X"): return int(s, 16)
    try:
        if all(c in '0123456789abcdefABCDEF' for c in s): return int(s, 16)
        return int(s)
    except: return None

def print_cmd(req_data, node, note=""):
    """打印 发送/回令 区域"""
    print(f"\n  {SEP}")
    crc = modbus_crc16(req_data)
    req = ' '.join(f'{b:02X}' for b in req_data)
    print(f"  发送: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
    # 回令
    func = req_data[1]
    if func == 0x03:  # 读
        dlen = req_data[5] * 2  # 寄存器数×2字节
        xx = ' '.join(['XX'] * dlen)
        print(f"  回令: {node:02X} 03 {dlen:02X} {xx} crc_h crc_l")
    else:  # 写
        print(f"  回令: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}  (echo)")
    if note: print(f"  {note}")

# ===== 1. 读实时数据 =====
def menu_read_realtime():
    print("\n====== 读实时数据 =====")
    for i, (addr, name, unit) in enumerate(REALTIME_REGS):
        u = f" [单位：{unit}]" if unit else ""
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(REALTIME_REGS): raise
        addr, name, unit = REALTIME_REGS[idx]
    except: print("无效"); return

    node = ask_node()
    u = f" ({unit})" if unit else ""
    nreg = 2 if addr == 0x3714 else 1
    req_data = [node, 0x03, (addr>>8)&0xFF, addr&0xFF, 0x00, nreg]

    # ── 说明/举例（上部） ──
    print(f"\n  ▎{name}{u}")
    if addr == 0x2737:
        print(f"  ▎值: 0=停止  1=开窗(逆时针)  2=关窗(顺时针)")
        print(f"  举例 (回令含CRC):")
        for lb, v in [("停止", 0), ("开窗(逆时针)", 1), ("关窗(顺时针)", 2)]:
            rd = [node, 0x03, 0x02, (v>>8)&0xFF, v&0xFF]
            rcr = modbus_crc16(rd)
            rq = ' '.join(f'{b:02X}' for b in rd)
            print(f"    {lb}: {rq} {rcr&0xFF:02X} {rcr>>8&0xFF:02X}")

    if addr == 0x3714:
        print(f"  ▎开窗(逆时针)→加  关窗(顺时针)→减  上电归零")
        print(f"  ▎公式: 角度(°) = 脉冲数 / 12 / 减速比 × 360°")
        print(f"  ▎例如: 3550脉冲 / 12 / 1183 × 360° ≈ 90°")
        print(f"  ▎重置: 01 06 37 14 00 00 87 BB")
        print(f"  举例 (回令含CRC):")
        for lb, v in [("复位后(0)", 0), ("关窗90°(~3550)", 3550)]:
            lo, hi = v & 0xFFFF, (v>>16) & 0xFFFF
            rd = [node, 0x03, 0x04, (lo>>8)&0xFF, lo&0xFF, (hi>>8)&0xFF, hi&0xFF]
            rcr = modbus_crc16(rd)
            rq = ' '.join(f'{b:02X}' for b in rd)
            print(f"    {lb}: {rq} {rcr&0xFF:02X} {rcr>>8&0xFF:02X}")

    # ── 发送/回令（下部） ──
    print_cmd(req_data, node)

# ===== 2. 控制 =====
def menu_control():
    print("\n====== 电机控制 =====")
    for s in ["1. 控制开启","2. 控制关闭（转动时不会停）","3. 急停",
              "4. 开窗（逆时针）","5. 关窗（顺时针）","6. 重启复位","0. 返回"]:
        print(f"  {s}")
    c = input("选择: ").strip()
    if c == '0' or c == '': return
    cm = {'1':0x0001,'2':0x0002,'3':0x0004,'4':0x0011,'5':0x0021,'6':0x0008}
    val = cm.get(c)
    if val is None: print("无效"); return

    if val & 0x08: print("\n  ⚠ 设备将复位!")
    node = ask_node()
    req_data = [node, 0x06, 0x27, 0x20, (val>>8)&0xFF, val&0xFF]
    print_cmd(req_data, node)

# ===== 3. 读配置寄存器 =====
def menu_read_config():
    print("\n====== 读配置寄存器 =====")
    for i, (addr, name, opts, unit) in enumerate(CONFIG_REGS):
        u = f" [单位：{unit}]" if unit and unit != "默认1183" and unit != "默认3" else ""
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(CONFIG_REGS): raise
        addr, name, opts, unit = CONFIG_REGS[idx]
    except: print("无效"); return

    node = ask_node()
    req_data = [node, 0x03, (addr>>8)&0xFF, addr&0xFF, 0x00, 0x01]

    # ── 说明/举例（上部） ──
    print(f"\n  ▎{name}（0x{addr:04X}）")
    if opts is not None:
        print(f"  举例 (回令含CRC):")
        for vi, opt_name in enumerate(opts):
            rd = [node, 0x03, 0x02, (vi>>8)&0xFF, vi&0xFF]
            rcr = modbus_crc16(rd)
            rq = ' '.join(f'{b:02X}' for b in rd)
            print(f"    {opt_name}: {rq} {rcr&0xFF:02X} {rcr>>8&0xFF:02X}")

    # ── 发送/回令（下部） ──
    print_cmd(req_data, node)

# ===== 4. 写配置寄存器 =====
def menu_write_config():
    print("\n====== 写配置寄存器 =====")
    for i, (addr, name, opts, unit) in enumerate(CONFIG_REGS):
        u = f" [单位：{unit}]" if unit and unit != "默认1183" and unit != "默认3" else ""
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(CONFIG_REGS): raise
        addr, name, opts, unit = CONFIG_REGS[idx]
    except: print("无效"); return

    print(f"\n  ▎{name}（0x{addr:04X}）")

    val = None
    if opts is not None:
        for j, opt_name in enumerate(opts):
            print(f"  {j+1}. {opt_name}")
        print("  0. 返回")
        c2 = input("选择: ").strip()
        if c2 == '0': return
        try:
            vi = int(c2)-1
            if vi < 0 or vi >= len(opts): raise
            val = vi
        except: print("无效"); return
    else:
        us = f" ({unit})" if unit else ""
        print(f"  范围:{us}")
        val = ask_value()
        if val is None: print("无效"); return

    node = ask_node()
    req_data = [node, 0x06, (addr>>8)&0xFF, addr&0xFF, (val>>8)&0xFF, val&0xFF]
    print_cmd(req_data, node)

# ===== 5. 查看故障 =====
def menu_read_fault():
    node = ask_node()
    req_data = [node, 0x03, 0x27, 0x40, 0x00, 0x01]

    print(f"\n  ▎故障状态（0x2740）")
    print(f"  ▎bit0=过压  bit1=过流  bit6=欠压")
    print(f"  举例 (回令含CRC):")
    for lb, v in [("无故障", 0), ("过压", 0x0001), ("过流", 0x0002), ("欠压", 0x0040)]:
        rd = [node, 0x03, 0x02, (v>>8)&0xFF, v&0xFF]
        rcr = modbus_crc16(rd)
        rq = ' '.join(f'{b:02X}' for b in rd)
        print(f"    {lb}: {rq} {rcr&0xFF:02X} {rcr>>8&0xFF:02X}")

    print_cmd(req_data, node)

# ===== 6. 清除故障 =====
def menu_clear_fault():
    print("\n====== 清除故障（0x2740）======")
    for i, (bit, name) in enumerate(FAULT_BITS):
        print(f"  {i+1}. {name}")
    print("  4. 清除全部故障")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        ci = int(c)
        val = 0x0000 if ci == 4 else FAULT_BITS[ci-1][0] if 1 <= ci <= len(FAULT_BITS) else None
        if val is None: print("无效"); return
    except: print("无效"); return

    node = ask_node()
    req_data = [node, 0x06, 0x27, 0x40, (val>>8)&0xFF, val&0xFF]
    print_cmd(req_data, node)

# ===== 主菜单 =====
MENU = [
    ("读实时数据",   menu_read_realtime),
    ("控制",         menu_control),
    ("读配置寄存器", menu_read_config),
    ("写配置寄存器", menu_write_config),
    ("查看故障",     menu_read_fault),
    ("清除故障",     menu_clear_fault),
]

def main():
    while True:
        print("\n" + "=" * 42)
        print("  Modbus RTU 指令生成器 v4.0")
        print("=" * 42)
        for i, (name, _) in enumerate(MENU):
            print(f"  {i+1}. {name}")
        print("  7. 退出")
        print("=" * 42)
        c = input("选择 [1-7]: ").strip()
        if c == '7' or c == '': print("退出"); break
        try:
            idx = int(c)-1
            if 0 <= idx < len(MENU): MENU[idx][1]()
            else: print("无效")
        except: print("无效")
        input("\n按 Enter 返回...")

if __name__ == '__main__':
    main()
