# -*- coding: utf-8 -*-
"""
Modbus RTU 指令生成器 v3.0
"""

def modbus_crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc

def make_frame(data_bytes):
    crc = modbus_crc16(data_bytes)
    frame = list(data_bytes) + [crc & 0xFF, (crc >> 8) & 0xFF]
    return ' '.join(f'{b:02X}' for b in frame)

# ===== 寄存器定义 (地址, 名称, 选项列表或None) =====
# 如果 options 是 list，则选项对应值 0,1,2...
# 如果 options 是 None，则自由输入值

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
]

REALTIME_REGS = [
    (0x2730, "实时转速",   "r/min"),
    (0x2731, "实时角度",   "0.1°"),
    (0x2732, "实时电压",   "0.1V"),
    (0x2733, "实时电流",   "mA"),
    (0x2737, "实时方向",   ""),
]

FAULT_BITS = [
    (0x01, "过压"),
    (0x02, "过流"),
    (0x40, "欠压"),
]

def ask_node():
    s = input("设备地址 [1]: ").strip()
    return int(s) if s else 1

def ask_value():
    s = input("值: ").strip()
    if s.startswith("0x") or s.startswith("0X"):
        return int(s, 16)
    try:
        if all(c in '0123456789abcdefABCDEF' for c in s):
            return int(s, 16)
        return int(s)
    except:
        return None

# ===== 1. 读实时数据 =====
def menu_read_realtime():
    print("\n====== 读实时数据 =====")
    for i, (addr, name, unit) in enumerate(REALTIME_REGS):
        unit_str = f" ({unit})" if unit else ""
        print(f"  {i+1}. {name}{unit_str}")
    print("  0. 返回")
    choice = input("选择: ").strip()
    if choice == '0': return
    try:
        idx = int(choice) - 1
        if idx < 0 or idx >= len(REALTIME_REGS): raise
        addr, name, unit = REALTIME_REGS[idx]
    except:
        print("无效"); return

    node = ask_node()
    unit_str = f" ({unit})" if unit else ""
    data = [node, 0x03, (addr>>8)&0xFF, addr&0xFF, 0x00, 0x01]
    crc = modbus_crc16(data)
    req = ' '.join(f'{b:02X}' for b in data)
    print(f"\n  {name}{unit_str}")
    print(f"  发送: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
    print(f"  回令: {node:02X} 03 02 XX XX crc_h crc_l")

    # 举例 (已知值域的寄存器)
    examples = {
        0x2737: [("停止", 0), ("开窗(逆时针)", 1), ("关窗(顺时针)", 2)],
    }
    if addr in examples:
        print(f"  举例:")
        for label, val in examples[addr]:
            resp_data = [node, 0x03, 0x02, (val>>8)&0xFF, val&0xFF]
            rcrc = modbus_crc16(resp_data)
            print(f"    {label}: {node:02X} 03 02 {val>>8&0xFF:02X} {val&0xFF:02X} {rcrc&0xFF:02X} {rcrc>>8&0xFF:02X}")

# ===== 2. 控制 =====
def menu_control():
    print("\n====== 电机控制 =====")
    print("  1. 控制开启")
    print("  2. 控制关闭（转动时不会停）")
    print("  3. 急停")
    print("  4. 开窗（逆时针）")
    print("  5. 关窗（顺时针）")
    print("  6. 重启复位")
    print("  0. 返回")
    choice = input("选择: ").strip()
    if choice == '0' or choice == '': return

    cmd_map = {'1':0x0001, '2':0x0002, '3':0x0004,
               '4':0x0011, '5':0x0021, '6':0x0008}
    val = cmd_map.get(choice)
    if val is None: print("无效"); return

    if val & 0x08: print("\n  ⚠ 设备将复位!")
    node = ask_node()
    data = [node, 0x06, 0x27, 0x20, (val>>8)&0xFF, val&0xFF]
    crc = modbus_crc16(data)
    req = ' '.join(f'{b:02X}' for b in data)
    print(f"\n  发送: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
    print(f"  回令: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}  (echo)")

# ===== 3. 读配置寄存器 =====
def menu_read_config():
    print("\n====== 读配置寄存器 =====")
    for i, (addr, name, opts, unit) in enumerate(CONFIG_REGS):
        print(f"  {i+1}. {name}")
    print("  0. 返回")
    choice = input("选择: ").strip()
    if choice == '0': return
    try:
        idx = int(choice) - 1
        if idx < 0 or idx >= len(CONFIG_REGS): raise
        addr, name, opts, unit = CONFIG_REGS[idx]
    except:
        print("无效"); return

    node = ask_node()
    data = [node, 0x03, (addr>>8)&0xFF, addr&0xFF, 0x00, 0x01]
    crc = modbus_crc16(data)
    req = ' '.join(f'{b:02X}' for b in data)
    print(f"\n  {name}")
    print(f"  发送: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
    print(f"  回令: {node:02X} 03 02 XX XX crc_h crc_l")

    # 举例 (有预定义选项的寄存器)
    if opts is not None:
        print(f"  举例:")
        for vi, opt_name in enumerate(opts):
            resp_data = [node, 0x03, 0x02, (vi>>8)&0xFF, vi&0xFF]
            rcrc = modbus_crc16(resp_data)
            print(f"    {opt_name}: {node:02X} 03 02 {vi>>8&0xFF:02X} {vi&0xFF:02X} {rcrc&0xFF:02X} {rcrc>>8&0xFF:02X}")

# ===== 4. 写配置寄存器 =====
def menu_write_config():
    print("\n====== 写配置寄存器 =====")
    for i, (addr, name, opts, unit) in enumerate(CONFIG_REGS):
        print(f"  {i+1}. {name}")
    print("  0. 返回")
    choice = input("选择: ").strip()
    if choice == '0': return
    try:
        idx = int(choice) - 1
        if idx < 0 or idx >= len(CONFIG_REGS): raise
        addr, name, opts, unit = CONFIG_REGS[idx]
    except:
        print("无效"); return

    print(f"\n  {name}")

    val = None
    if opts is not None:
        # 有预定义选项，列出让用户选
        for j, opt_name in enumerate(opts):
            print(f"  {j+1}. {opt_name}")
        print("  0. 返回")
        c2 = input("选择: ").strip()
        if c2 == '0': return
        try:
            vi = int(c2) - 1
            if vi < 0 or vi >= len(opts): raise
            val = vi
        except:
            print("无效"); return
    else:
        unit_str = f" ({unit})" if unit else ""
        print(f"  范围: {unit_str}")
        val = ask_value()
        if val is None: print("无效"); return

    node = ask_node()
    data = [node, 0x06, (addr>>8)&0xFF, addr&0xFF, (val>>8)&0xFF, val&0xFF]
    crc = modbus_crc16(data)
    req = ' '.join(f'{b:02X}' for b in data)
    print(f"\n  发送: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
    print(f"  回令: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}  (echo)")

# ===== 5. 查看故障 =====
def menu_read_fault():
    node = ask_node()
    data = [node, 0x03, 0x27, 0x40, 0x00, 0x01]
    crc = modbus_crc16(data)
    req = ' '.join(f'{b:02X}' for b in data)
    print(f"\n  故障状态")
    print(f"  发送: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
    print(f"  回令: {node:02X} 03 02 XX XX crc_h crc_l")
    print(f"  bit0=过压  bit1=过流  bit6=欠压")
    print(f"  举例:")
    fault_examples = [("无故障", 0x0000), ("过压", 0x0001), ("过流", 0x0002), ("欠压", 0x0040)]
    for label, val in fault_examples:
        resp_data = [node, 0x03, 0x02, (val>>8)&0xFF, val&0xFF]
        rcrc = modbus_crc16(resp_data)
        print(f"    {label}: {node:02X} 03 02 {val>>8&0xFF:02X} {val&0xFF:02X} {rcrc&0xFF:02X} {rcrc>>8&0xFF:02X}")

# ===== 6. 清除故障 =====
def menu_clear_fault():
    print("\n====== 清除故障 =====")
    for i, (bit, name) in enumerate(FAULT_BITS):
        print(f"  {i+1}. {name}")
    print("  4. 清除全部故障")
    print("  0. 返回")
    choice = input("选择: ").strip()
    if choice == '0': return
    try:
        c = int(choice)
        if c == 4:
            val = 0x0000
        elif 1 <= c <= len(FAULT_BITS):
            val = FAULT_BITS[c-1][0]
        else:
            print("无效"); return
    except:
        print("无效"); return

    node = ask_node()
    data = [node, 0x06, 0x27, 0x40, (val>>8)&0xFF, val&0xFF]
    crc = modbus_crc16(data)
    req = ' '.join(f'{b:02X}' for b in data)
    print(f"\n  发送: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
    print(f"  回令: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}  (echo)")

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
        print("  Modbus RTU 指令生成器 v3.0")
        print("=" * 42)
        for i, (name, _) in enumerate(MENU):
            print(f"  {i+1}. {name}")
        print("  7. 退出")
        print("=" * 42)

        choice = input("选择 [1-7]: ").strip()
        if choice == '7' or choice == '':
            print("退出")
            break
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(MENU):
                MENU[idx][1]()
            else:
                print("无效")
        except:
            print("无效")

        input("\n按 Enter 返回...")

if __name__ == '__main__':
    main()
