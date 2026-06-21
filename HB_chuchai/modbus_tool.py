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

# 校验规则: regAddr -> (min, max, step, unit, unit_label)
VALIDATION_RULES = {
    0x2714: (250,  270,   2,  0.1, "V"),    # 过压阈值 25.0~27.0V, 步进0.2V
    0x2715: (210,  230,   2,  0.1, "V"),    # 欠压阈值 21.0~23.0V, 步进0.2V
    0x2716: (0,    2300, 50,  1,   "mA"),   # 过流阈值 0~2300mA, 步进50mA
    0x271E: (0,    2000, 20,  1,   "ms"),   # 判定时间 0~2000ms, 步进20ms
    0x3712: (10,  65535,  0,  0.1, ""),     # 减速比 1.0~6553.5, 单位0.1
    0x3713: (1,     100,  0,  1,   ""),     # 极对数 1~100, 不取整
}

def apply_validation(regAddr, raw_value):
    rule = VALIDATION_RULES.get(regAddr)
    if rule is None:
        return raw_value, False
    vmin, vmax, step, unit, label = rule
    result = raw_value
    modified = False
    if result < vmin:
        result = vmin; modified = True
    if result > vmax:
        result = vmax; modified = True
    if step > 0:
        lower = (result // step) * step
        upper = lower + step
        if (result - lower) >= (upper - result):
            rounded = upper
        else:
            rounded = lower
        if rounded != result:
            result = rounded; modified = True
    return result, modified

def fmt_constraint(regAddr):
    rule = VALIDATION_RULES.get(regAddr)
    if rule is None:
        return ""
    vmin, vmax, step, unit, label = rule
    parts = []
    if vmin > 0 or regAddr == 0x2716 or regAddr == 0x271E:
        parts.append(f"最小 {vmin*unit:.1f}{label}" if unit < 1 else f"最小 {vmin}{label}")
    parts.append(f"最大 {vmax*unit:.1f}{label}" if unit < 1 else f"最大 {vmax}{label}")
    if step > 0:
        s = step * unit
        parts.append(f"步进 {s:.1f}{label}" if unit < 1 else f"步进 {s}{label}")
    return "  [" + ", ".join(parts) + "]"

CONFIG_REGS = [
    (0x2710, "设备地址",       None,         "1~247"),
    (0x2714, "过压阈值",       None,         "0.1V"),
    (0x2715, "欠压阈值",       None,         "0.1V"),
    (0x2716, "过流阈值",       None,         "mA"),
    (0x271C, "关窗极限角度",   None,         "0.1°"),
    (0x271D, "开窗极限角度",   None,         "0.1°"),
    (0x271E, "过流判定时间",   None,         "ms"),
]

DEV_REGS = [
    (0x3710, "霍尔方向",       ["正常", "反转"], ""),
    (0x3711, "电机方向",       ["正常", "反转"], ""),
    (0x3712, "减速比",         None,         "0.1"),
    (0x3713, "电机极对数",     None,         ""),
]

REALTIME_REGS = [
    (0x2730, "实时转速",   "r/min"),
    (0x2731, "实时角度",   "0.1°"),
    (0x2732, "实时电压",   "0.1V"),
    (0x2733, "实时电流",   "mA"),
    (0x2737, "实时方向",   ""),
]

DEV_REGS = [
    (0x3714, "霍尔脉冲累计", None,         ""),
    (0x3710, "霍尔方向",       ["正常", "反转"], ""),
    (0x3711, "电机方向",       ["正常", "反转"], ""),
    (0x3712, "减速比",         None,         "0.1"),
    (0x3713, "电机极对数",     None,         ""),
]

FAULT_BITS = [
    (0x01, "过压"), (0x02, "过流"), (0x40, "欠压"),
]

DEV_PASSWORD = "5858"
_dev_authenticated = False

def check_dev_password():
    global _dev_authenticated
    if _dev_authenticated:
        return True
    pwd = input("  密码: ").strip()
    if pwd != DEV_PASSWORD:
        print("  密码错误!")
        return False
    _dev_authenticated = True
    return True

def ask_node():
    s = input("设备地址 [1]: ").strip()
    return int(s) if s else 1

def ask_value(prompt="值"):
    s = input(f"{prompt}: ").strip()
    if s.startswith("0x") or s.startswith("0X"):
        return int(s, 16)
    has_hex_letter = any(c in 'abcdefABCDEF' for c in s)
    if has_hex_letter:
        if all(c in '0123456789abcdefABCDEF' for c in s):
            return int(s, 16)
        return None
    try:
        return int(s)
    except:
        return None

def print_cmd(req_data, node, note="", skip_echo=False):
    print(f"\n  {SEP}")
    crc = modbus_crc16(req_data)
    req = ' '.join(f'{b:02X}' for b in req_data)
    print(f"  发送: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
    if not skip_echo:
        func = req_data[1]
        if func == 0x03:
            dlen = req_data[5] * 2
            xx = ' '.join(['XX'] * dlen)
            print(f"  回令: {node:02X} 03 {dlen:02X} {xx} crc_h crc_l")
        else:
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
    req_data = [node, 0x03, (addr>>8)&0xFF, addr&0xFF, 0x00, 0x01]

    print(f"\n  ▎{name}{u}")
    if addr == 0x2737:
        print(f"  ▎值: 0=停止  1=开窗(逆时针)  2=关窗(顺时针)")
        print(f"  举例 (回令含CRC):")
        for lb, v in [("停止", 0), ("开窗(逆时针)", 1), ("关窗(顺时针)", 2)]:
            rd = [node, 0x03, 0x02, (v>>8)&0xFF, v&0xFF]
            rcr = modbus_crc16(rd)
            rq = ' '.join(f'{b:02X}' for b in rd)
            print(f"    {lb}: {rq} {rcr&0xFF:02X} {rcr>>8&0xFF:02X}")

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
        c = fmt_constraint(addr)
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}{c}")
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

    print(f"\n  ▎{name}（0x{addr:04X}）")
    if opts is not None:
        print(f"  举例 (回令含CRC):")
        for vi, opt_name in enumerate(opts):
            rd = [node, 0x03, 0x02, (vi>>8)&0xFF, vi&0xFF]
            rcr = modbus_crc16(rd)
            rq = ' '.join(f'{b:02X}' for b in rd)
            print(f"    {opt_name}: {rq} {rcr&0xFF:02X} {rcr>>8&0xFF:02X}")

    print_cmd(req_data, node)

# ===== 4. 写配置寄存器 =====
def menu_write_config():
    print("\n====== 写配置寄存器 =====")
    for i, (addr, name, opts, unit) in enumerate(CONFIG_REGS):
        u = f" [单位：{unit}]" if unit and unit != "默认1183" and unit != "默认3" else ""
        c = fmt_constraint(addr)
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}{c}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(CONFIG_REGS): raise
        addr, name, opts, unit = CONFIG_REGS[idx]
    except: print("无效"); return

    print(f"\n  ▎{name}（0x{addr:04X}）")
    rule = VALIDATION_RULES.get(addr)

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
        if rule:
            vmin, vmax, step, r_unit, r_label = rule
            min_disp = f"{vmin * r_unit:.1f}" if r_unit < 1 else str(vmin)
            max_disp = f"{vmax * r_unit:.1f}" if r_unit < 1 else str(vmax)
            s = step * r_unit
            step_disp = f"{s:.1f}" if r_unit < 1 else str(s)
            print(f"  范围: {min_disp}~{max_disp}{r_label}, 步进{step_disp}{r_label}")
            prompt = f"值（单位：{unit}）" if unit and unit != "1~247" else "值"
        else:
            print(f"  范围:{us}")
            prompt = f"值（单位：{unit}）" if unit and unit != "1~247" else "值"
        val = ask_value(prompt)
        if val is None: print("无效"); return

    node = ask_node()
    req_data = [node, 0x06, (addr>>8)&0xFF, addr&0xFF, (val>>8)&0xFF, val&0xFF]

    if rule:
        result, modified = apply_validation(addr, val)
        if modified:
            vmin, vmax, step, r_unit, r_label = rule
            disp_raw = f"{val * r_unit:.1f}{r_label}" if r_unit < 1 else f"{val}{r_label}"
            disp_res = f"{result * r_unit:.1f}{r_label}" if r_unit < 1 else f"{result}{r_label}"
            note = f"MCU校验后: {disp_raw} → {disp_res}"
        else:
            note = ""
        print_cmd(req_data, node, note, skip_echo=True)
        if modified:
            echo_data = [node, 0x06, (addr>>8)&0xFF, addr&0xFF, (result>>8)&0xFF, result&0xFF]
            crc = modbus_crc16(echo_data)
            echo = ' '.join(f'{b:02X}' for b in echo_data)
            print(f"  预期回令: {echo} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
        else:
            crc = modbus_crc16(req_data)
            req = ' '.join(f'{b:02X}' for b in req_data)
            print(f"  回令: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}  (echo)")
    else:
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

# ===== 7. 开发者选项 =====
def menu_dev_options():
    if not check_dev_password(): return
    while True:
        print("\n====== 开发者选项 =====")
        print("  1. 读配置寄存器")
        print("  2. 写配置寄存器")
        print("  3. 计算霍尔脉冲")
        print("  4. 读霍尔脉冲")
        print("  5. 重置霍尔脉冲")
        print("  6. 计算实时角度")
        print("  0. 返回")
        c = input("选择: ").strip()
        if c == '0': return
        elif c == '1':
            menu_read_dev_regs()
        elif c == '2':
            menu_write_dev_regs()
        elif c == '3':
            menu_calc_hall_pulse()
        elif c == '4':
            menu_read_hall_pulse()
        elif c == '5':
            menu_reset_hall_pulse()
        elif c == '6':
            menu_calc_realtime_angle()
        else:
            print("无效")

def menu_read_dev_regs():
    print("\n====== 开发者选项 - 读配置 =====")
    for i, (addr, name, opts, unit) in enumerate(DEV_REGS):
        u = f" [单位：{unit}]" if unit and unit != "默认1183" and unit != "默认3" else ""
        c = fmt_constraint(addr)
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}{c}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(DEV_REGS): raise
        addr, name, opts, unit = DEV_REGS[idx]
    except: print("无效"); return

    node = ask_node()
    nreg = 2 if addr == 0x3714 else 1
    req_data = [node, 0x03, (addr>>8)&0xFF, addr&0xFF, 0x00, nreg]

    print(f"\n  ▎{name}（0x{addr:04X}）")
    if addr == 0x3714:
        print(f"  ▎开窗(逆时针)→加  关窗(顺时针)→减  上电归零")
        print(f"  ▎公式: 角度(°) = 脉冲数 / 12 / 减速比 × 360°")
        print(f"  ▎重置: 01 06 37 14 00 00 87 BB")
    if opts is not None:
        print(f"  举例 (回令含CRC):")
        for vi, opt_name in enumerate(opts):
            rd = [node, 0x03, 0x02, (vi>>8)&0xFF, vi&0xFF]
            rcr = modbus_crc16(rd)
            rq = ' '.join(f'{b:02X}' for b in rd)
            print(f"    {opt_name}: {rq} {rcr&0xFF:02X} {rcr>>8&0xFF:02X}")

    print_cmd(req_data, node)

def menu_write_dev_regs():
    print("\n====== 开发者选项 - 写配置 =====")
    for i, (addr, name, opts, unit) in enumerate(DEV_REGS):
        u = f" [单位：{unit}]" if unit and unit != "默认1183" and unit != "默认3" else ""
        c = fmt_constraint(addr)
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}{c}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(DEV_REGS): raise
        addr, name, opts, unit = DEV_REGS[idx]
    except: print("无效"); return

    print(f"\n  ▎{name}（0x{addr:04X}）")
    rule = VALIDATION_RULES.get(addr)

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
        if rule:
            vmin, vmax, step, r_unit, r_label = rule
            min_disp = f"{vmin * r_unit:.1f}" if r_unit < 1 else str(vmin)
            max_disp = f"{vmax * r_unit:.1f}" if r_unit < 1 else str(vmax)
            if step > 0:
                s = step * r_unit
                step_disp = f"{s:.1f}" if r_unit < 1 else str(s)
                print(f"  范围: {min_disp}~{max_disp}{r_label}, 步进{step_disp}{r_label}")
            else:
                print(f"  范围: {min_disp}~{max_disp}{r_label}")
        else:
            print(f"  范围:{us}")
        prompt = f"值（单位：{unit}）" if unit and unit != "1~247" else "值"
        val = ask_value(prompt)
        if val is None: print("无效"); return

    node = ask_node()
    req_data = [node, 0x06, (addr>>8)&0xFF, addr&0xFF, (val>>8)&0xFF, val&0xFF]

    if rule:
        result, modified = apply_validation(addr, val)
        if modified:
            vmin, vmax, step, r_unit, r_label = rule
            disp_raw = f"{val * r_unit:.1f}{r_label}" if r_unit < 1 else f"{val}{r_label}"
            disp_res = f"{result * r_unit:.1f}{r_label}" if r_unit < 1 else f"{result}{r_label}"
            note = f"MCU校验后: {disp_raw} → {disp_res}"
        else:
            note = ""
        print_cmd(req_data, node, note, skip_echo=True)
        if modified:
            echo_data = [node, 0x06, (addr>>8)&0xFF, addr&0xFF, (result>>8)&0xFF, result&0xFF]
            crc = modbus_crc16(echo_data)
            echo = ' '.join(f'{b:02X}' for b in echo_data)
            print(f"  预期回令: {echo} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
        else:
            crc = modbus_crc16(req_data)
            req = ' '.join(f'{b:02X}' for b in req_data)
            print(f"  回令: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}  (echo)")
    else:
        print_cmd(req_data, node)

def menu_read_hall_pulse():
    print("\n====== 读霍尔脉冲 =====")
    node = ask_node()
    req_data = [node, 0x03, 0x37, 0x14, 0x00, 0x02]
    print_cmd(req_data, node)

def menu_reset_hall_pulse():
    print("\n====== 重置霍尔脉冲 =====")
    print("  ⚠ 将清零霍尔脉冲累计值")
    node = ask_node()
    req_data = [node, 0x06, 0x37, 0x14, 0x00, 0x00]
    print_cmd(req_data, node)

def menu_calc_realtime_angle():
    print("\n====== 计算实时角度 =====")
    print("  粘贴 Modbus 回令帧 (如: 01 03 02 03 93 F8 D9)")
    s = input("  回令: ").strip()
    try:
        parts = s.split()
        if len(parts) < 4:
            print("  格式错误: 至少需要4字节")
            return
        raw = [int(x, 16) for x in parts]

        if raw[1] == 0x03:
            byte_count = raw[2]
            if len(raw) < 3 + byte_count:
                print(f"  数据不完整, 需要 {3+byte_count} 字节")
                return
            data_bytes = raw[3:3+byte_count]
            # Modbus大端序: int16 = (H<<8)|L
            val = (data_bytes[0] << 8) | data_bytes[1]
            if val > 32767:
                val = val - 65536  # 有符号 int16
            angle = val * 0.1
            print(f"  实时角度: {angle:.1f}°")
        else:
            print("  不是 0x03 回令帧")
    except Exception as e:
        print(f"  解析失败: {e}")

def menu_calc_hall_pulse():
    print("\n====== 计算霍尔脉冲 → 角度 =====")
    print("  粘贴 Modbus 回令帧 (如: 01 03 04 0D C9 00 00 28 A1)")
    s = input("  回令: ").strip()
    try:
        parts = s.split()
        if len(parts) < 4:
            print("  格式错误: 至少需要4字节")
            return
        raw = [int(x, 16) for x in parts]

        # 解析 0x03 回令: addr 03 byteCount data[byteCount] crc_l crc_h
        if raw[1] == 0x03:
            byte_count = raw[2]
            if len(raw) < 3 + byte_count:
                print(f"  数据不完整, 需要 {3+byte_count} 字节")
                return
            data_bytes = raw[3:3+byte_count]
        else:
            # 尝试直接解析为 hex 数据
            data_bytes = raw

        # Modbus 是大端序: 每2字节一个寄存器, 低地址寄存器在前
        # data_bytes = [regL_H, regL_L, regH_H, regH_L]
        pulse_count = 0
        for i, b in enumerate(data_bytes):
            # 每16位寄存器内大端序, 寄存器间低16位在前
            reg_idx = i // 2         # 0=低16位寄存器, 1=高16位寄存器
            byte_in_reg = 1 - (i % 2)  # 寄存器内: 高位字节在前
            shift = reg_idx * 16 + byte_in_reg * 8
            pulse_count |= (b << shift)

        if pulse_count > 0x7FFFFFFF:
            pulse_count = pulse_count - 0x100000000  # 转为有符号 int32

        print(f"  脉冲数: {pulse_count}")
    except Exception as e:
        print(f"  解析失败: {e}")
        return

    # 减速比
    rs = input("  减速比 (单位0.1, 默认11830): ").strip()
    try:
        ratio_raw = int(rs) if rs else 11830
    except:
        print("  无效"); return
    ratio = ratio_raw / 10.0

    # 一圈几脉冲
    ps = input("  一圈几脉冲 [12]: ").strip()
    try:
        pulses_per_rev = int(ps) if ps else 12
    except:
        print("  无效"); return
    if pulses_per_rev <= 0:
        print("  脉冲数必须 > 0"); return

    angle = pulse_count / pulses_per_rev / ratio * 360.0
    print(f"\n  {'='*42}")
    print(f"  脉冲数: {pulse_count}")
    print(f"  减速比: {ratio:.1f}:1")
    print(f"  一圈脉冲: {pulses_per_rev}")
    print(f"  转动角度: {angle:.2f}°")
    print(f"  {'='*42}")

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
        print("  7. 开发者选项")
        print("  8. 退出")
        print("=" * 42)
        c = input("选择 [1-8]: ").strip()
        if c == '8' or c == '': print("退出"); break
        try:
            idx = int(c)-1
            if idx == 6:   # 7. 开发者选项
                menu_dev_options()
            elif 0 <= idx < len(MENU):
                MENU[idx][1]()
            else:
                print("无效")
        except: print("无效")
        input("\n按 Enter 返回...")

if __name__ == '__main__':
    main()
