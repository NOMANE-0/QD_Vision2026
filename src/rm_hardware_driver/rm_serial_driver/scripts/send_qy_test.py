#!/usr/bin/env python3
"""发送 34 字节 QY 测试帧；仅依赖 Linux Python 标准库，不依赖 ROS。"""

import argparse
import binascii
import fcntl
import math
import os
import select
import struct
import termios
import time
from contextlib import contextmanager


def crc16(data, algorithm):
    """计算前 32 字节的 CRC；返回尚未编码为小端字节的整数。"""
    if algorithm == "ccitt-false":
        return binascii.crc_hqx(data, 0xFFFF)
    # rm 对应 RoboMaster 常见的 MCRF4XX：反射多项式 0x8408、初值 0xFFFF。
    polynomial = 0xA001 if algorithm == "modbus" else 0x8408
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (polynomial if crc & 1 else 0)
    return crc ^ (0xFFFF if algorithm == "x25" else 0)


def make_frame(args):
    """按截图偏移组包；浮点数采用小端 IEEE754，底盘指令先限幅再取反。"""
    chassis = [-max(-1.0, min(1.0, value)) for value in
               (args.linear_x, args.linear_y, args.angular_z)]
    # 将负零统一为正零，便于串口助手逐字节对照。
    chassis = [value if value else 0.0 for value in chassis]
    payload = struct.pack(
        "<2sB6fBf", b"QY", args.mode,
        max(-math.pi, min(math.pi, args.yaw)),
        max(-math.pi, min(math.pi, args.pitch)),
        *chassis, args.distance, args.target_id, args.target_v_yaw,
    )
    return payload + struct.pack("<H", crc16(payload, args.crc))


def byte_value(value):
    """接受十进制或带 0x/0b 前缀的单字节整数。"""
    try:
        result = int(value, 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError("应为整数，例如 17、0x11、0b1100") from error
    if not 0 <= result <= 255:
        raise argparse.ArgumentTypeError("必须在 0～255 范围内")
    return result


def finite_float(value):
    """拒绝 NaN、无穷大及不能用 float32 表示的数值。"""
    try:
        result = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("应为有限浮点数") from error
    if not math.isfinite(result) or abs(result) > 3.4028234663852886e38:
        raise argparse.ArgumentTypeError("应为 float32 范围内的有限浮点数")
    return result


@contextmanager
def open_port(port, baud):
    """配置 Linux 串口为 8N1、无流控；退出时恢复串口参数并关闭设备。"""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    exclusive = False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        original = termios.tcgetattr(fd)
        settings = termios.tcgetattr(fd)
        settings[0] = 4
        settings[1] = 2
        settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        settings[3] = 7
        settings[4] = settings[5] = getattr(termios, f"B{baud}")
        settings[6][termios.VMIN] = 8
        settings[6][termios.VTIME] = 6
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        yield fd
    finally:
        try:
            if original is not None:
                termios.tcsetattr(fd, termios.TCSANOW, original)
        finally:
            try:
                if exclusive:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
            finally:
                os.close(fd)


def write_frame(fd, frame):
    """处理短写；一帧超过一秒未写完时停止并报错。"""
    remaining = memoryview(frame)
    deadline = time.monotonic() + 1.0
    while remaining:
        timeout = deadline - time.monotonic()
        if timeout <= 0 or not select.select([], [fd], [], timeout)[1]:
            raise TimeoutError("串口写入超时")
        try:
            written = os.write(fd, remaining)
        except BlockingIOError:
            continue
        if written == 0:
            raise OSError("串口写入返回 0 字节")
        remaining = remaining[written:]


def main():
    """解析测试参数，预览或按指定频率重复发送；Ctrl+C 停止。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="串口路径")
    parser.add_argument("--baud", type=int, default=921600, help="波特率，默认 921600")
    parser.add_argument("--hz", type=finite_float, default=10.0, help="发送频率，默认 10 Hz")
    parser.add_argument("--count", type=int, default=0, help="发送帧数；默认 0 表示持续发送")
    parser.add_argument("--dry-run", action="store_true", help="只显示数据，不打开串口")
    parser.add_argument("--mode", type=int, choices=(0, 1, 2), default=0,
                        help="mode：0 禁止云台控制/开火，1 允许云台控制/禁止开火，2 允许控制/开火")
    parser.add_argument("--yaw", type=finite_float, default=0.1234, help="yaw，rad，默认测试值 0.1234，限幅到 ±π")
    parser.add_argument("--pitch", type=finite_float, default=-0.2345,
                        help="pitch，rad，默认测试值 -0.2345，限幅到 ±π")
    for field in ("linear-x", "linear-y", "angular-z"):
        parser.add_argument(f"--{field}", type=finite_float, default=0.0,
                            help="底盘指令，默认 0，限幅到 [-1, 1] 后取反")
    parser.add_argument("--distance", type=finite_float, default=1.2345,
                        help="距离，m；默认使用非零测试值 1.2345")
    parser.add_argument("--target-id", type=byte_value, default=1,
                        help="目标编号，默认测试值 1")
    parser.add_argument("--target-v-yaw", type=finite_float, default=0.3456,
                        help="目标自转角速度，rad/s，默认测试值 0.3456")
    parser.add_argument("--crc", choices=("rm", "modbus", "ccitt-false", "x25"), default="rm",
                        help="CRC16 算法，默认按截图的 RM CRC（初值 0xFFFF、反射多项式 0x8408、无最终异或）")
    args = parser.parse_args()
    if args.baud <= 0 or not hasattr(termios, f"B{args.baud}"):
        parser.error("当前系统不支持该波特率")
    if args.hz <= 0 or args.hz * 34 * 10 > args.baud:
        parser.error("频率必须大于 0，且 34 字节/帧的 8N1 数据量不能超过波特率")
    if args.count < 0:
        parser.error("count 必须大于等于 0")
    frame = make_frame(args)
    print(f"串口 {args.port} | {args.baud} 8N1 | {args.hz:g} Hz | CRC16={args.crc}", flush=True)
    print(f"TX {len(frame)} 字节: {frame.hex(' ').upper()}", flush=True)
    print("CRC 覆盖前 32 字节，按截图所示小端存储。", flush=True)
    if args.dry_run:
        return 0
    sent = 0
    try:
        with open_port(args.port, args.baud) as fd:
            print("开始发送，Ctrl+C 停止。", flush=True)
            interval = 1.0 / args.hz
            next_send = time.monotonic()
            next_report = next_send + 1.0
            while args.count == 0 or sent < args.count:
                write_frame(fd, frame)
                sent += 1
                now = time.monotonic()
                if now >= next_report:
                    print(f"已写入串口 {sent} 帧", flush=True)
                    next_report = now + 1.0
                # 落后时不突发补发；有限帧发送也留出最后一帧的传输时间。
                next_send = max(next_send + interval, now)
                time.sleep(max(0.0, next_send - time.monotonic()))
    except KeyboardInterrupt:
        print(f"\n已停止，共写入 {sent} 帧。", flush=True)
    except (OSError, termios.error) as error:
        print(f"串口错误：{error}；已写入 {sent} 帧。", flush=True)
        return 1
    else:
        print(f"完成，共写入 {sent} 帧。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
