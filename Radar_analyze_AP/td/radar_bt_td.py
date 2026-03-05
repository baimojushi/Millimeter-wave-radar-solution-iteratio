"""
ESP32 雷达 — 经典蓝牙 SPP 接入模块 for TouchDesigner
=========================================================
解决 TouchDesigner Serial DAT 无法读取 SPP COM 口的问题：
Serial DAT 打开端口后超时等待很短，SPP RFCOMM 协商（约20~30秒）
期间没有数据，Serial DAT 判定失败并关闭端口。

本脚本在后台线程中用 pyserial 打开 COM 口，
设置足够长的超时，等待 RFCOMM 握手完成后自动开始接收。

放置方式：
  1. 新建 Text DAT，命名为 radar_bt_module，粘贴本脚本
  2. 新建 Script CHOP，callbacks DAT 参考 td_script_chop_callbacks.py

使用方法：
  mod('radar_bt_module').connect('COM5')   # 替换为实际 COM 口号
  mod('radar_bt_module').disconnect()
  mod('radar_bt_module').get_phase()
  mod('radar_bt_module').get_breath()
  mod('radar_bt_module').get_human()
  mod('radar_bt_module').is_connected()

如何找到正确的 COM 口：
  设备管理器 → 端口(COM和LPT) → 找到
  "标准串行 over 蓝牙链接 (COMx)" — 取传出方向那个
"""

import threading
import json
import time
import collections

# pyserial 在 TD 内置 Python 中通常可用；若报 ImportError 需手动安装：
# 在 TD 的 Textport 执行: import subprocess; subprocess.run(['python','-m','pip','install','pyserial'])
try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False
    print('[RadarBT] WARNING: pyserial not found. Run: pip install pyserial')

# ==================== 配置 ====================
_DEFAULT_PORT    = 'COM5'       # 修改为实际 COM 口
_BAUD            = 115200       # SPP透传，波特率任意
_OPEN_TIMEOUT    = 60.0         # 等待 RFCOMM 握手的最长秒数
_READ_TIMEOUT    = 2.0
_HISTORY_LEN     = 200
_RECONNECT_WAIT  = 5.0

# ==================== 内部状态 ====================
_lock  = threading.Lock()
_state = {
    'phase':     0.0,
    'breath':    0.0,
    'human':     0,
    'connected': False,
}
_phase_history = collections.deque(maxlen=_HISTORY_LEN)
_thread        = None
_stop_event    = threading.Event()
_com_port      = _DEFAULT_PORT

# ==================== 接收线程 ====================
def _recv_loop():
    while not _stop_event.is_set():
        ser = None
        try:
            print(f'[RadarBT] Opening {_com_port} (waiting up to {_OPEN_TIMEOUT}s for RFCOMM)...')
            # timeout=None 表示阻塞读，open() 本身在 SPP 上可能就会阻塞到握手完成
            ser = serial.Serial()
            ser.port    = _com_port
            ser.baudrate = _BAUD
            ser.timeout  = _READ_TIMEOUT
            # 关闭 DSR/DTR 硬件流控，SPP 不需要
            ser.dsrdtr   = False
            ser.rtscts   = False
            ser.open()  # 这一步会阻塞到 RFCOMM 链路建立（约20~30秒）

            with _lock:
                _state['connected'] = True
            print(f'[RadarBT] Connected on {_com_port}')

            buf = ''
            while not _stop_event.is_set():
                raw = ser.readline()   # 等待换行，超时返回空bytes
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if line:
                    _parse_line(line)

        except serial.SerialException as e:
            print(f'[RadarBT] Serial error: {e}')
        except Exception as e:
            print(f'[RadarBT] Error: {e}')
        finally:
            with _lock:
                _state['connected'] = False
            if ser and ser.is_open:
                try:
                    ser.close()
                except Exception:
                    pass
            if not _stop_event.is_set():
                print(f'[RadarBT] Retrying in {_RECONNECT_WAIT}s...')
                time.sleep(_RECONNECT_WAIT)

    print('[RadarBT] Thread stopped.')

def _parse_line(line: str):
    try:
        obj = json.loads(line)
        t   = obj.get('t', '')
        with _lock:
            _state['human'] = int(obj.get('h', 0))
            if t == 'p':
                p = float(obj.get('p', 0.0))
                _state['phase'] = p
                _phase_history.append(p)
            elif t == 'r':
                p = float(obj.get('p', 0.0))
                _state['phase']  = p
                _state['breath'] = float(obj.get('r', 0.0))
                _phase_history.append(p)
    except Exception:
        pass

# ==================== 公开 API ====================
def connect(port: str = _DEFAULT_PORT):
    global _thread, _com_port
    if not _SERIAL_AVAILABLE:
        print('[RadarBT] pyserial not available, cannot connect.')
        return
    _com_port = port
    if _thread and _thread.is_alive():
        print('[RadarBT] Already running.')
        return
    _stop_event.clear()
    _thread = threading.Thread(target=_recv_loop, daemon=True, name='RadarBTRecv')
    _thread.start()
    print(f'[RadarBT] Starting on {port}')

def disconnect():
    _stop_event.set()
    if _thread:
        _thread.join(timeout=5.0)
    with _lock:
        _state['connected'] = False
    print('[RadarBT] Disconnected.')

def get_phase() -> float:
    with _lock: return _state['phase']

def get_breath() -> float:
    with _lock: return _state['breath']

def get_human() -> int:
    with _lock: return _state['human']

def is_connected() -> bool:
    with _lock: return _state['connected']

def get_history(n: int = _HISTORY_LEN) -> list:
    with _lock:
        hist = list(_phase_history)
    return hist[-n:] if n < len(hist) else hist

# ==================== TD 生命周期钩子 ====================
def onLoad():
    connect(_DEFAULT_PORT)

def onUnload():
    disconnect()
