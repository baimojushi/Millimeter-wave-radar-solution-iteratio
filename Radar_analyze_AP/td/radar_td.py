"""
ESP32 雷达 TCP 数据接收模块 for TouchDesigner
=====================================================
放置方式：
  1. 新建一个 Text DAT，命名为 radar_module，粘贴本脚本
  2. 新建一个 Script CHOP，在 callbacks DAT 里调用本模块提供的数据
  3. 可选：新建一个 Table DAT，命名为 radar_table，用于存储最近数据快照

使用方法（在任意 TD Python 上下文中）：
  mod('radar_module').connect()           # 开始连接
  mod('radar_module').disconnect()        # 断开
  mod('radar_module').get_phase()         # 当前滤波相位（float）
  mod('radar_module').get_breath()        # 呼吸速率 BPM（float，0=无效）
  mod('radar_module').get_human()         # 有无人（int 0/1）
  mod('radar_module').is_connected()      # TCP 是否已连接（bool）
  mod('radar_module').get_history(n)      # 最近 n 条相位值列表

Script CHOP callbacks 示例（粘贴到 Script CHOP 的 callbacks DAT）：
  import radar_module as rm              # 若已 import，也可用 mod()
  def cook(scriptOp):
      m = mod('radar_module')
      scriptOp.clear()
      scriptOp.appendChan('phase')[0]  = m.get_phase()
      scriptOp.appendChan('breath')[0] = m.get_breath()
      scriptOp.appendChan('human')[0]  = float(m.get_human())
      scriptOp.appendChan('conn')[0]   = float(m.is_connected())
"""

import socket
import threading
import json
import time
import collections

# ==================== 配置 ====================
_ESP32_IP   = '192.168.4.1'
_ESP32_PORT = 8080
_TIMEOUT    = 5.0           # 连接超时秒数
_HISTORY_LEN = 200          # 保存最近 N 条相位值（用于波形显示）
_RECONNECT_INTERVAL = 3.0   # 断线后重连等待秒数

# ==================== 内部状态（线程安全） ====================
_lock = threading.Lock()

_state = {
    'phase':     0.0,
    'breath':    0.0,
    'human':     0,
    'connected': False,
    'last_msg':  '',
}

_phase_history = collections.deque(maxlen=_HISTORY_LEN)

_thread = None
_stop_event = threading.Event()

# ==================== 内部：接收线程 ====================
def _recv_loop():
    while not _stop_event.is_set():
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(_TIMEOUT)
            sock.connect((_ESP32_IP, _ESP32_PORT))

            with _lock:
                _state['connected'] = True
            print(f'[Radar] Connected to {_ESP32_IP}:{_ESP32_PORT}')

            buf = ''
            sock.settimeout(2.0)

            while not _stop_event.is_set():
                try:
                    chunk = sock.recv(1024).decode('utf-8', errors='ignore')
                except socket.timeout:
                    continue

                if not chunk:
                    break

                buf += chunk
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    _parse_line(line)

        except Exception as e:
            print(f'[Radar] Connection error: {e}')
        finally:
            with _lock:
                _state['connected'] = False
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            if not _stop_event.is_set():
                time.sleep(_RECONNECT_INTERVAL)

    print('[Radar] Recv thread stopped.')

def _parse_line(line: str):
    try:
        obj = json.loads(line)
        msg_type = obj.get('t', '')
        with _lock:
            _state['human']    = int(obj.get('h', 0))
            _state['last_msg'] = line

            if msg_type == 'p':
                phase = float(obj.get('p', 0.0))
                _state['phase'] = phase
                _phase_history.append(phase)

            elif msg_type == 'r':
                _state['phase']  = float(obj.get('p', 0.0))
                _state['breath'] = float(obj.get('r', 0.0))
                _phase_history.append(_state['phase'])

    except (json.JSONDecodeError, ValueError, KeyError):
        pass

# ==================== 公开 API ====================
def connect(ip: str = _ESP32_IP, port: int = _ESP32_PORT):
    """启动后台接收线程，开始连接 ESP32。"""
    global _thread, _ESP32_IP, _ESP32_PORT
    _ESP32_IP   = ip
    _ESP32_PORT = port

    if _thread and _thread.is_alive():
        print('[Radar] Already running.')
        return

    _stop_event.clear()
    _thread = threading.Thread(target=_recv_loop, daemon=True, name='RadarRecv')
    _thread.start()
    print(f'[Radar] Connecting to {ip}:{port} ...')

def disconnect():
    """停止后台线程，断开连接。"""
    _stop_event.set()
    if _thread:
        _thread.join(timeout=3.0)
    with _lock:
        _state['connected'] = False
    print('[Radar] Disconnected.')

def get_phase() -> float:
    with _lock:
        return _state['phase']

def get_breath() -> float:
    with _lock:
        return _state['breath']

def get_human() -> int:
    with _lock:
        return _state['human']

def is_connected() -> bool:
    with _lock:
        return _state['connected']

def get_history(n: int = _HISTORY_LEN) -> list:
    """返回最近 n 条相位值列表，最新值在末尾。"""
    with _lock:
        hist = list(_phase_history)
    return hist[-n:] if n < len(hist) else hist

def get_snapshot() -> dict:
    """返回当前所有字段的快照字典（线程安全副本）。"""
    with _lock:
        return dict(_state)

# ==================== TD 生命周期钩子（可选） ====================
# TouchDesigner 重新加载 DAT 时自动调用
def onLoad():
    connect()

def onUnload():
    disconnect()
