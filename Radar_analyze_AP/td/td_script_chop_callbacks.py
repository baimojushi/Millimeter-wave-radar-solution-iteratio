"""
TouchDesigner Script CHOP callbacks
=====================================
新建 Script CHOP，将本内容粘贴到其 callbacks DAT。
确保已新建一个 Text DAT 命名为 radar_module，
并将 radar_td.py 的内容粘贴进去。

Script CHOP 会以 TD 项目帧率持续 cook，
输出4个通道：phase / breath / human / conn
"""

def setupParameters(scriptOp, page):
    # 可在此添加 Custom Parameters，本示例不需要
    return


def cook(scriptOp):
    m = mod('radar_module')

    scriptOp.clear()

    # 当前滤波相位（连续浮点，可接波形显示）
    scriptOp.appendChan('phase')[0]  = m.get_phase()

    # 呼吸速率 BPM（每8秒更新，0=无效/计算中）
    scriptOp.appendChan('breath')[0] = m.get_breath()

    # 有无人（0 或 1）
    scriptOp.appendChan('human')[0]  = float(m.get_human())

    # TCP 连接状态（0 或 1）
    scriptOp.appendChan('conn')[0]   = float(m.is_connected())
