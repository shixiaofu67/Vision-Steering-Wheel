import threading
import time
import ctypes
import win32api
import win32con
from pynput.keyboard import Controller
import pydirectinput
KEY_L,KEY_R="a","d"#绑定转向
SWAP=False#判断角度反向
PERIOD=0.02#完整PWM周期
TICK=0.003#线程调度节拍，多久检查一次"该按还是该松"，可改变大小提升丝滑程度
FULL=0.85#摇杆值超过0.85长按
MIN_DUTY=0.22#最短按下比例
#PWM(脉宽调制):推一半就按一半时间，推3/4就按3/4时间，推到底就一直按


def _sleep_precise(seconds):
    """高精度睡眠:用sleep睡大头,忙等补尾,避开Windows 15ms时间片"""
    if seconds < 0:
        return
    sleep_t=seconds-0.001
    if sleep_t>0:
        time.sleep(sleep_t)
    end=time.perf_counter()+0.001
    while time.perf_counter()<end:
        pass


class PulseKey(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)#线程生命周期底线
        self.kb=Controller()
        self.v=0.0#当前摇杆值
        self.phase=0.0#PWM相位计数器,记录当前PWM周期走到哪了
        self.pressed=None#记录当前物理按住的键
        self.running=True#中断循环的条件
        # 把睡眠粒度从 15.6ms 提到 1ms，占空比才准
        try:
            ctypes.windll.winmm.timeBeginPeriod(1)
        except Exception:
            pass

    def set(self,v):
        self.v=float(v)
    def stop(self):
        self.running=False
        self.join(timeout=0.3)#阻塞主线程最多300ms
        self.release()
        # 把睡眠粒度从 15.6ms 提到 1ms，占空比才准
        try:
            ctypes.windll.winmm.timeEndPeriod(1)
        except Exception:
            pass
    def hold(self, key):
        if self.pressed != key:
            self.release()
            pydirectinput.keyDown(key)      # 走 DirectInput，游戏能看到
            self.pressed = key
    def release(self):
        if self.pressed:
            pydirectinput.keyUp(self.pressed)
            self.pressed = None

    def run(self):
        try:
            while self.running:
                v = self.v
                if abs(v) < 1e-6:
                    self.release()
                    time.sleep(0.003)  # 空转也要睡
                    continue

                key = KEY_R if v > 0 else KEY_L
                if SWAP:
                    key = KEY_L if v > 0 else KEY_R

                if abs(v) >= FULL:
                    self.hold(key)
                    _sleep_precise(0.01)#满值时每 10ms 确认一次，不直接死循环
                    continue

                duty = MIN_DUTY + (1.0 - MIN_DUTY) * (abs(v) / FULL)
                self.hold(key)
                _sleep_precise(PERIOD * duty)  # 导通
                self.release()
                _sleep_precise(PERIOD * (1.0 - duty))  # 关断
        finally:
            self.release()








