#!/usr/bin/env python3
"""
PID 自动调优工具 (全参数)
调优范围: Kp, Ki, Kd, max_output, min_output, max_integral, min_integral
"""

import serial, time, csv, sys, threading, signal, os
from datetime import datetime
from collections import OrderedDict

# 固件原始PID参数（完整7个）
ORIG_CF = OrderedDict([
    ('kp', 2.0), ('ki', 0.85), ('kd', 40.0),
    ('max_output', 100.0), ('min_output', 82.0),
    ('max_integral', 95.0), ('min_integral', 0.0),
])

ORIG_RESIN = OrderedDict([
    ('kp', 2.2), ('ki', 0.6), ('kd', 150.0),
    ('max_output', 100.0), ('min_output', 78.0),
    ('max_integral', 130.0), ('min_integral', 0.0),
])

CH_NAMES = {1: '纤维(CF)', 2: '树脂(Resin)'}
PARAM_LABELS = {
    'kp': 'Kp', 'ki': 'Ki', 'kd': 'Kd',
    'max_output': 'max_output', 'min_output': 'min_output',
    'max_integral': 'max_integral', 'min_integral': 'min_integral',
}


class AutoTuner:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=2)
        self.running = False
        self.data = []
        self.lock = threading.Lock()
        self.temp_key = ''
        self.target_key = ''
        time.sleep(2)

    def start(self):
        self.running = True
        threading.Thread(target=self._monitor, daemon=True).start()

    def _monitor(self):
        buf = ''
        while self.running:
            try:
                d = self.ser.read(256)
                if d:
                    buf += d.decode('utf-8', errors='ignore')
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        if line.strip().startswith('STAT'):
                            self._parse(line.strip())
            except:
                break

    def _parse(self, line):
        rec = {'t': time.time(), 'elapsed': 0.0,
               'temp_cf': 0.0, 'temp_resin': 0.0,
               'target_cf': 0.0, 'target_resin': 0.0}
        for p in line.split()[1:]:
            if '=' in p:
                k, v = p.split('=', 1)
                try: rec[k] = float(v)
                except: pass
        with self.lock:
            if self.data:
                rec['elapsed'] = rec['t'] - self.data[0]['t']
            self.data.append(rec)

    def cmd(self, s):
        self.ser.write(s.encode())

    def fan(self, channel, on):
        tag = 'cf' if channel == 1 else 'resin'
        self.cmd(f"EV fan_{tag} {1 if on else 0}\n")
        time.sleep(0.1)

    def set_pid(self, channel, params):
        tag = 'cf' if channel == 1 else 'resin'
        self.cmd(f"EV pid_set_{tag} {params['kp']} {params['ki']} {params['kd']} "
                 f"{params['max_output']} {params['min_output']} "
                 f"{params['max_integral']} {params['min_integral']}\n")
        time.sleep(0.05)
        print(f"  → {dict((k, f'{v:.2f}') for k, v in params.items())}")

    def heat(self, channel, temp):
        self.cmd(f"EV heat_{'cf' if channel == 1 else 'resin'} {temp}\n")

    def stop_heat(self, channel):
        self.cmd(f"EV heat_{'cf' if channel == 1 else 'resin'} 0\n")

    def clear(self):
        with self.lock:
            self.data.clear()

    def get_temp(self):
        with self.lock:
            if not self.data: return 25.0
            return self.data[-1].get(self.temp_key, 25.0)

    def wait_stable(self, target, tol=2.0, min_secs=20):
        enter_time = None
        while self.running:
            time.sleep(1)
            cur = self.get_temp()
            if abs(cur - target) <= tol:
                if enter_time is None:
                    enter_time = time.time()
                elif time.time() - enter_time >= min_secs:
                    print(f"\n  ✓ 稳定: {cur:.1f}°C")
                    return True
            else:
                enter_time = None
            sys.stdout.write(f"\r  {cur:.1f}°C / {target:.0f}°C")
            sys.stdout.flush()
        return False

    def wait_cool(self, below=45):
        while self.running:
            time.sleep(1)
            cur = self.get_temp()
            if cur <= below:
                print(f"\n  ✓ 冷却: {cur:.1f}°C")
                return True
            sys.stdout.write(f"\r  冷却 {cur:.1f}°C")
            sys.stdout.flush()
        return False

    def get_data(self):
        with self.lock:
            return list(self.data)

    def analyze(self, target, log_file, params):
        data = self.get_data()
        if len(data) < 10:
            return None

        temps = [d[self.temp_key] for d in data]
        times = [d['elapsed'] for d in data]
        n = len(temps)
        t0 = temps[0]
        tmax = max(temps)
        idx_max = temps.index(tmax)
        tmax_t = times[idx_max]

        overshoot = max(0, tmax - target)
        overshoot_pct = overshoot / target * 100 if target > 0 else 0

        rise_time = None
        if target - t0 > 0:
            t10 = t0 + 0.1 * (target - t0)
            t90 = t0 + 0.9 * (target - t0)
            i10 = next((i for i, v in enumerate(temps) if v >= t10), None)
            i90 = next((i for i, v in enumerate(temps) if v >= t90), None)
            if i10 is not None and i90 is not None:
                rise_time = times[i90] - times[i10]

        steady = sum(temps[-10:]) / 10
        steady_err = steady - target
        steady_ripple = max(temps[-10:]) - min(temps[-10:])

        # 打印参数
        param_str = '  '.join(f"{k}={v}" for k, v in params.items())
        print(f"\n  参数: {param_str}")
        print(f"  {'─'*52}")
        print(f"  初始: {t0:.1f}°C  最高: {tmax:.1f}°C  "
              f"超调: {overshoot:.1f}°C ({overshoot_pct:.1f}%)")
        if rise_time:
            print(f"  上升: {rise_time:.1f}s  峰值: {tmax_t:.1f}s")
        print(f"  稳态: {steady:.1f}°C  误差: {steady_err:+.1f}°C  "
              f"波动: {steady_ripple:.1f}°C")
        print(f"  {'─'*52}")

        # 保存CSV
        os.makedirs('pid_logs', exist_ok=True)
        with open(log_file, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'temperature', 'target'])
            for d in data:
                w.writerow([f"{d['elapsed']:.1f}",
                           f"{d[self.temp_key]:.1f}",
                           f"{d[self.target_key]:.1f}"])
        print(f"  → 日志: {log_file}")

        return {
            'overshoot': overshoot, 'overshoot_pct': overshoot_pct,
            'rise_time': rise_time, 'steady_err': steady_err,
            'steady_ripple': steady_ripple, 'points': n,
        }

    def suggest(self, params, result, target):
        """自动调优全部7个参数"""
        p = dict(params)
        changes = []

        # ---- min_output: 波动大就逐步降低，没有下限 ----
        if result['steady_ripple'] > 3:
            p['min_output'] = p['min_output'] * 0.85
            if 'min_output' not in str(changes):
                changes.append(f"波动{result['steady_ripple']:.1f}°C → min_output×0.85")
        elif result['steady_ripple'] > 2:
            p['min_output'] = p['min_output'] * 0.93
            if 'min_output' not in str(changes):
                changes.append(f"波动{result['steady_ripple']:.1f}°C → min_output×0.93")

        # ---- Kp, Kd, max_integral: 超调控制 ----
        # min_output固定偏高时，积分饱和是超调主因，降max_integral最有效
        if result['overshoot_pct'] > 10:
            p['kp'] *= 0.85
            p['kd'] *= 1.2
            p['max_integral'] *= 0.80
            changes.append(f"超调{result['overshoot_pct']:.0f}% → Kp×0.85 Kd×1.2 max_integral×0.8")
        elif result['overshoot_pct'] > 5:
            p['kp'] *= 0.95
            p['kd'] *= 1.1
            p['max_integral'] *= 0.90
            changes.append(f"超调{result['overshoot_pct']:.0f}% → Kp×0.95 Kd×1.1 max_integral×0.9")

        # ---- Kp: 升温速度 ----
        if result['rise_time'] is not None:
            if result['rise_time'] > 120:
                p['kp'] *= 1.2
                changes.append(f"升温{result['rise_time']:.0f}s过慢 → Kp×1.2")
            elif result['rise_time'] > 90:
                p['kp'] *= 1.1
                changes.append(f"升温{result['rise_time']:.0f}s偏慢 → Kp×1.1")

        # ---- Ki, max_integral: 稳态误差 ----
        if abs(result['steady_err']) > 3:
            p['ki'] *= 1.3
            p['max_integral'] = min(500, p['max_integral'] * 1.15)
            changes.append(f"静差{result['steady_err']:+.1f}°C → Ki×1.3 max_integral×1.15")
        elif abs(result['steady_err']) > 2:
            p['ki'] *= 1.15
            p['max_integral'] = min(500, p['max_integral'] * 1.1)
            changes.append(f"静差{result['steady_err']:+.1f}°C → Ki×1.15 max_integral×1.1")

        # ---- Ki, Kd: 稳态波动 ----
        # 波动靠降低Ki（减少积分震荡）+ 增大Kd（增加阻尼），不动min_output
        if result['steady_ripple'] > 3:
            p['ki'] *= 0.8
            p['kd'] *= 1.2
            changes.append(f"波动{result['steady_ripple']:.1f}°C过大 → Ki×0.8 Kd×1.2")
        elif result['steady_ripple'] > 2:
            p['ki'] *= 0.9
            p['kd'] *= 1.1
            changes.append(f"波动{result['steady_ripple']:.1f}°C偏大 → Ki×0.9 Kd×1.1")

        # ---- 限幅保护 ----
        p['kp'] = max(0.1, min(20.0, p['kp']))
        p['ki'] = max(0.0, min(5.0, p['ki']))
        p['kd'] = max(0.0, min(500.0, p['kd']))
        p['min_output'] = min(p['max_output'], max(0.0, p['min_output']))
        p['max_integral'] = max(p['min_integral'] + 1, min(500.0, p['max_integral']))

        if not changes:
            changes.append("指标在合理范围，收敛中")

        return p, changes

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.fan(1, False)
            self.fan(2, False)
            self.ser.close()


def main():
    if len(sys.argv) < 2:
        print("用法: python3 tools/pid_autotune.py <串口> [波特率]")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    ch = int(input("通道 (1=纤维, 2=树脂): "))
    target = float(input("目标温度 (°C): "))

    if ch not in (1, 2):
        print("无效通道"); sys.exit(1)

    tuner = AutoTuner(port, baud)
    tuner.temp_key = 'temp_cf' if ch == 1 else 'temp_resin'
    tuner.target_key = 'target_cf' if ch == 1 else 'target_resin'
    tuner.start()

    def cleanup(*_):
        print("\n[结束] 关闭风扇/加热...")
        tuner.stop_heat(ch)
        tuner.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, cleanup)

    current = dict(ORIG_CF if ch == 1 else ORIG_RESIN)
    iteration = 1
    ch_name = CH_NAMES[ch]

    print(f"\n{'='*60}")
    print(f"  PID 自动调优 — {ch_name}  目标 {target}°C")
    print(f"  原始参数: {dict((k,f'{v:.2f}') for k,v in current.items())}")
    print(f"{'='*60}")

    tuner.fan(ch, True)
    print("[风扇] 已开启\n")

    while True:
        print(f"{'#'*55}")
        print(f"  第 {iteration} 轮")
        print(f"{'#'*55}")

        tuner.clear()
        tuner.set_pid(ch, current)
        time.sleep(0.3)
        tuner.heat(ch, target)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = f"pid_logs/iter{iteration}_{ch_name}_{ts}.csv"
        tuner.wait_stable(target, tol=2.0, min_secs=20)
        tuner.stop_heat(ch)
        time.sleep(3)

        result = tuner.analyze(target, log_file, current)
        if result is None:
            print("[错误] 数据不足"); break

        ripple = result['steady_ripple']
        err = abs(result['steady_err'])
        overshoot = result['overshoot_pct']

        if ripple <= 2.0 and err <= 2.0 and overshoot <= 8.0:
            print(f"\n  ★ 达标！波动{ripple:.1f}°C ≤ 2°C")
            print(f"    推荐参数:")
            print(f"      Kp={current['kp']:.3f}  Ki={current['ki']:.3f}  Kd={current['kd']:.3f}")
            print(f"      max_output={current['max_output']:.1f}  min_output={current['min_output']:.1f}")
            print(f"      max_integral={current['max_integral']:.1f}  min_integral={current['min_integral']:.1f}")
            print(f"    可以将这些值填入 pid_ctrl.c 的全局参数中永久生效")
            break

        new_p, changes = tuner.suggest(current, result, target)
        print(f"\n  调整:")
        for c in changes:
            print(f"    • {c}")

        current = new_p
        iteration += 1

        if iteration > 15:
            print("[结束] 达到最大迭代次数"); break

        print(f"\n  冷却中...")
        tuner.wait_cool(45)

    tuner.fan(ch, False)
    tuner.stop()
    print(f"\n调优结束，共 {iteration} 轮，日志在 pid_logs/")


if __name__ == '__main__':
    main()
