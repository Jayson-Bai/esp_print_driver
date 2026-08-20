#!/usr/bin/env python3
"""M1/M2 自动找零时的限位毛刺回归测试。

不需要 ESP-IDF、串口或真实硬件。它覆盖两个目标：

1. 以状态模型复现当前固件的危险路径：下降沿毛刺会在自动下探时
   直接关闭驱动，并把一次非真实限位当作已归零。
2. 检查 ``components/motor/motor.c`` 是否仍包含该危险实现。

运行方式：

    python3 tools/test_limit_interrupt_regression.py

当前代码下，测试应报告 ``已确认当前回归`` 并以 0 退出；这表示测试
成功重现了问题，而不是表示固件已经安全。修复后使用：

    python3 tools/test_limit_interrupt_regression.py --assert-fixed

该模式会要求 ISR 不直接失能电机/写入归零状态，并要求找零逻辑具有
稳定低电平确认；当前版本预期失败。
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
MOTOR_SOURCE = REPO_ROOT / "components" / "motor" / "motor.c"


def c_function_body(source: str, signature: str) -> str:
    """返回 C 函数体；用花括号计数，避免正则跨嵌套 if 时误截断。"""
    start = source.find(signature)
    if start < 0:
        raise AssertionError(f"未找到函数：{signature}")
    open_brace = source.find("{", start)
    if open_brace < 0:
        raise AssertionError(f"未找到函数体：{signature}")

    depth = 0
    for index in range(open_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[open_brace + 1:index]
    raise AssertionError(f"函数体未闭合：{signature}")


class AxisState:
    """仅保存本问题需要的轴状态，不模拟真实 GPIO/RMT。"""

    def __init__(self) -> None:
        self.direction_cw = True
        self.driver_enabled = True
        self.limit_triggered = False
        self.homed = False
        self.position_steps = 48000  # 已在安全高度 60 mm 的软件坐标
        self.raw_limit_low = False


def current_isr_falling_edge(axis: AxisState) -> None:
    """当前 ISR 的关键副作用（对应 motor.c:321-329）。"""
    if axis.direction_cw:
        axis.limit_triggered = True
        axis.driver_enabled = False


def current_position_finish(axis: AxisState) -> None:
    """当前 motor_position_impl 的结算逻辑（对应 motor.c:798-805）。"""
    if axis.limit_triggered:
        axis.position_steps = 0
        axis.homed = True
    else:
        axis.homed = False


def desired_isr_falling_edge(axis: AxisState) -> None:
    """修复后的 ISR 应只唤醒监测任务，不改变运动/原点状态。"""
    del axis


def desired_confirm_stable_limit(axis: AxisState, samples: list[bool]) -> bool:
    """只有下探期间连续低电平采样成功，才允许停止并建立 0 点。"""
    if not axis.direction_cw or len(samples) < 5 or not all(samples[-5:]):
        return False
    axis.limit_triggered = True
    axis.driver_enabled = False
    axis.position_steps = 0
    axis.homed = True
    return True


class LimitNoiseModelTest(unittest.TestCase):
    def test_current_firmware_model_reproduces_false_stop(self) -> None:
        """毛刺消失后，当前逻辑仍已关驱动并伪造了 M1 原点。"""
        axis = AxisState()

        # GPIO13 出现一个下降沿，但下一次轮询前恢复为高电平。
        current_isr_falling_edge(axis)
        axis.raw_limit_low = False
        current_position_finish(axis)

        self.assertFalse(axis.driver_enabled)
        self.assertTrue(axis.limit_triggered)
        self.assertTrue(axis.homed)
        self.assertEqual(axis.position_steps, 0)
        self.assertFalse(axis.raw_limit_low)

    def test_desired_model_rejects_short_noise_pulse(self) -> None:
        """同一毛刺不能停止电机，也不能污染 0 点。"""
        axis = AxisState()

        desired_isr_falling_edge(axis)
        accepted = desired_confirm_stable_limit(axis, [False, True, False])

        self.assertFalse(accepted)
        self.assertTrue(axis.driver_enabled)
        self.assertFalse(axis.limit_triggered)
        self.assertFalse(axis.homed)
        self.assertEqual(axis.position_steps, 48000)

    def test_desired_model_accepts_real_limit(self) -> None:
        """真实限位仍会安全停止并将该轴设为 0 点。"""
        axis = AxisState()

        desired_isr_falling_edge(axis)
        accepted = desired_confirm_stable_limit(axis, [True, True, True, True, True])

        self.assertTrue(accepted)
        self.assertFalse(axis.driver_enabled)
        self.assertTrue(axis.limit_triggered)
        self.assertTrue(axis.homed)
        self.assertEqual(axis.position_steps, 0)


class SourceSafetyContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = MOTOR_SOURCE.read_text(encoding="utf-8")
        cls.isr_body = c_function_body(
            cls.source, "static void IRAM_ATTR stop_gpio_isr_handler"
        )
        cls.monitor_body = c_function_body(cls.source, "void stop_monitor_task")
        cls.position_body = c_function_body(cls.source, "void motor_position_impl")

    def test_isr_is_notification_only(self) -> None:
        self.assertNotIn("gpio_set_level(EN_PIN_1, 1)", self.isr_body)
        self.assertNotIn("gpio_set_level(EN_PIN_2, 1)", self.isr_body)
        self.assertNotIn("limit_triggered_1 = true", self.isr_body)
        self.assertNotIn("limit_triggered_2 = true", self.isr_body)

    def test_position_requires_confirmed_low_limit(self) -> None:
        self.assertIn(
            "axis_limit_stably_active(motor_id)",
            self.position_body,
            "自动找零必须在 motor_position_impl 内确认稳定低电平",
        )

    def test_monitor_zeros_only_while_lowering(self) -> None:
        m1_start = self.monitor_body.index("if (last_level_1 == 1 && level1 == 0")
        m1_end = self.monitor_body.index("last_level_1 = level1", m1_start)
        m1_edge_block = self.monitor_body[m1_start:m1_end]
        self.assertLess(
            m1_edge_block.index("motor1_current_dir == CW"),
            m1_edge_block.index("motor1_position_steps = 0"),
            "M1 仅可在向下找限位时清零",
        )

        m2_start = self.monitor_body.index("if (last_level_2 == 1 && level2 == 0")
        m2_end = self.monitor_body.index("last_level_2 = level2", m2_start)
        m2_edge_block = self.monitor_body[m2_start:m2_end]
        self.assertLess(
            m2_edge_block.index("motor2_current_dir == CW"),
            m2_edge_block.index("motor2_position_steps = 0"),
            "M2 仅可在向下找限位时清零",
        )


def run_current_regression_confirmation() -> int:
    """当前模式：模型必须能复现问题，源码必须仍含该实现。"""
    suite = unittest.TestSuite()
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(LimitNoiseModelTest))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():
        return 1

    source = MOTOR_SOURCE.read_text(encoding="utf-8")
    required_current_patterns = (
        ".intr_type = GPIO_INTR_NEGEDGE",
        "gpio_set_level(EN_PIN_1, 1)",
        "gpio_set_level(EN_PIN_2, 1)",
        "if (*limit_triggered) {",
    )
    missing = [pattern for pattern in required_current_patterns if pattern not in source]
    if missing:
        print("\n当前源码已不再包含完整的已知回归特征。")
        print("请使用 --assert-fixed 验证修复后的安全契约。")
        return 0

    print("\n已确认当前回归：短暂限位下降沿可关闭轴驱动并伪造 0 点。")
    print("运行 --assert-fixed 可验证后续修复是否满足安全契约。")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--assert-fixed",
        action="store_true",
        help="按修复后的安全契约检查源码；当前版本预期失败。",
    )
    args = parser.parse_args()

    if args.assert_fixed:
        result = unittest.TextTestRunner(verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(SourceSafetyContractTest)
        )
        return 0 if result.wasSuccessful() else 1
    return run_current_regression_confirmation()


if __name__ == "__main__":
    sys.exit(main())
