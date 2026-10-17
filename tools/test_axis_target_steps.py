#!/usr/bin/env python3
"""Compile the real axis target calculation against a host-only motor stub."""
import pathlib
import subprocess
import tempfile
import unittest

from test_limit_interrupt_regression import c_function_body

ROOT = pathlib.Path(__file__).resolve().parents[1]


class AxisTargetStepsTest(unittest.TestCase):
    def test_safe_target_is_exact_from_every_step_on_both_axes(self):
        source = (ROOT / "components/motor/motor.c").read_text()
        body = c_function_body(source, "void motor_move_to_mm_impl(")
        harness = r"""
#include <stdlib.h>
#include <math.h>
typedef int motor_id_t;
typedef int motor_dir_t;
enum { MOTOR1=1, MOTOR2=2, CW=-1, CCW=1 };
#define motor1_microstep 8
#define motor2_microstep 8
int motor1_position_steps, motor2_position_steps;
int sent_steps, sent_direction;
float motor1_position_mm(void) { return motor1_position_steps / 800.0f; }
float motor2_position_mm(void) { return motor2_position_steps / 800.0f; }
void motor_send_command(int motor, int direction, int steps) {
    (void)motor; sent_steps=steps; sent_direction=direction;
}
void motor_move_to_mm_impl(motor_id_t motor_id, float target_mm) {
""" + body + r"""
}
int main(void) {
    for (int axis=1; axis<=2; ++axis) {
        for (int initial=-800; initial<=48800; ++initial) {
            motor1_position_steps=motor2_position_steps=initial;
            sent_steps=sent_direction=0;
            motor_move_to_mm_impl(axis, 60.0f);
            if (initial + sent_direction * sent_steps != 48000) return 1;
        }
    }
    return 0;
}
"""
        with tempfile.TemporaryDirectory() as directory:
            c_file=pathlib.Path(directory)/"target.c"
            binary=pathlib.Path(directory)/"target"
            c_file.write_text(harness)
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", str(c_file), "-lm", "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
