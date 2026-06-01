#pragma once

void extruder_task_start(void);
void extruder_set_absolute(int tool_id, float total_mm);
void extruder_reset_absolute(int tool_id);
void extruder_ui_start(int tool_id, int dir_sign);
void extruder_ui_stop(int tool_id);
