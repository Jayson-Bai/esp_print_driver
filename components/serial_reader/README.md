# 串口挤出指令（E）说明

## 串口行格式

- 行协议：每条命令以 `\n` 结尾（`\r` 会被忽略）。
- 挤出命令为绝对长度：

```
E <seq> <tool_id> <total_mm>\n
```

- `seq`：仅解析，不参与逻辑。
- `tool_id`：`1`=纤维挤出（motor3），`2`=树脂挤出（motor4）。其它值直接忽略。
- `total_mm`：绝对挤出长度（不是增量），固件会用“新值-上次值”得到本次增量。

## 接收与解析流程

1. `serial_rx_task` 从 UART 读取数据并按 `\n` 组包成一行。
2. `serial_parse_task` 去尾空白并解析命令。
3. 匹配 `E %d %d %f` 后入队为 `UART_CMD_EXTRUDE`。
4. `command_dispatch_task` 调用 `handle_extrude_cmd(tool_id, total_mm)`。
5. `handle_extrude_cmd` 内部调用 `extruder_set_absolute(tool_id, total_mm)`。

## 电机控制逻辑（挤出）

- `extruder_set_absolute`：
  - `delta_mm = total_mm - last_abs_mm[tool]`。
  - 按 `steps_per_mm_for_tool(tool_id)` 转成步数。
  - 累加到 `g_target_steps_total`，并记录 `g_host_steps_per_s`。
- `extruder_task` 周期运行（`EXTRUDER_SLICE_US = 500us`）：
  - 计算误差：`g_target_steps_total - g_emit_steps_total`。
  - 方向判定：误差为正 → `CCW`（挤出），为负 → `CW`（回抽）。
  - 速度：`host_rate + EXTRUDER_ERR_K * abs(error)`。
  - 以分片方式下发步进（`EXTRUDER_MAX_STEPS` 上限），用 RMT 输出脉冲。
  - 每次输出后更新 `g_emit_steps_total`。

## 注意事项

- 这是**绝对挤出模式**。上位机重启后需重置绝对计数，避免一次性产生巨大增量。
- 重置命令（事件类）：

```
EV <seq> extrude_reset <tool_id>\n
```

- 线缓冲长度 256 字节，超长行会被丢弃。

## 相关源码

- `components/serial_reader/serial_reader.c`
- `components/motor/extruder_ctrl.c`
- `components/motor/motor.h`（引脚与微步配置）
