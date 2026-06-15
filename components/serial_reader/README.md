# 串口挤出指令（E）说明

## 串口行格式

- 行协议：每条命令以 `\n` 结尾（`\r` 会被忽略）。固件按 `\n` 组包解析，连续高频 `E` 命令会排队处理；固件所有 UART 输出行通过互斥写入，避免 `STAT`、`EVACK`、`EVDONE`、`EACK` 混流时互相穿插。
- 挤出命令为绝对长度：

```
E <seq> <tool_id> <total_mm>\n
```

- `seq`：用于固件侧去重，必须大于上一条已接受 `E` 的序号；收到 `extrude_reset` 后会清零去重状态，允许从 0 重新开始。
- `tool_id`：`1`=纤维挤出（motor3），`2`=树脂挤出（motor4）。测试 NPZ 使用树脂时应发送 `tool_id=2`，固件接受 `E ... 2 ...`。其它值直接忽略。
- `total_mm`：绝对挤出长度（不是增量），固件会用“新值-上次值”得到本次增量。固件只接受 `seq > last_e_seq` 的 `E`；接受后更新 `STAT last_e_seq/last_e_abs/last_e_us`，旧序号返回 `EWARN old_seq` 并丢弃。

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
- 重置命令（事件类）：固件收到后返回同 `seq`、同事件名的 `EVACK`；reset 真正执行完成并清除串口层 `last_e_seq/last_e_abs/last_e_us` 后，才返回同 `seq`、同事件名的 `EVDONE`。如果 reset 失败或事件未知，固件返回明确 `EVERR`，且不返回 `EVDONE`。

```
EV <seq> extrude_reset <tool_id>\n
```

- 上位机开始新轨迹前必须满足以下二选一：
  - 已收到本次 `extrude_reset` 对应的 `EVACK` 和 `EVDONE`，并从后续 `STAT` 看到 `last_e_seq=-1`、`last_e_abs=0.000`、`last_e_us=0` 后，才允许 `E` 序号从 0 开始。
  - 如果没有完成 reset 确认，则不得从 0 开始发 `E`，应读取当前 `STAT last_e_seq`，从 `last_e_seq + 1` 继续递增发送。
- reset 完成后的下一次或后续 `STAT` 必须显示 `last_e_seq=-1 last_e_abs=0.000 last_e_us=0`；`last_e_seq=-1` 是有符号语义，表示去重状态已清空。此后固件必须接受新的 `E 0 <tool_id> <total_mm>`。
- `STAT` 稳定提供上位机判断所需字段：`temp_cf/temp_resin/target_cf/target_resin/fan_ok_cf/fan_ok_resin/tool/err/last_e_seq/last_e_abs/last_e_us`。
- 手动准备命令（如 `EV 0 fan_resin 1`、`EV 0 heat_resin <temp>`）也返回 `EVACK`/`EVDONE`；防止 `E` 被 `old_seq` 丢弃的严格门控仍以后续 NPZ 中的 `extrude_reset` 确认链为准。
- 线缓冲长度 256 字节，超长行会被丢弃。

## 相关源码

- `components/serial_reader/serial_reader.c`
- `components/motor/extruder_ctrl.c`
- `components/motor/motor.h`（引脚与微步配置）
