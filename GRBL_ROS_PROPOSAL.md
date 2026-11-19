# Proposal: changes to grbl_ros for the CNC Manager

**Status:** not implemented. grbl_ros is unchanged. This lists what the `cnc_manager`
node needs from `grbl_ros` and what it currently does as a workaround.

Files referenced: `grbl_ros_humble/grbl_ros/device.py`, `_command.py`, `_control.py`.

---

## 1. Periodic status polling inside grbl_node (high priority)

**Problem:** grbl_ros publishes `/<id>/state`, `/<id>/machine_position` and `/tf` only
when a `?` happens to be sent over serial. When the machine is idle, nothing is published.

**Workaround in cnc_manager:** sends `?` as a `send_gcode_cmd` goal every
`poll_rate_hz` (default 2 Hz) while nothing else is running. Side effects:
- grbl_ros logs a warning for each poll and aborts the goal, because GRBL answers `?`
  with a status report and no `ok`.
- Every poll adds a full action round trip.

**Proposal:** add a timer in `grbl_node.__init__`, for example 5 Hz, that writes `?` and
parses the answer. It has to share a serial lock with `send()` (see 3).

## 2. Real-time commands: feed hold, resume, soft reset, jog cancel (high priority, safety)

**Problem:** `command.send()` writes a line and then blocks on `readline()` with no
timeout. GRBL real-time commands (`!` feed hold, `~` resume, `0x18` soft reset,
`0x85` jog cancel) produce no `ok`, so sending them through `send_gcode_cmd` would hang
grbl_node forever.

**Effect today:** `/cnc/stop` in the manager cannot stop the machine. It only clears the
manager's queue. A jog with zero velocity cannot cancel a running jog; the last short jog
step (`jog_period_s`, default 0.1 s) finishes on its own.

**Proposal:**
- Add `command.send_realtime(byte)`, which writes the byte and returns without reading.
- Add a service `/<id>/realtime` with request `string command` (`hold|resume|reset|jog_cancel`)
  and response `bool success, string message`. Alternatively, reuse `Stop.srv`; see 4.
- Add a timeout to the serial port: `serial.Serial(..., timeout=1.0)`.

## 3. Thread-safe serial access (high priority)

**Problem:** grbl_node runs a `MultiThreadedExecutor`, and `send_gcode_file` calls
`send_gcode_cmd` through its own action client. Polling (1) or a stop (2) could write to
the serial port while another thread is in `readline()`, and responses get mixed up.

**Proposal:** add a `threading.Lock` around write+read in `command.send()`. Real-time
bytes skip the lock, because GRBL processes them immediately.

## 4. Stop service is broken (bug)

`device.py: stopCallback` reads `request.data`, but `Stop.srv` defines
`machine_id` and `command`. Every call raises `AttributeError` and crashes the callback.
It also never returns `response`.

**Fix:**
```python
def stopCallback(self, request, response):
    if request.command == 's':
        self.machine.disableSteppers()
    elif request.command == 'f':
        self.machine.enableSteppers()
    response.success = True
    return response
```
Until this is fixed, the manager keeps `use_grbl_stop_service:=false` (the default).

## 5. Action cancel support (medium)

The action servers have no `cancel_callback`, so every cancel is rejected.
`/cnc/send_file` therefore cannot be aborted.

**Proposal:** pass `cancel_callback=lambda goal: CancelResponse.ACCEPT` and check
`goal_handle.is_cancel_requested` in the `streamCallback` loop between lines. On cancel,
send feed hold + soft reset (2) and call `goal_handle.canceled()`.

## 6. Smaller bugs found while reading the code

| Where | Problem | Fix |
|---|---|---|
| `_control.py: home()` | `self.send(self, '$H')` passes `self` twice and raises TypeError | `self.send('$H')` |
| `device.py: __init__` | `max_z = self.get_parameter('x_max')` | use `'z_max'` |
| `device.py: gcodeCallback` | The `else` branch returns without `goal_handle.succeed()`/`abort()`, so rclpy logs "goal state not set" | call `goal_handle.abort()` |
| `device.py: gcodeCallback` | Loops only while state is `RUN`; `HOME` and `JOG` are not waited for | wait while state is in `{RUN, HOME, JOG}` |
| `device.py: streamCallback` | The file is opened twice and never closed; a missing file crashes the node | `with open(...)`, catch `FileNotFoundError` and abort |
| `config/cnc001.yaml` | `accleration` is a typo, so the parameter is ignored | `acceleration` |
| `command.send()` | Returns `responses[-1]`, which raises IndexError when GRBL answers nothing | handle empty responses |

## 7. Nice to have

- Publish work position (`WPos`) and feed/spindle (`FS:`) from the status report. The parser already splits the fields.
- Report the GRBL error code and its text in the `SendGcodeCmd` result (add `string message`).
