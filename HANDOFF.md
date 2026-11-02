# HEACON / mmrobot-web 交接

日期：2026-10-26。本文件根据当晚会话、当前源码和 8765 现场日志整理。工作区 **不是 git 仓库**（`git status` 报 `fatal: not a git repository`），无法用 diff 对照历史版本；下面的文件与行号以写本文时磁盘上的代码为准。

不要提交，除非用户明确要求。不要覆盖桌面上的封存 freeze zip。不要用 `GrpOpenFreeDriver`。不要为了“测手感”自动点「确认路线并开始」。离线必须锁定机械臂。

---

## 1. 项目目标和用户需求

HEACON 是华炎 E05-L1345_Pro 机械臂 + USB Intel RealSense D435i 的理疗控制台。Mac 是大脑，平板（`com.heacon.pad` / UA `HeaconPad`）是遥控器。本仓库是 Vue/静态操作页 + Python 桥。

用户角色是产品经理，要直接在代码和真机上迭代。本轮核心产品手感（用户原话，必须按这个理解）：

- 手持续拉末端：放下原路线，优先顺着这个力自由被牵引（平拉、斜拉、带高度都可以）。
- 松手后确认没力了，再慢慢直线回正，继续工作。不是弹簧，也不是一松手就乱走。
- 身体高低起伏（接触反力）：XY 仍走原路线，只顺势改高度，尽量保持接触力大小。
- 末端保持朝下看，不要用 6 轴去滚画面。
- 本次相机是 USB RealSense，不要把相机选择写死。
- 不要主动掉电/锁定，除非操作员断开或离线。
- 不要启动真实服务来替用户点「开始」。离线 = 锁定。

用户明确不要的手感：顶手对抗、一卡一卡、一动就锁死、一拖就乱动、只能水平不能斜拉、松手后乱走。

---

## 2. 已经完成的工作（时间顺序）

### 本轮之前（同一项目、较早会话，代码仍在）

- 操作页 + 桥 + 手臂 TCP + 相机 + 平板遥控已能走通手法。
- 平板扫描死区：`public/heacon-link.js` 用顶层 `lastFitKey`，缓存 `fs36`；`canEnter('scan')` 仍要求 `flowState.planConfirmed`。
- 力位混合第一版：优先厂家力控自由驱动（`SetFTFreeDriveState`），失败则软件 `WayPoint`/`MoveL` 跟手。
- 现场连续修过：空推无反应后卡死、一卡一卡、一动就锁死、一拖乱动。曾把跟手收成 XY-only 止乱动，用户当场否定（“手也可能斜着拉”）。

### 2026-10-26 本对话（约 20:45–21:25）

1. **斜着拉**：恢复手拖三维（含 Z），自由度和软件目标都不再锁死高度。
2. **跟手定义对齐**：持续用力不回；松手约 0.8s 后再匀速回正；身体起伏不是手拖。
3. **对抗感**：日志证实服务中 `force free-drive fallback`，全程位置伺服。曾改为先松开路径再跟。
4. **优先级**：手持续拉放下路线；身体起伏只改高度。`_wait_settled` 里用 `_hybrid_body_on_path` 改同一 XY 目标的高度。
5. **仍顶手 / 不跟着走**：现场有 `hand lead, path aside`，但自由驱动没开上；软件跟手里对 40010 反复 `pause()`，等于刹住末端。
6. **跟手预览**：用户要求先解锁、用和服务同一套反应试拖，再上平板。新增 `POST /api/lead/start|stop`。第一版预览仍走软件短跳，用户反馈顶手、卡顿、松手乱动。日志里大量 `hand follow`，回程方向乱跳。
7. **改预览为先开厂家自由牵引，松手直线回锚点**。多次尝试 `SetForceControlState` 后立刻 `SetFTFreeDriveState`，全部 `40010 Robot is in Moving state`。在力控运动态里 `GrpInterrupt` 曾把 FSM 打到 24/22。预览失败后已停。
8. **用户自行用页面开了一次服务**（21:20，`p1` 肩颈温阳灸 / 大椎循经灸）。手法中再次出现 `hand lead, path aside` + 一串 `hand follow`，没有 `force free-drive on`。约 21:22 服务结束，手臂回到待机。
9. **用户要求只写本交接文档，不要改代码、不要继续测试。**

---

## 3. 修改过的文件、关键位置和原因

本对话主要改了桥。更早会话还动过前端和力控封装，下列一并标出现状。

### 本对话改过

| 文件 | 关键位置 | 原因 |
|---|---|---|
| `bridge/route_motion.py` | `HYBRID_*` 约 224–245 行 | 跟手阈值、包络 80/40 mm、回正 0.8s、跟手步长/速度、自由牵引柔顺度 80/8 |
| 同上 | `_hybrid_sample` 约 356–419 行 | 标定力优先，否则 raw−bias；转到基座；接触=与桌面法向点积；侧向=去掉接触后的力；低通 0.35/0.65；`lead` 为跟手方向 |
| 同上 | `_hybrid_is_drag` / `_hybrid_held_yield` 约 430–450 行 | 手拖 = 侧向≥2.8 N 或上提接触≤−3.0 N；持续约 80ms 才切入。正接触按压不是手拖 |
| 同上 | `_hybrid_open_drive` / `_hybrid_try_free_drive` 约 669–719 行 | 仅 FSM 33 时开力控，立刻 `SetFTFreeDriveState`；失败则关力控。当前现场仍 40010 |
| 同上 | `_hybrid_yield_and_return` 约 879–909 行 | 记下松手前位姿，走软件跟手，结束关力控 |
| 同上 | `_hybrid_yield_software` / `_hybrid_drop_path` 约 985–1047 行 | 打断原路径一次，按力方向短 `WayPoint`；`lead=True` 时 40010 不再 pause（避免刹死） |
| 同上 | `_hybrid_return_home` 约 958–982 行 | 关力控后直线回 home；回程不再重新切入跟手（防乱动） |
| 同上 | `_hybrid_body_on_path` 约 1092–1107 行；`_wait_settled` 约 2964–2971 行 | 非手拖时保持目标 XY，只改高度 |
| 同上 | `start_lead_preview` / `_lead_preview_loop` 约 2103–2221 行 | 解锁后预览：先开自由牵引，松手回锚点。开不起来就报错退出，不再用短跳预览 |
| `bridge/server.py` | `_ROBOT_LIVE` 含 `/api/lead/start|stop`；POST 约 309–312 行 | 跟手预览接口，需已建立会话 |

### 更早会话、本轮未再改但仍生效

| 文件 | 现状 |
|---|---|
| `public/heacon-link.js` | 顶层 `lastFitKey`；`startScan` 包装：`canEnter` → 非 `armLive` 用 mockScan，否则 realScan。缓存 `fs36` |
| `/Users/patrickyang/Desktop/mmrobot-ui.html` | `onclick="startScan()"`；脚本 `/heacon-link.js?v=fs36` |
| `bridge/robot_client.py` | TCP `192.168.0.10:10003`；命令以 `,;` 结尾；`Fail` 含 `,Fail,`。`pause`=`GrpInterrupt`，`stop`=`GrpStop`，`lock()` 仅在力控状态≠0 时关力控。力控封装见 186–246 行 |
| `bridge/https_gateway.py` | 8766 HTTPS，反代 8765，给平板麦克风 |
| `bridge/vision.py` | `_rotation` ZYX；`up_vector()` 来自手眼平面（桌面朝向机器人） |

厂家协议 PDF：`/Users/patrickyang/Desktop/SDK-2026年/HuayanRobotV6控制通信协议接口_V1.0.19.1.pdf`。力控摘录曾落到 `/tmp/heacon-ft-protocol.txt`。固件记录：HR6.5.20d.beta.T631.20260206。

---

## 4. 关键技术决策、约束和踩过的坑

### 必须遵守

- 华炎 TCP：`192.168.0.10:10003`，命令 `,;`，失败 `,Fail,`。
- FSM：7 初始化，21 碰撞，22 错误，24 下使能/锁定，25 运动，33 待机。
- 常见错误：20018/40004 当前状态禁止；40010 Moving；40020 不在力控。
- 力控：`SetForceControlState` 开启后机器人会跳到运动态。`ReadFTControlState`：0 关，1 探寻，2 探寻完成，3 自由驱动。自由牵引是 `SetFTFreeDriveState`，**不是** `GrpOpenFreeDriver`（关节零力，可能坠落）。
- `SetFTFreeFactor` 平移/旋转柔顺度 0–100。速度模式：0 正常，1 慢，2 快。
- 高度：`WORK_LIFT_MM=-30`；`WORK_FLOOR_MM=WORK_LIFT-22`；`SKIN_STANDOFF_MM=40`；`GRADE_STEP_MM=10`；`TRAVEL_CLEARANCE_MM=50`。
- 手拖包络相对切入点约 XY 80 mm、Z 40 mm；高度不低于工作面。
- 公开 `connected`/`live` 表示操作员会话。`isTabletRemote()` 在 localhost 为 false。离线 mockScan 不写记录。
- 行程中不要开 Z 探寻力控（曾让 FSM 一直 25，`_wait_settled` 等 33 挂死）。接触中不要 `SetForceZero`。
- 8765 若已有服务在跑，不要杀端口。重载只在 `running`/`positioning` 均为 false 时。
- 相机朝下；不用关节 6 滚画面。

### 已对齐的产品逻辑

手拖 / 间歇脉冲 / 身体起伏 三套，对应行业里的自由牵引 vs 力位混合 vs 松手停留再回。实现意图：

- 手拖：侧向或上提。还在用力（或自由牵引下位姿仍在动且力未静）就保持跟手。
- 松开：力静且基本不动约 0.80s，再匀速回切入点。
- 身体：正接触不是手拖；XY 继续原目标，Z 小步滤后跟接触力。

### 踩过的坑（按现象）

| 现象 | 原因 |
|---|---|
| 推了没反应又卡死 | `_hybrid_ft_goal()` 缺参 TypeError，服务线程死掉 |
| 锁定时报“正在运动，不能锁定” | idle lock 在运动中狂发 `SetForceControlState`。现 `lock()` 只在 FT 状态≠0 时关力控 |
| 手法全程卡在运动 | 一开始就开 Z 探寻力控，FSM 25，`_wait_settled` 死等 33 |
| 自由驱动从未打开 | `SetFTFreeFactor` 等在力控开启前发 → 40020；或运动中发 → 40010/20018 |
| 误让步 | cab 失败时把未去偏的 fx/fy（约 4N 重力）当侧向 |
| 垂直下降当卡住 | stall 只看 XY，8s 误判。后改 3D remain |
| 一动就锁死 | 一受力就 Stop + 长等 + 回程顶手 |
| 一拖乱动 | 软件 140mm 前瞻、把 up 叠进方向、频繁改目标 |
| 只能水平 | 为止乱动锁了 Z；用户否定斜拉 |
| 一直顶手 | 厂家自由驱动没开，位置环钉在原路径/短目标上 |
| 预览不跟着走 | 对每个 40010 `pause()`，形成刹车风暴 |
| 松手乱动 | 回程里 residual 力再次 `_hybrid_held_yield` 切入新方向。回程已改为不重新跟手 |
| 自由牵引开不起来 | `SetForceControlState,1` 后 FSM=25，`SetFTFreeDriveState` 稳定 40010。力控运动态里 `pause()` 曾到 FSM 24 再 22 |
| 较早一次日志里自由驱动开过三次 | 当时是 pause + 等待机约 0.7s + 再开力控并重试。本轮待机开力控后立刻切自由驱动，**没有再成功过** |

软件短跳（`WayPoint` 12–42 mm）用户已否决：必然顶手、卡顿。不要再把短跳当最终手感。

---

## 5. 当前代码与运行状态

### 跟手主路径（服务中）

`_wait_settled` / `_hybrid_follow` → 持续 80ms 手拖 → `_hybrid_yield_and_return` → **只走** `_hybrid_yield_software`（不再先开自由驱动）→ 松手 0.8s → `_hybrid_return_home` 直线回切入点。

`_hybrid_open_drive` 仍给跟手预览用：要求 FSM 33 → 探寻自由度全 0 → 开力控 → 立刻自由驱动。失败则关力控。

### 进程与端口（写本文时）

- **8765** 在听：`Python` PID **94727**（父进程 94684，标题为 “Reload 8765 with immediate free-drive”）。`0.0.0.0:8765`。
- **8766** 在听：`Python` PID **89203**（更早的 HTTPS 网关，本轮没有重载它）。反代 `http://127.0.0.1:8765`。
- 操作页：`/Users/patrickyang/Desktop/mmrobot-ui.html`，由 8765 提供。
- 手臂：华炎 E05-L1345_Pro，`192.168.0.10:10003`。
- 相机：USB RealSense D435i（本任务）。

### 设备状态（写本文时 `GET /api/state` 与 `/api/service/status`）

- 会话：`connected=true`，`live=true`，`session=true`，`phase=ready`。
- 手臂：FSM **33**，`ready=true`。
- 服务：`running=false`，`leadPreview=false`，`positioning=false`，`positionReady=true`。
- 残留字段（服务已结束未清干净）：`plan=p1`「肩颈温阳灸」，`point=左肩井`，`phase=手法`，`working=true`，`message=服务已结束，机械臂正在回到就位位置`。以 `running=false` 和 FSM 33 为准：服务不在跑，手臂待机。

---

## 6. 已验证、未验证、未解决

### 已验证

- 力传感器能采到：多次 `force ready`，偏置大约 fx≈−3.7～−3.9，fy≈−0.9，fz≈14.5～14.9。
- 手拖能被识别：服务和预览都打出 `hand lead, path aside`。
- 软件跟手会发短目标：大量 `hand follow dx dy dz`（三维，含 Z）。
- 厂家自由驱动在本轮预览中 **打不开**：连续 `40010`，随后 `lead preview: 末端没有放开，先不要拖`。
- 用户否定软件短跳手感（顶、卡、回程乱）。
- 用户否定 XY-only。
- 平板/页面走线、就位、开始服务仍可用：21:19 打开页面，21:20 `service/start` 先 409 后 200，语音「正在服务 大椎，循经灸」。
- 服务结束后手臂回到 FSM 33。力控运动态里 pause 造成 22 后，用 `/api/stop` + `/api/wake` 恢复过 33。

### 未验证 / 未解决

- **厂家 `SetFTFreeDriveState` 在 TCP 下的可靠时序仍未知。** 开力控必进 25，自由驱动被 40010 拒。这是跟手手感的主阻塞。
- 自由牵引一旦能打开，松手直线回锚点的预览循环 **没有成功跑过**。
- 身体起伏只改高度：代码在，真机按背起伏未单独验收。
- 间歇脉冲 0.8s 再回：被回程乱动和短跳盖过，未作为合格手感验收。
- 回程不再重新跟手：只改了代码，没有一次“松手后干净回原位”的用户确认。
- 8766 网关是旧进程，本轮没验证它是否和当前 8765 完全一致。
- 状态机残留 `working=true` / `phase=手法` 在服务结束后仍可能出现。

---

## 7. 最近一轮修改的具体情况

目标：预览时先真正放开末端，松手直线回原位，不再短跳。

实际代码（仍在磁盘上）：

1. `_hybrid_open_drive`：仅待机；探寻自由度全 0；`force_control_on`；马上最多 12 次 `set_ft_free_drive(True)`；成功后再设 XYZ 运动自由度、工具系、柔顺度 80/8、速度模式 0。失败关力控。
2. `_lead_preview_loop`：warm → 记锚点 → 最多 3 次 open_drive → 失败抛「末端没有放开」→ 成功则循环：自由牵引中看是否离开锚点；静且停约 0.8s 则关力控、`_hybrid_return_home`、再尝试开自由牵引。包络边缘先关驱动。
3. `_hybrid_return_home`：关力控，短等 33，匀速回 home，**回程不重新跟手**。
4. 服务路径仍是软件短跳跟手（`_hybrid_yield_and_return` 直接 `_hybrid_yield_software`）。21:20 用户自己开的 `p1` 服务走的就是这条，日志仍是 `hand follow` 短跳。

最后一次预览日志（21:17）：

```
force ready -3.89 -0.93 14.75
free-drive … 40010
free-drive … 40010
free-drive … 40010
lead preview: 末端没有放开，先不要拖，请再试一次
arm ready: 机械臂已就绪
```

没有 `force free-drive on`。

---

## 8. 下一步建议和安全注意事项

### 建议（先解决自由驱动，再谈手感）

1. 查华炎 V6 协议/示教器里 **力控自由驱动** 的官方顺序。重点：`SetForceControlState` 会进运动态，`SetFTFreeDriveState` 示例失败码是 40020（不在力控），现场却是 40010（运动中）。需要确认：是否必须在力控态且非 MoveL 时发；`SetFTMotionFreedom` 应在开力控前还是后；是否要先到 `ReadFTControlState=1/2` 再切 3。
2. 不要在力控运动态（FSM 25 + FT on）里 `GrpInterrupt`/`GrpStop`，本轮打出过 24/22。
3. 在确认 `ReadFTControlState==3` 且用户觉得不顶手之前，不要再用软件短跳当验收，也不要再拉用户拖末端。
4. 自由驱动可用之后：预览先验收「拉着走 / 松手直线回锚点」；再并回手法（运动中如何切入仍是第二问）。身体起伏保持 XY 路径 + Z。
5. 清掉服务结束后的 `phase`/`working` 残留，避免平板误以为还在手法。
6. 不要提交。工作区无 git remote 可推。

### 安全

- 禁止 `GrpOpenFreeDriver`。
- 自由牵引只开平移 XYZ，`rx,ry,rz=0`，避免末端翻转、相机离开朝下。
- 包络约 80/40 mm，高度不低于工作面。
- 离线必须 `lock()`（下使能）。不要在 `running`/`positioning` 时杀 8765。
- 真机服务只由操作员点开始。不要替用户点「确认路线并开始」。
- 掉电/锁定只在操作员断开或离线时。
- 不要覆盖桌面封存 freeze zip。
- 现场有人在臂边时，先看 FSM：21/22 先 `GrpReset` 再使能；25 先停稳再发力控。

### 给接手助手的第一件事

不要先改短跳参数。先搞清楚怎样让 `SetFTFreeDriveState` 在这台 HR6.5.20d 上回到状态 3。用户要的是被力牵引，不是位置环小步跟着走。当前 8765 已在跑、手臂待机已连接；重载 8765 会掉会话，用户需重新连接。8766 仍是旧网关进程。
