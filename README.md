# HEACON 按摩艾灸机器人

华炎 E05-L1345_Pro 机械臂 + Intel RealSense D435i 的理疗控制台。一块屏幕做操作页，
一台电脑当控制台，一台平板当遥控器。

- **艾灸**（方案 p1/p2）：每个穴位悬停在穴上方 30 mm，柔顺导纳。推开会让开，松手拉回。
- **按摩**（方案 p3/p4）：落到工作高度后力控一直开着。穴上 Z 恒力下压，穴间抬起走线，
  空中按轨迹柔顺，末端姿态锁朝下。被外力推开不会结束服务。

---

## 一分钟上手（没有机械臂也能跑）

```bash
git clone <repo-url> && cd massage_robot
scripts/setup.sh
source .venv/bin/activate
python main.py --simulate
```

浏览器打开 <http://127.0.0.1:8765/>。`--simulate` 会用一个假机械臂和合成画面，
整条流程都能走：就位、开始服务、走线、手法、换穴。不需要真机，也不需要摄像头。

---

## 在真机上跑

```bash
scripts/setup.sh --all          # 含 RealSense 依赖和语音模型
source .venv/bin/activate
python main.py all              # 控制台 + 平板入口 + 摄像头
```

Ubuntu 24.04 的正式部署（systemd、udev、证书、逐项验证）见
[`deploy/systemd/README.md`](deploy/systemd/README.md)。

---

## 目录

```
main.py                 入口：console / tablet / camera / all
heacon/
  config.py             所有路径、端口、机械臂地址都在这里
  camera_daemon.py      RealSense 采集，唯一打开摄像头的进程
  robot/
    client.py           华炎 TCP 客户端
    simulator.py        假机械臂（HEACON_SIMULATE=1）
  motion/
    route.py            服务状态机：走线、贴穴、手法、力控
    constants.py        所有调机参数（改这里会改变手感）
    plans.py            四个方案的穴位与手法（产品内容）
    demo.py             演示用的左右慢移
  vision/
    geometry.py         相机坐标 ↔ 机械臂坐标
    body.py             画面里找人、定位穴位
    synthetic.py        合成画面，只在模拟模式用
  voice/
    stt.py              平板语音识别
    tts.py              语音提示音
  http/
    app.py              控制台 8765 + 控制 API
    https_gateway.py    平板 HTTPS 入口 8766
web/
  mmrobot-ui.html       操作页面
  heacon-link.js        页面与控制台的连接层
  voices/               预生成的语音片段
  pwa/                  平板加桌面用的 manifest 与 service worker
config/                 标定与示教位（只放示例，真值不进仓库）
deploy/                 systemd / udev / launchd / Ubuntu 安装脚本
docs/                   交接与排障
tests/                  不需要硬件的测试
```

---

## 配置

所有可调项走环境变量，默认值在 [`heacon/config.py`](heacon/config.py)。
复制 `.env.example` 改成自己的值即可。最常改的两个：

| 变量 | 默认 | 说明 |
|---|---|---|
| `HEACON_ROBOT_HOST` | `192.168.0.10` | 机械臂 IP |
| `HEACON_SIMULATE` | `0` | `1` = 假机械臂，**现场必须为 0** |

查看当前解析结果：

```bash
python main.py --print-config
```

---

## 接口（控制台 8765）

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/` | 操作页面 |
| GET | `/api/state` | 机械臂、会话、画面状态 |
| GET | `/api/camera.jpg` | 当前画面。`X-Heacon-Camera` 为 `live` / `stale` / `missing` / `simulated` |
| GET | `/api/service/status` | 服务进度 |
| POST | `/api/session/connect` | 建立操作员会话，机械臂进入守候 |
| POST | `/api/position` | 走到就位位 |
| POST | `/api/service/start` | 开始服务 |
| POST | `/api/service/stop` | 停止 |
| POST | `/api/service/speed` | 调速 |
| POST | `/api/service/level` | 调档 |

平板走 8766，同样的路径，HTTPS 转发到 8765。浏览器只在 HTTPS 下允许用麦克风，
所以平板必须走 8766。

---

## 测试

```bash
.venv/bin/python -m pytest tests/ -q
```

全部跑在假机械臂上，不需要硬件。其中 `tests/test_console_smoke.py` 会真的启动控制台、
建立会话、走到就位位、开一次按摩服务并确认它推进到第二个手法——这就是"开箱即用"的直接证明。

---

## 安全约定

这些是现场踩出来的，改代码前请先读：

- **不要用 `GrpOpenFreeDriver`**。它放开全部关节，机械臂会掉。
- **力控只在工作高度开**。行程中开 Z 探寻会让状态机卡在运动态。
- **手上还按着末端时不要清零**。带载清零会让力值反号，机械臂猛冲。
- **离线必须锁定机械臂**（下使能）。这是默认行为，别绕开。
- **不要替操作员点「开始」**。真机服务只能由现场的人发起。
- **重启控制台会掉平板会话**，操作员需要重连。摄像头可以随时重启，不影响其他。

---

## 版本

`docs/HANDOFF.md` 记录了早期手感和踩坑，历史背景用，不是当前行为说明。
标定文件（`handeye.json`、`color_calib.json`、`work_home.json`）与机器绑定，不进仓库：
仓库里只有 `*.example.json`，首次上线必须在本机重新标定。
