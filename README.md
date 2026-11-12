# HEACON 按摩艾灸机器人

华炎 E05-L1345_Pro 机械臂 + Intel RealSense D435i 的理疗控制台。

平板当遥控器，一台电脑（NUC 或任意 Linux / macOS）当控制台。这台电脑通过
TCP 直接控制机械臂，所以**只要网络能通到机械臂，笔记本可以替代 NUC**。

- **艾灸**（方案 p1/p2）：每穴上方 30 mm 柔顺导纳，推开会让开，松手拉回。
- **按摩**（方案 p3/p4）：落到工作高度后力控常开，穴上 Z 恒力，穴间抬起来走线，
  末端姿态锁朝下。被外力推开不会结束服务。

本仓库对应的是 **NUC 现场版（2026-11-04）**，即在 NUC 上调通并验收的那一版。

---

## 三种身份，先搞清楚自己在哪

| | 作用 | 能不能替代 |
|---|---|---|
| **NUC**（192.168.0.100） | 现场在跑的控制台，平板连它 | 基准，**不要改动** |
| **你的笔记本** | 装了同一套程序，直接连机械臂 | ✅ 可以完全替代 NUC |
| **平板** | 浏览器打开控制台页面 | 只负责显示和点按 |

**笔记本替代 NUC 的前提**只有一条：**笔记本和机械臂在同一网段**。
因为笔记本要直接连 `192.168.0.10:10003`。

⚠️ 同一时刻只能有一台电脑控制机械臂。笔记本要试的时候，先让 NUC 上的服务停下，
否则两边会抢同一个 TCP 会话。

---

## 一分钟上手（无硬件也能跑）

```bash
git clone <repo-url> && cd massage_robot
scripts/setup.sh
source .venv/bin/activate
python main.py --simulate
```

打开 <http://127.0.0.1:8765/>。`--simulate` 用假机械臂和合成画面，整条流程都能走。
不需要机械臂，不需要摄像头。

---

## 接真机械臂

### 1. 网络

```bash
ping 192.168.0.10                    # 机械臂控制器
timeout 3 bash -c 'cat < /dev/null > /dev/tcp/192.168.0.10/10003' && echo OPEN
```

不通就先解决网络：网线、IP 网段、防火墙。**这一步不通，后面全白搭。**

### 2. 安装

```bash
scripts/setup.sh --all      # 含 RealSense 依赖、识别模型、语音模型
source .venv/bin/activate
python main.py --print-config
python main.py all          # 控制台 + 平板入口 + 摄像头
```

### 3. 自检

```bash
python scripts/check_force_control.py
```

全部 PASS 才算能用。它会实际打开一次力控再关掉，验证机械臂响应。

### 4. 平板

浏览器打开 `https://<这台电脑的局域网IP>:8766`。自签证书会提示不安全，继续访问即可。

---

## Ubuntu 24.04 正式部署（systemd）

见 [`deploy/systemd/README.md`](deploy/systemd/README.md)，或直接：

```bash
sudo deploy/install-ubuntu.sh
```

---

## 目录

```
main.py                 入口：console / tablet / camera / all
heacon/
  config.py             所有路径、端口、机械臂地址
  camera_daemon.py      RealSense 采集（彩色 + 深度），唯一打开摄像头的进程
  robot/
    client.py           华炎 TCP 客户端
    simulator.py        假机械臂（HEACON_SIMULATE=1）
  motion/
    route.py            服务状态机：走线、贴穴、手法、力控
    constants.py        调机参数（改这里会改变手感）
    plans.py            四个方案的穴位与手法
    demo.py             演示用左右慢移
  vision/
    hybrid_geom.py      穴位识别（MediaPipe + 深度几何）★ 核心
    body.py             识别入口，只走 hybrid_geom
    geometry.py         相机坐标 ↔ 机械臂坐标
    synthetic.py        合成画面（仅模拟模式）
  voice/                平板语音识别与提示音
  http/
    app.py              控制台 8765 + 控制 API
    https_gateway.py    平板 HTTPS 入口 8766
web/
  mmrobot-ui.html       操作页面
  heacon-link.js        页面与控制台的连接层
  voices/               预生成语音
  pwa/                  平板 manifest
config/                 标定与示教位（只放示例，真值不进仓库）
deploy/                 systemd / udev / launchd / Ubuntu 安装脚本
docs/                   交接、排障
scripts/                安装、下载模型、自检、诊断
tests/                  不需要硬件的测试
```

---

## 识别为什么必须用深度

`heacon/vision/hybrid_geom.py` 的背部定位靠**深度图**判断人体凸起：

- 彩色图只看颜色，桌子、纸张、衣服都会干扰
- 深度图能直接分出「比桌面高 25 mm 以上」的区域，稳定得多

所以 `camera_daemon` 必须同时输出：

```
runtime/color-live.jpg     彩色帧（给页面显示）
runtime/depth-live.png     深度图（给识别用）
```

**只给彩色帧，识别会返回「没有人体」。** 这是这套系统最容易踩的坑。

---

## 配置

所有可调项走环境变量，默认值在 [`heacon/config.py`](heacon/config.py)。
复制 `.env.example` 改自己的值。最常改的：

| 变量 | 默认 | 说明 |
|---|---|---|
| `HEACON_ROBOT_HOST` | `192.168.0.10` | 机械臂 IP |
| `HEACON_HTTP_PORT` | `8765` | 控制台端口 |
| `HEACON_HTTPS_PORT` | `8766` | 平板入口端口 |
| `HEACON_SIMULATE` | `0` | `1` = 假机械臂，**现场必须为 0** |
| `HEACON_RUNTIME_DIR` | `./runtime` | 帧、日志、证书 |
| `HEACON_CONFIG_DIR` | `./config` | 标定文件 |

---

## 测试

```bash
.venv/bin/python -m pytest tests/ -q
```

全部跑在假机械臂上，不需要硬件。

---

## 安全约定

这些是现场踩出来的，改代码前请先读：

- **不要用 `GrpOpenFreeDriver`**。它放开全部关节，机械臂会掉。
- **力控只在工作高度开**。行程中开会让状态机卡在运动态。
- **手上还按着末端时不要清零**。带载清零会让力值反号，机械臂猛冲。
- **不要替操作员点「开始」**。真机服务只能由现场的人发起。
- **同一时刻只能一台电脑连机械臂**。
- **不要用 Mac 旧版覆盖这一版**。Mac 第一/二/三版是 `body_route` 旧识别路径，
  迁到 NUC 会退回错误的穴位图。

---

## 与旧版的关系

| 版本 | 说明 |
|---|---|
| Mac 第一 / 二 / 三版 | macOS 现场版，识别走旧的 `body_route` 轮廓回退 |
| 艾灸成品1 | Mac 上艾灸调通的那一版 |
| **NUC 第一版（本仓库）** | Ubuntu 24.04，识别换成 `hybrid_geom`，现场验收通过 |

力控与手法代码在 Mac 第三版和 NUC 版之间**逐字节相同**，差异只在识别和部署方式。
可以用 `scripts/compare_v3_force_path.py` 复验。
