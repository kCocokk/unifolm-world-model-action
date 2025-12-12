# Unitree Deploy

本文档提供了为 Unitree G1 和 Z1 平台设置部署环境的说明，包括依赖安装、图像服务启动和夹爪控制。

# 0. 📖 简介

此代码库用于 Unitree 机器人模型的部署。

---

# 1. 🛠️ 环境设置

```bash
conda create -n unitree_deploy python=3.10 && conda activate unitree_deploy

conda install pinocchio -c conda-forge
pip install -e .

# 可选：安装 lerobot 依赖
pip install -e ".[lerobot]"

git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python  && pip install -e . && cd ..
```

---
# 2. 🚀 启动

**提示：确保所有设备处于同一局域网内**

## 2.1 🤖 运行 G1 和 Dex_1 夹爪

### 2.1.1 📷 图像捕获服务设置（G1 pc2）

[按照以下步骤启动 image_server](https://github.com/unitreerobotics/xr_teleoperate?tab=readme-ov-file#31-%EF%B8%8F-image-service)
1. 连接到 G1：
  ```bash
  ssh unitree@192.168.123.164  # 密码：123
  ```

2. 激活环境并启动图像服务：
  ```bash
  conda activate tv
  cd ~/image_server
  python image_server.py
  ```

---

### 2.1.2 🤏 Dex_1 夹爪服务设置（开发 PC2）

参考 [Dex_1 夹爪安装指南](https://github.com/unitreerobotics/dex1_1_service?tab=readme-ov-file#1--installation) 获取详细设置说明。

1. 进入服务目录：
  ```bash
  cd ~/dex1_1_service/build
  ```

2. 启动夹爪服务，**ifconfig 检查其自身的 dds 网络接口**：
  ```bash
  sudo ./dex1_1_gripper_server --network eth0 -l -r
  ```

3. 验证与夹爪服务的通信：
  ```bash
  ./test_dex1_1_gripper_server --network eth0 -l -r
  ```

---

### 2.1.3 ✅ 测试

执行以下测试以确保功能正常：

- **Dex1 夹爪测试**：
  ```bash
  python test/endeffector/test_dex1.py
  ```

- **G1 机械臂测试**：
  ```bash
  python test/arm/g1/test_g1_arm.py
  ```

- **图像客户端相机测试**：
  ```bash
  python test/camera/test_image_client_camera.py
  ```

- **G1 数据集回放**：
  ```bash
  # --repo-id     Your unique repo ID on Hugging Face Hub 
  # --robot_type     The type of the robot e.g., z1_dual_dex1_realsense, z1_realsense, g1_dex1, 

  python test/test_replay.py --repo-id unitreerobotics/G1_CameraPackaging_NewDataset --robot_type g1_dex1
  ```
---

## 2.2 🦿 运行 Z1

### 2.2.1 🦿 Z1 设置
克隆并构建所需的代码库：

1. 下载 [z1_controller](https://github.com/unitreerobotics/z1_controller.git) 和 [z1_sdk](https://github.com/unitreerobotics/z1_sdk.git)。

2. 构建代码库：
  ```bash
  mkdir build && cd build
  cmake .. && make -j
  ```

3. 复制 `unitree_arm_interface` 库：[根据您的路径修改]
  ```bash
  cp z1_sdk/lib/unitree_arm_interface.cpython-310-x86_64-linux-gnu.so ./unitree_deploy/robot_devices/arm
  ```

4. 启动 Z1 控制器 [根据您的路径修改]：
  ```bash
  cd z1_controller/build && ./z1_ctrl
  ```

---

### 2.2.2 ✅ 测试

运行以下测试：

- **Realsense 相机测试**：
  ```bash
  python test/camera/test_realsense_camera.py # 根据您的 Realsense 修改对应的序列号
  ```

- **Z1 机械臂测试**：
  ```bash
  python test/arm/z1/test_z1_arm.py
  ```

- **Z1 环境测试**：
  ```bash
  python test/arm/z1/test_z1_env.py
  ```

- **Z1 数据集回放**：
  ```bash
  # --repo-id     Your unique repo ID on Hugging Face Hub 
  # --robot_type     The type of the robot e.g., z1_dual_dex1_realsense, z1_realsense, g1_dex1, 

  python test/test_replay.py --repo-id unitreerobotics/Z1_StackBox_Dataset --robot_type z1_realsense
  ```
---

## 2.3 🦿 运行 Z1_Dual

### 2.3.1 🦿 Z1 设置和 Dex1 设置
克隆并构建所需的代码库：

1. 按照上述 Z1 步骤下载并编译代码，并下载夹爪程序以本地启动。

2. [根据文档修改多机控制](https://support.unitree.com/home/zh/Z1_developer/sdk_operation)

3. [下载修改后的 z1_sdk_1 并编译](https://github.com/unitreerobotics/z1_sdk/tree/z1_dual)，复制 `unitree_arm_interface` 库：[根据您的路径修改]
  ```bash
  cp z1_sdk/lib/unitree_arm_interface.cpython-310-x86_64-linux-gnu.so ./unitree_deploy/robot_devices/arm
  ```

4. 启动 Z1 控制器 [根据您的路径修改]：
  ```bash
  cd z1_controller/builb && ./z1_ctrl
  cd z1_controller_1/builb && ./z1_ctrl
  ```
5. 启动夹爪服务，**ifconfig 检查其自身的 dds 网络接口**：
  ```
  sudo ./dex1_1_gripper_server --network eth0 -l -r
  ```
---

### 2.3.2 ✅ 测试

运行以下测试：

- **Z1_Dual 机械臂测试**：
  ```bash
  python test/arm/z1/test_z1_arm_dual.py
  ```

- **Z1_Dual 数据集回放**：
  ```bash
  # --repo-id     Your unique repo ID on Hugging Face Hub 
  # --robot_type     The type of the robot e.g., z1_dual_dex1_realsense, z1_realsense, g1_dex1,

  python test/test_replay.py --repo-id unitreerobotics/Z1_Dual_Dex1_StackBox_Dataset_V2 --robot_type z1_dual_dex1_realsense
  ```
---
## 2.4 🦾 运行 D1（更新版）

> ⚠️ 注意  
> - 已在本仓库中集成 `D1ArmConfig`、`D1_ArmController`、`D1_OnlyArm_RobotConfig`、`D1_Realsense_RobotConfig` 等配置与控制代码。  
> - D1 控制采用「C++ 官方 SDK（`unitree_sdk2` + `D1_SDK`）+ 自定义 bridge」的方式：  
>   - C++ 进程 `d1_bridge` 通过 DDS 话题 `rt/arm_Command` / `current_servo_angle` 与机械臂通信；  
>   - Python 端通过 TCP socket（例如 `127.0.0.1:5555`）连接 `d1_bridge` 进行控制；  
>   - 不再直接依赖 `unitree_sdk2_python` 控制 D1。

### 2.4.1 🧩 D1 前置配置

#### 1）确认 D1 机械臂状态

1. 确认 D1 上电，机械臂已完成零点校准。
2. 确保开发电脑与 D1 在同一网段，并能 `ping` 通 D1（默认 IP 为 `192.168.123.100`，如已修改则以实际为准）：

   ```bash
   ping 192.168.123.100
   ```

3. 确认 D1 内部 marm 服务正常运行（出厂默认开启），或按官方文档完成多机械臂场景下的话题名修改并重启服务，使得：

   - 控制指令话题：`rt/arm_Command`（或你自定义的 `rt/arm_Command_1` 等）；
   - 关节角度反馈话题：`current_servo_angle`（或 `current_servo_angle_1` 等）。

---

#### 2）构建官方 C++ SDK 示例（在开发机上）

在开发机上部署官方 D1 SDK（`D1_SDK.zip`），并确保安装了 C++ 版 `unitree_sdk2`，可以正常编译官方示例：

```bash
cd /path/to/D1_SDK

mkdir -p build
cd build
cmake ..
make -j4
```

构建成功后，可以运行官方示例确认环境正确，例如：

```bash
./arm_zero_control              # 机械臂归零示例
./multiple_joint_angle_control  # 多关节角度控制示例
./joint_enable_control          # 关节使能 / 卸力示例
./get_arm_joint_angle           # 关节角度反馈示例
```

如果上述程序能正确运行且机械臂有对应动作，说明 C++ SDK 与 DDS 网络工作正常。

---

#### 3）构建 C++ D1 bridge（`d1_bridge`）

在 `D1_SDK` 工程中新增 `d1_bridge.cpp`，用于在 DDS 与 Python 之间转发数据：

- 通过 DDS 订阅 `current_servo_angle`（或 `current_servo_angle_1` 等）获取关节角反馈；
- 通过 DDS 发布 `rt/arm_Command`（或带后缀的控制话题）发送控制命令；
- 在本机开启一个 TCP 服务器（例如监听 `127.0.0.1:5555`），与 Python 侧 `D1_ArmController` 通信；
- 支持简单文本协议，例如：
  - 发送控制命令：  
    `CMD {"seq":4,"address":1,"funcode":2,"data":{...}}
`
  - 读取当前关节角：  
    `GET_Q
` → 返回 `Q s0 s1 ... s6
`（单位：度，第 7 维为夹爪行程 mm）。

在 `D1_SDK/CMakeLists.txt` 中增加可执行程序（示例）：

```cmake
add_executable(d1_bridge
    src/d1_bridge.cpp
    # 如有需要，在此添加其他源文件
)

target_link_libraries(d1_bridge
    unitree_sdk2
    # 以及官方 SDK 要求的 DDS / pthread 等库
)
```

重新编译：

```bash
cd /path/to/D1_SDK/build
cmake ..
make -j4
# 完成后应生成 ./d1_bridge 可执行文件
```

---

#### 4）启动 C++ bridge 服务

在一个单独终端中（后续所有 Python 测试都依赖该进程，需保持运行）：

```bash
cd /path/to/D1_SDK/build
./d1_bridge
```

预期输出类似：

```text
[d1_bridge] Init DDS...
[d1_bridge] DDS spin thread started.
[d1_bridge] Listening on 127.0.0.1:5555 ...
```

---

#### 5）配置 Python 环境与 D1 适配

在 `unifolm-world-model-action` 顶层配置 `unitree_deploy` 环境（如尚未完成）：

```bash
conda create -n unitree_deploy python=3.10
conda activate unitree_deploy

pip install -e .
# （可选）安装 LeRobot 相关依赖
pip install -e ".[lerobot]"
```

在本仓库中完成 D1 相关适配代码集成（关键位置示例）：

- `unitree_deploy/robot_devices/arm/configs.py`  
  - 增加 `D1ArmConfig`。
- `unitree_deploy/robot_devices/arm/d1_arm.py`  
  - 实现 `D1_ArmController`：通过 socket 连接 `d1_bridge`，封装上电、使能 / 卸力、多关节控制、关节角读取等接口。
- `unitree_deploy/robot_devices/arm/utils.py`  
  - 在 `make_arm_motors_buses_from_configs` / `make_arm_motors_bus` 中注册 `"d1"` 分支。
- `unitree_deploy/robot/robot_configs.py`  
  - 增加：
    - `d1_motors` 关节映射（如 `"joint0"~"joint5"` + `"gripper"` → 索引 0~6）；
    - `D1_OnlyArm_RobotConfig`（仅机械臂，无摄像头）；
    - `D1_Realsense_RobotConfig`（机械臂 + RealSense 摄像头）。
- `unitree_deploy/robot/robot_utils.py`  
  - 在 `make_robot_config` 中注册：
    - `"d1_only_arm"`
    - `"d1_realsense"`

---

### 2.4.2 🧪 D1 测试（无摄像头）

> 前提：`d1_bridge` 已在终端中启动并保持运行：  
> `cd /path/to/D1_SDK/build && ./d1_bridge`

1. 在另一个终端中激活环境并进入 `unitree_deploy`：

   ```bash
   conda activate unitree_deploy
   cd /path/to/unifolm-world-model-action/unitree_deploy
   ```

2. 运行 D1 机械臂测试脚本（`d1_only_arm`）：

   ```bash
   python test/arm/d1/test_d1_arm.py
   ```

   该脚本会：

   - 通过 `make_robot("d1_only_arm")` 创建只包含 D1 机械臂的 `UnitreeRobot`；
   - 调用 `robot.connect()`：
     - `D1_ArmController` 连接本地 `d1_bridge`（例如 `127.0.0.1:5555`）；
     - 上电（funcode=6, `power=1`）；
     - 所有关节使能（funcode=5, `mode=80000`）；
   - 读取当前关节角（弧度制）并打印；
   - 在当前姿态基础上，让第 0 号关节增加约 `+10°`，同时调整夹爪开合；
   - 等待约 2 秒，再次读取关节角并打印；
   - 调用 `robot.disconnect()`：
     - 所有关节卸力（funcode=5, `mode=0`）；
     - 关节断电（funcode=6, `power=0`）；
     - 关闭 socket 连接。

3. 预期现象

   - 运行 `d1_bridge` 的终端中看到客户端连接与断开日志，例如：

     ```text
     [d1_bridge] Client connected.
     ...
     [d1_bridge] Client disconnected.
     ```

   - 运行 `test_d1_arm.py` 的终端中看到类似输出：

     ```text
     [INFO] [test_d1_arm] current q (rad): [...]
     [INFO] [test_d1_arm] Sending +10deg command on joint 0 ...
     [INFO] [test_d1_arm] new q (rad): [...]
     [SUCCESS] [test_d1_arm] DONE, 请观察 D1 底座关节是否有小幅旋转。
     ```

   - D1 机械臂本体：
     - 底座关节（J0）有约 `+10°` 的小幅旋转；
     - 夹爪开合也会有一定变化（取决于测试脚本中的设定）。

---

### 2.4.3 📷 D1 + RealSense 环境测试（可选）

> 如当前尚未接入实体摄像头，可暂时跳过本小节。

1. 确认 RealSense 摄像头可用

   - 安装 Intel RealSense SDK（`librealsense`），连接摄像头后执行：

     ```bash
     rs-enumerate-devices
     ```

   - 记录摄像头序列号，将其写入 `robot_configs.py` 中 `D1_Realsense_RobotConfig` 的 `cameras` 字段，例如：

     ```python
     "cam_high": IntelRealSenseCameraConfig(
         serial_number="<你的摄像头序列号>",
         fps=30,
         width=640,
         height=480,
     )
     ```

2. 摄像头单独测试

   ```bash
   conda activate unitree_deploy
   cd /path/to/unifolm-world-model-action/unitree_deploy

   python test/camera/test_d1_realsense_camera.py
   ```

   预期：终端打印图像尺寸、帧率或每帧 `rgb/depth` 的 shape。

3. D1 + 摄像头联合环境测试（`d1_realsense`）

   ```bash
   # 确保 d1_bridge 已在终端中运行
   cd /path/to/unifolm-world-model-action/unitree_deploy
   conda activate unitree_deploy

   python test/arm/d1/test_d1_env.py
   ```

   该脚本会实例化 `robot_type="d1_realsense"` 的 `UnitreeRobot`，在若干步内：

   - 读取 observation（关节状态 + 相机图像）；
   - 对关节施加小幅度控制指令（例如全部关节小角度扰动）；
   - 打印观测信息维度与部分统计信息。

---

### 2.4.4 🧠 D1 数据集回放 / 推理测试（可选）

当 D1 硬件测试与环境测试均通过后，可以使用本仓库提供的统一回放脚本，在真实 D1 上进行数据集回放或策略推理。

```bash
conda activate unitree_deploy
cd /path/to/unifolm-world-model-action

python test/test_replay.py   --repo-id <你的_d1_dataset_repo_id_或本地数据路径>   --robot_type d1_realsense
```

脚本逻辑（示意）：

- 加载数据集中的关节轨迹 / 动作命令；
- 在每一步将数据中的 action 发送给真实 D1（通过 `D1_ArmController` → `d1_bridge` → DDS → D1）；
- 同时记录真实执行过程中的关节状态 / 图像，用于验证回放效果与真实表现差异。



# 3.🧠 推理与部署
1. [根据您的配置修改相应参数](./unitree_deploy/robot/robot_configs.py)
2. 返回 [决策模式下的推理与部署](https://github.com/unitreerobotics/unifolm-world-model-action/blob/main/README.md) 中的 **客户端设置步骤 2**。

# 4.🏗️ 代码结构

[如果您想添加自己的机器人设备，可以根据此文档进行构建](./docs/GettingStarted.md)

# 5. 🤔 故障排除

如需帮助，请联系项目维护人员或参考相应的 GitHub 仓库文档。📖

# 6. 🙏 致谢

此代码基于以下开源代码库构建。请访问相关 URL 查看相应的 LICENSES（如果您觉得这些项目有价值，请为它们点亮星星）：

1. https://github.com/huggingface/lerobot
2. https://github.com/unitreerobotics/unitree_sdk2_python
