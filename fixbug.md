# mvs_ros2_driver 启动崩溃修复记录

## 问题现象

执行以下命令启动相机驱动：

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
source ~/fast_livo2_handhold2_ws/install/setup.bash
ros2 launch mvs_ros2_driver single_camera.py
```

节点加载 YAML 配置文件后立即崩溃：

```text
terminate called after throwing an instance of 'std::logic_error'
what(): basic_string::_M_construct null not valid
```

由于 `single_camera.py` 设置了 `respawn=True`，节点崩溃后会被立即重新拉起，造成大量重复日志。

## 根因

`src/grab_trigger.cc` 原来使用 `getlogin()` 获取用户名：

```cpp
const char *user_name = getlogin();
std::string path_for_time_stamp =
    "/home/" + std::string(user_name) + "/timeshare";
```

当前运行环境中 `getlogin()` 返回空指针。把该空指针传入 `std::string` 后触发：

```text
basic_string::_M_construct null not valid
```

动态库检查显示 `libMvCameraControl.so` 可以正常加载，因此该异常不是 `LD_LIBRARY_PATH` 或 MVS 动态库缺失造成的。

## 已完成的修改

### 1. 修复用户目录空指针

改用 `HOME` 环境变量构造时间戳文件路径：

```cpp
const char *home_dir = std::getenv("HOME");
if (home_dir == nullptr || home_dir[0] == '\0') {
  ROS_ERROR("HOME environment variable is not set");
  return -1;
}

std::string path_for_time_stamp = std::string(home_dir) + "/timeshare";
```

当前时间戳文件为：

```text
/home/unitree/timeshare
```

### 2. 增加时间戳文件错误检查

对 `open()` 和 `mmap()` 的返回值进行检查，并在映射成功后关闭文件描述符：

```cpp
int fd = open(shared_file_name, O_RDWR);
if (fd == -1) {
  ROS_ERROR("Failed to open timestamp file %s: %s", shared_file_name,
            std::strerror(errno));
  return -1;
}

pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp),
                            PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
if (pointt == MAP_FAILED) {
  ROS_ERROR("Failed to map timestamp file %s: %s", shared_file_name,
            std::strerror(errno));
  close(fd);
  return -1;
}
close(fd);
```

### 3. 增加命令行参数检查

避免未传入 YAML 文件时访问无效的 `argv[1]`：

```cpp
if (argc < 2 || argv[1] == nullptr) {
  ROS_ERROR("Usage: mvs_camera_node <settings_file>");
  rclcpp::shutdown();
  return -1;
}
```

### 4. 增加 MVS SDK 生命周期管理

按照本机 MVS 4.6 SDK 示例，在枚举相机之前调用 `MV_CC_Initialize()`，退出时自动调用 `MV_CC_Finalize()`。

```cpp
class MvsSdkSession {
public:
  MvsSdkSession() : status_(MV_CC_Initialize()) {}

  ~MvsSdkSession() {
    if (status_ == MV_OK) {
      MV_CC_Finalize();
    }
  }

  int status() const { return status_; }

private:
  int status_;
};
```

### 5. 限制自动重启频率

在 `launch/single_camera.py` 中增加：

```python
respawn=True,
respawn_delay=2.0,
```

相机断开或节点退出后仍会自动恢复，但不会再以极高频率反复创建进程。

## 构建

使用以下命令重新构建：

```bash
cd ~/fast_livo2_handhold2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build \
  --packages-select mvs_ros2_driver \
  --symlink-install \
  --event-handlers console_direct+
```

构建结果：

```text
Summary: 1 package finished
```

## 测试结果

### 包测试

```bash
cd ~/fast_livo2_handhold2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select mvs_ros2_driver
colcon test-result --verbose
```

结果：

```text
Summary: 0 tests, 0 errors, 0 failures, 0 skipped
```

该包当前没有定义自动化测试。

### 启动参数测试

不传 YAML 文件时，节点现在会输出明确的用法提示，不再因为访问 `argv[1]` 而崩溃：

```text
Usage: mvs_camera_node <settings_file>
```

### 实际启动测试

执行：

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
source ~/fast_livo2_handhold2_ws/install/setup.bash
ros2 launch mvs_ros2_driver single_camera.py
```

原来的 `std::logic_error` 已经消失。驱动能够完成配置文件加载，并进入 MVS SDK 设备枚举阶段。

当前运行结果为：

```text
Find No Devices!
```

## 当前硬件阻塞

项目 README 指定的相机型号为海康 `MV-CU013-A0UC` USB3 相机。MVS USB udev 规则使用的厂商 ID 为 `2bdf`，但当前主机的 `lsusb` 和 USB 拓扑中没有任何 `2bdf` 设备。

这表示当前错误来自相机没有被主机 USB 总线枚举，常见原因包括：

- 相机未供电；
- USB3 数据线未连接或接触不良；
- 使用了仅供电、不传输数据的线缆；
- 相机没有连接到当前运行 ROS 2 的电脑；
- USB 接口或转接设备异常。

连接相机后先检查：

```bash
lsusb | grep -i 2bdf
```

能够看到设备后，再启动驱动：

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
source ~/fast_livo2_handhold2_ws/install/setup.bash
ros2 launch mvs_ros2_driver single_camera.py
```

然后检查图像话题：

```bash
ros2 topic list | grep camera
ros2 topic hz /front_camera/image_raw
ros2 topic echo /front_camera/image_raw --once
```

## 其他处理

测试过程中发现一个旧的 `ros2 launch mvs_ros2_driver single_camera.py` 进程已经卡死。该进程此前收到 Ctrl+C 后仍未退出，并且忽略后续 SIGINT。确认其进程树和日志后，已精确终止该旧进程；未影响其他 ROS 2 进程。

