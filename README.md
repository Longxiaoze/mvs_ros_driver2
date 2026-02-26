# mvs_ros_driver2
publish image using Hikvision MV-CU013-A0UC for fast-livo2

## 更新过程（2026-02-26）
1. 新建并切换分支：
   `git checkout -b g1`
2. 修复启动崩溃 `basic_string::_M_construct null not valid`：
   - 增加配置参数校验，避免 `argv[1]` 为空时构造 `std::string`。
   - 去掉对 `getlogin()` 的依赖，改为 `HOME`/`getpwuid(getuid())` 获取 home 目录。
   - `timeshare` 共享内存打开或映射失败时回退到 ROS 时间戳。
   - 仅在映射成功时执行 `munmap`。
3. 重新编译：
   `source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_EXE_LINKER_FLAGS="-lusb-1.0"`
4. 验证：
   - 无配置参数时节点会报错退出，不再崩溃。
   - 带配置文件可正常加载 YAML，不再触发 `basic_string` 异常。
