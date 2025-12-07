# Cartographer ROS 修复说明

本文档记录了修复 Cartographer ROS 编译和运行错误的所有修改。

```
git clone https://github.com/ros2/cartographer.git -b ros2
git clone https://github.com/ros2/cartographer_ros.git -b ros2

```

## 问题1: gflags 重复定义错误

### 错误现象
```
ERROR: flag 'collect_metrics' was defined more than once (in files 
'/home/only/workspace/xxx/smartcar/cartographer_ws/src/cartographer_ros/cartographer_ros/src/offline_node.cpp' 
and '/home/only/workspace/xxx/smartcar/cartographer_ws/src/cartographer_ros/cartographer_ros/src/node_main.cpp').
```

### 问题原因
`node_main.cpp` 和 `offline_node.cpp` 两个文件中都使用 `DEFINE_*` 宏定义了相同的 gflags 标志，导致重复定义。这些标志包括：
- `collect_metrics`
- `configuration_directory`
- `load_state_filename`
- `load_frozen_state`
- `save_state_filename`

### 解决方案
创建独立的 `flags.cpp` 文件统一定义共享标志，其他文件使用 `DECLARE_*` 声明。

---

## 修改详情

### 1. 创建新文件：`src/cartographer_ros/cartographer_ros/src/flags.cpp`

**文件路径**: `src/cartographer_ros/cartographer_ros/src/flags.cpp`

**修改内容**: 新建文件，定义所有共享的 gflags 标志

```cpp
#include "gflags/gflags.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_string(save_state_filename, "",
              "Explicit name of the file to which the serialized state will be "
              "written before shutdown. If left empty, the filename will be "
              "inferred from the first bagfile's name as: "
              "<bag_filenames[0]>.pbstream");
```

**修改原因**: 在库中统一定义共享标志，避免多个源文件重复定义。

---

### 2. 修改文件：`src/cartographer_ros/cartographer_ros/src/offline_node.cpp`

**文件路径**: `src/cartographer_ros/cartographer_ros/src/offline_node.cpp`

**修改位置**: 第 42-73 行的 gflags 定义部分

**修改前**:
```cpp
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
// ... 其他 DEFINE 语句
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_string(save_state_filename, "",
              "Explicit name of the file to which the serialized state will be "
              "written before shutdown. If left empty, the filename will be "
              "inferred from the first bagfile's name as: "
              "<bag_filenames[0]>.pbstream");
```

**修改后**:
```cpp
DECLARE_bool(collect_metrics);
DECLARE_string(configuration_directory);
// ... 保留 offline_node 特有的 DEFINE 语句
DECLARE_string(load_state_filename);
DECLARE_bool(load_frozen_state);
DECLARE_string(save_state_filename);
```

**修改原因**: 将重复定义改为声明，引用 `flags.cpp` 中的定义。

---

### 3. 修改文件：`src/cartographer_ros/cartographer_ros/src/node_main.cpp`

**文件路径**: `src/cartographer_ros/cartographer_ros/src/node_main.cpp`

**修改位置**: 第 26-46 行的 gflags 定义部分

**修改前**:
```cpp
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");
```

**修改后**:
```cpp
DECLARE_bool(collect_metrics);
DECLARE_string(configuration_directory);
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DECLARE_string(load_state_filename);
DECLARE_bool(load_frozen_state);
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DECLARE_string(save_state_filename);
```

**修改原因**: 将共享标志改为声明，保留 `node_main.cpp` 特有的标志定义（如 `configuration_basename` 和 `start_trajectory_with_default_topics`）。

---

### 4. 修改文件：`src/cartographer_ros/cartographer_ros/CMakeLists.txt`

**文件路径**: `src/cartographer_ros/cartographer_ros/CMakeLists.txt`

#### 修改 4.1: 添加 flags.cpp 到库源文件

**修改位置**: 第 67-89 行，库源文件列表

**修改前**:
```cmake
add_library(${PROJECT_NAME} SHARED
  src/assets_writer.cpp
  src/map_builder_bridge.cpp
  src/msg_conversion.cpp
  # ... 其他源文件
)
```

**修改后**:
```cmake
add_library(${PROJECT_NAME} SHARED
  src/assets_writer.cpp
  src/flags.cpp
  src/map_builder_bridge.cpp
  src/msg_conversion.cpp
  # ... 其他源文件
)
```

**修改原因**: 将新创建的 `flags.cpp` 添加到库的编译列表中。

#### 修改 4.2: 添加 gflags 到库链接依赖

**修改位置**: 第 120-128 行，库的私有链接依赖

**修改前**:
```cmake
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${CAIRO_LIBRARIES}
  pcl_common
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  ${tf2_msgs_TARGETS}
  urdf::urdf
)
```

**修改后**:
```cmake
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${CAIRO_LIBRARIES}
  gflags
  pcl_common
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  ${tf2_msgs_TARGETS}
  urdf::urdf
)
```

**修改原因**: 库中使用了 gflags，需要链接 gflags 库以解决链接时的 undefined reference 错误。

---

## 问题2: Lua 配置文件语法错误

### 错误现象
```
F1206 22:15:42.697067 2255754 lua_parameter_dictionary.cc:83] 
Check failed: status == 0 (3 vs. 0) 
[string "-- Copyright 2016 The Cartographer Authors..."]:23: 
unfinished string near '"laser,'
```

### 问题原因
`revo_lds.lua` 配置文件第 23 行字符串缺少闭合引号。

---

### 5. 修改文件：`src/cartographer_ros/cartographer_ros/configuration_files/revo_lds.lua`

**文件路径**: `src/cartographer_ros/cartographer_ros/configuration_files/revo_lds.lua`

**修改位置**: 第 23 行

**修改前**:
```lua
published_frame = "laser,
```

**修改后**:
```lua
published_frame = "laser",
```

**修改原因**: 修复 Lua 语法错误，字符串必须有闭合引号。

---

## 编译和运行

修改完成后，执行以下命令重新编译：

```bash
cd ~/workspace/wang/smartcar/cartographer_ws
colcon build --packages-select cartographer_ros --allow-overriding cartographer_ros
```

运行测试：

```bash
ros2 launch cartographer_ros demo_revo_lds.launch.py
```

---

## 总结

本次修复解决了两个主要问题：

1. **gflags 重复定义问题**: 通过创建独立的 `flags.cpp` 文件统一管理共享标志，避免多个源文件重复定义导致的编译错误。

2. **Lua 配置文件语法错误**: 修复了配置文件中未闭合的字符串引号，解决了运行时的 Lua 解析错误。

这些修改确保了 Cartographer ROS 能够正确编译和运行。
