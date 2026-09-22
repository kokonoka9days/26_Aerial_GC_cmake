# 26_Aerial_GC_cmake

基于 STM32F405 的航空云台固件工程，使用 STM32CubeMX 生成底层初始化代码，使用 CMake + Ninja 构建，并通过 ARM GCC 交叉工具链生成 STM32 可执行文件。工程同时包含 C 和 C++ 源文件，其中 `main.c` 按 C++ 编译，以便调用 `RM2023_Lib_V1.2` 中的 C++ 类库。

## 项目结构

```text
26_Aerial_GC_cmake/
├── Core/                         # STM32 应用层和 HAL 外设初始化代码
│   ├── Inc/                      # CubeMX 生成的应用头文件
│   └── Src/
│       ├── main.c                # 主程序入口（位于 Core）
│       ├── can.c                 # CAN 外设初始化与底层配置
│       ├── dma.c                 # DMA 配置
│       ├── gpio.c                # GPIO 配置
│       ├── spi.c                 # SPI 配置
│       ├── tim.c                 # 定时器配置
│       ├── usart.c               # USART 配置
│       ├── stm32f4xx_it.c        # 中断服务函数
│       ├── stm32f4xx_hal_msp.c   # HAL MSP 初始化与反初始化
│       ├── system_stm32f4xx.c    # STM32 系统时钟支持
│       ├── syscalls.c            # newlib 系统调用适配
│       └── sysmem.c              # 堆内存适配
├── Drivers/                      # STM32 芯片和 HAL/CMSIS 驱动
│   ├── CMSIS/                    # ARM CMSIS 内核及设备支持文件
│   └── STM32F4xx_HAL_Driver/    # STM32F4 HAL 驱动头文件、源文件和许可证
├── RM2023_Lib_V1.2/              # RM2023 控制算法与通信库
│   ├── RM_Lib.hpp                # C++ 控制器、姿态、传感器和电机类定义
│   ├── RM_Lib.cpp                # C++ 类库实现
│   ├── CP_System.h               # 裁判系统数据结构和 C 接口
│   ├── CP_System.c               # 裁判系统通信实现（本工程使用此版本）
│   ├── communication.h           # 上位机通信接口
│   ├── communication.c           # 上位机通信实现
│   ├── my_math.h                 # 滤波、卡尔曼等数学接口
│   └── my_math.c                 # 数学工具实现
├── cmake/
│   ├── gcc-arm-none-eabi.cmake   # ARM GCC 交叉编译工具链配置
│   ├── starm-clang.cmake         # 可选的 Clang 工具链配置
│   └── stm32cubemx/
│       └── CMakeLists.txt        # CubeMX 生成源文件和 HAL 目标配置
├── .vscode/                      # VS Code、Cortex-Debug 和 OpenOCD 配置
│   ├── c_cpp_properties.json     # C/C++ 代码索引配置
│   ├── daplink-stm32f4.cfg       # DAPLink/OpenOCD 配置
│   ├── jlink-stm32f4.cfg         # J-Link/OpenOCD 配置
│   ├── keybindings.json          # VS Code 快捷键配置
│   ├── launch.json               # Cortex-Debug 调试配置
│   ├── settings.json             # 工作区设置
│   └── tasks.json                # 构建和下载任务
├── CMakeLists.txt                # 顶层 CMake 构建配置
├── CMakePresets.json             # Debug/Release 配置预设
├── STM32F405XX_FLASH.ld          # STM32F405 Flash 链接脚本
├── startup_stm32f405xx.s         # Cortex-M4 启动文件和中断向量表
├── 26_Aerial_GC_cmake.ioc        # 当前 CubeMX 工程配置
├── NewFolder.ioc                 # 旧版/兼容用 CubeMX 配置文件
├── make.sh                       # Debug 配置并构建固件
├── downloads.sh                  # 使用 OpenOCD 下载 ELF 固件
├── .clangd                       # clangd 工程索引配置
├── .gitignore                    # Git 忽略规则
├── .mxproject                    # STM32CubeMX 工程元数据
└── README.md                     # 项目说明文档
```

`Core` 保存应用程序、`main.c` 和 STM32 外设初始化代码；`Drivers` 保存 CMSIS 以及 STM32F4 HAL 驱动。`build/` 是 CMake 生成的构建目录，不属于源代码目录。

## 开发环境

以下步骤以 Ubuntu/Debian 为例。交叉编译是指在 x86/x86_64 主机上编译出运行于 STM32F405 ARM Cortex-M4 的程序。

### 1. 安装工具链和构建工具

#### 安装交叉编译工具链
```bash
sudo apt update
sudo apt install -y cmake ninja-build gcc-arm-none-eabi \
    gdb-multiarch openocd build-essential
```

#### 安装ninja
```bash
sudo apt install ninja-build
```

#### 安装clangd
```bash
sudo apt install clangd
```

### 2. cubemx创建新工程
创建新工程后将.vscode文件夹、download.sh、make.sh、.clangd复制到新工程对应路径下


GDB 用于断点、单步、变量和内存检查；OpenOCD 通过 J-Link 或 DAPLink 的 SWD 接口完成下载和远程调试。

### 3. 编译

使用 CMake 预设构建 Debug 版本：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

也可以直接执行：

```bash
./make.sh
```

构建产物为 `build/Debug/26_Aerial_GC_cmake.elf` 和 `build/Debug/26_Aerial_GC_cmake.map`。Release 版本使用：

```bash
cmake --preset Release
cmake --build --preset Release
```

### 3. 下载和调试

连接 STM32F405 与 J-Link 后执行：

```bash
./downloads.sh
```

脚本默认使用 `target/stm32f4x.cfg`、SWD 和 1000 kHz。可用环境变量覆盖：

```bash
OPENOCD_SPEED=4000 ./downloads.sh
ELF_FILE=build/Release/26_Aerial_GC_cmake.elf ./downloads.sh
```

VS Code 可选安装 C/C++、clangd、Cortex-Debug 扩展。`.vscode/tasks.json` 提供构建/下载任务，`.vscode/launch.json` 提供 J-Link/OpenOCD 调试配置。

### 4. vscode配置

#### clangd配置
ctrl+shift+p搜索 “clangd:Restart language server” 

## C/C++ 混合编译

`RM_Lib.cpp` 使用 C++ 编译；`communication.c`、`my_math.c` 和 `main.c` 在 `CMakeLists.txt` 中显式设置为 C++ 编译，以便使用 `RM_Lib.hpp`。纯 HAL 驱动文件仍按 C 编译。C 与 C++ 共享的函数声明应使用 `extern "C"`，避免 C++ 名字修饰导致链接失败。

本工程使用 `RM2023_Lib_V1.2/CP_System.c` 这一份裁判系统实现，不应再同时加入其他同名实现，否则会产生接口不一致或重复定义问题。

## 常见检查

```bash
cmake --build build/Debug --verbose
rm -rf build/Debug
cmake --preset Debug
cmake --build --preset Debug
```

检查 ARM C/C++ 交叉编译器：

```bash
which arm-none-eabi-gcc
which arm-none-eabi-g++
arm-none-eabi-gcc --version
arm-none-eabi-g++ --version
```

GDB 和 OpenOCD 检查：

```bash
sudo ln -sf "$(command -v gdb-multiarch)" /usr/local/bin/arm-none-eabi-gdb
arm-none-eabi-gdb --version
openocd --version
```

如果出现 `arm-none-eabi-gcc: command not found`，检查工具链安装和 `PATH`；如果出现 `openocd: command not found`，安装 OpenOCD；如果出现结构体字段或符号错误，检查 `main.c` 与 `RM2023_Lib_V1.2/CP_System.h` 是否来自同一版本接口。
