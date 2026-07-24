## STM32 HAL xmake 模板

一个基于 `xmake` 的便于跨平台使用的 STM32 HAL 工程模板。
- 使用 [xmake](https://xmake.io/) 构建，构建配置简单
- 使用 gcc arm（`arm-none-eabi`）开源工具链
- 不依赖于任何特定的IDE，方便跨平台使用
- 使用 CubeMX 生成 HAL 代码，便于迁移
- 便于使用C++编写应用代码


## 环境要求

- STM32CubeMX
- [xmake](https://xmake.io/)（v2.8.5以上）
- [arm-none-eabi-gcc](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

## 使用
#### 1. 克隆本仓库：

```bash
git clone https://github.com/XylaAlyx/stm32_xmake_template.git
```

#### 2. 生成代码：
使用STM32CubeMX生成代码到 `bsp/HAL` 目录下，确保Project Manager 处的 Toolchain/IDE 为 `Makefile`。
#### 3. 配置项目：

在项目根目录执行：
```
xmake f --sdk="<arm-gnu-toolchain-path>"
```

请将 `<arm-gnu-toolchain-path>` 更改为安装时输入的路径。不需要带 `bin` 目录。



#### 4. 构建项目：
```
xmake
```
最终固件输出位于 `build/cross/arm/release/application.elf`。

可通过执行`xmake f -m debug`或`xmake f -m release`来切换构建模式。


在 CubeMX 中重新生成代码后，可以继续复用同一套 `xmake` 构建流程。

## Flash & Debug
下载可使用 ST-Link Utility、OpenOCD、JLink、CubeProgrammer 等工具。

调试可使用 OpenOCD+GDB、Ozone(使用Jlink或DAPLink推荐)等。

可在ST官网下载[芯片对应的svd文件](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html#cad-resources)，调试时加载该文件即可获得寄存器视图。

## 说明

- 当前cubemx工程中配置目标为 STM32F407（`cortex-m4`、`fpv4-sp-d16`、hard-float ABI）。
- 如果在 CubeMX 中修改了目标芯片，同步修改 `xmake.lua` 中的目标链接脚本即可。

## 致谢

工程结构参考于[rmcs-slave](https://github.com/Alliance-Algorithm/rmcs_slave)。

## 许可证
GPL-3.0 License
