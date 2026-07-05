# 电赛准备 — 项目进度

> 最后更新：2026-05-11
> Claude Code 新会话会自动读取此文件，了解断点位置。

## 当前状态
- **阶段**：地基搭建 → 工具链闭环
- **等待用户**：跑通工具链的硬件信息确认（STM32F103C8T6 板型 + LED引脚 + 串口方式）

## 已完成

### Git + Gitee
- [x] SSH 密钥生成（ed25519）
- [x] Gitee 账号：phz-electronic-design-competition
- [x] 仓库：electric-sai-stm32-measurement（私有）
- [x] 首次 push 成功

### 地基搭建
- [x] 记忆系统 10 个文件（规则/工具链/芯片/电赛攻略）
- [x] 协作规则完全校准
- [x] 桌面指南文件：`C:\Users\48747\Desktop\电赛嵌入式开发指南_Claude_AI协作.md`

### 工具链验证
- [x] Keil v5 (UV4.exe + ARMCLANG)
- [x] STM32CubeMX (D:\STM32CubeMX\)
- [x] ST-LINK_CLI (PATH 可用)
- [x] Git 2.53.0 (zzzmomo-code / h78381445@gmail.com)
- [x] Python pt_env (3.11.15)
- [x] WSL Ubuntu 24.04 (stm32 用户, arm-gcc/openocd/gdb/cmake/make/ninja)
- [x] VS Code (code 命令)
- [x] 7-Zip v26.01
- [x] SSCOM v5.13.1 (D:\SSCOM\sscom5.13.1.exe)
- [x] 串口工具 2：波特律动助手 (keysking 推荐, serial.keysking.com)

### 芯片信息
- [x] STM32F103C8T6 规格存档（当前学习阶段）
- [x] STM32F407ZET6 规格存档（电赛候选主控）
- [x] AD7606 规格存档（片外 ADC 模块）

### 电赛准备
- [x] 测量/信号采集方向攻略存档
- [x] AI 辅助电赛工作流存档
- [x] 7 个月学习路线存档

## 待办（按优先级）

### 高优先级
1. [ ] 用户注册 Gitee → 生成 SSH Key → 创建电赛仓库 → 初始化 Git → push
2. [ ] 跑通工具链闭环：CubeMX 生成 → Keil 编译 → ST-LINK 烧录 → 串口打印
   - 阻塞：待用户确认 STM32F103C8T6 板型 + LED 引脚 + 串口方式
3. [ ] 第一个 STM32 项目目录下创建精简 CLAUDE.md

### 中优先级
4. [ ] 跑 keysking 教程例程，边做边解释
5. [ ] 准备 F407+AD7606 驱动代码模板（硬件到位后）

### 低优先级
6. [ ] 安装 Gitee 的 git 凭证配置
