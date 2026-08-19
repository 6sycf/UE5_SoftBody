# SoftBody_XPBDGPU

基于 **Unreal Engine 5.6** 的软体形变模拟项目。物理求解采用 **XPBD（Extended Position Based Dynamics）**，基于 **GPU 计算**、**高/低模代理 LOD**，支持 **全局距离场（GDF）静态网格体碰撞**，并支持用 **3D 交互笔（Geomagic OpenHaptics 设备）** 在三维空间中戳动软体（当前默认禁用，见下文）。

> 说明：这里所谓的"触觉"本质只是**用笔在三维空间定位/交互**——把设备当作 3D 输入笔去戳软体、软体实时形变，并非力反馈（haptic feedback）体验。

## 功能特性

- **XPBD 求解器**：距离约束、弯曲约束、二面角弯曲约束、内部支撑约束（ISPC），以及基于散度定理的体积保持。
- **GPU 计算路径**：基于 UE 的 RDG（Render Graph）+ Compute Shader，含异步双缓冲回读。
- **代理 LOD（Proxy）**：用网格简化生成低模 `ProxyMesh` 做物理计算，高模通过重心坐标 + 法线偏移（`FScaffoldBinding`）绑定到低模，低模驱动高模。
- **VAT 渲染（纯 GPU 变形）**：把高模顶点位移/法线导出到两张 RenderTarget 纹理，材质采样驱动变形，极大提升帧率。
- **全局距离场碰撞（GPU）**：`CollideGDFCS` 采样 UE 全局距离场（Page Atlas 稀疏 clipmap），粒子按 SDF 距离 + 梯度推出表面，含速度修正（恢复系数 0 + 摩擦）。
- **3D 交互笔（可选，默认禁用）**：集成 Geomagic OpenHaptics 设备作为 3D 空间输入笔，1000Hz 线程读取笔尖位置，软体实时形变。由 `SoftBody.Build.cs` 里的 `bUseOpenHaptics` 开关控制。

## 目录结构

```
SoftBody_XPBDGPU.uproject         # 项目文件
Source/SoftBody/                  # 游戏模块
  ├─ SoftBody.Build.cs            # 模块构建（含 OpenHaptics 开关）
  ├─ Public/MySoftBodyMeshComponent.h   # 软体组件（核心）
  ├─ Public/OpenHapticsComponent.h      # 3D 交互笔组件
  └─ Private/*.cpp                      # 实现（GPU 模拟、GDF 碰撞、交互笔等）
Plugins/SoftBodyGPU/              # GPU 计算插件（独立 Runtime 模块）
  ├─ SoftBodyGPU.uplugin
  ├─ Source/SoftBodyGPU/
  │   ├─ Public/SoftBodyComputeShader.h  # Compute Shader 参数定义
  │   └─ Private/*.cpp                    # Shader 注册与模块
  └─ Shaders/Private/SoftBodyCompute.usf # HLSL 实现（约 18 个 Kernel）
Config/                           # UE 配置（距离场/渲染设置等）
Content/                          # 关卡与资产
ThirdParty/OpenHaptics/           # OpenHaptics SDK 放置处（用户自行下载，不纳入仓库）
```

## 环境要求

- **Unreal Engine 5.6**（源码版或安装版均可）
- **Visual Studio 2022**（C++ 编译）
- **Geomagic OpenHaptics 设备**（可选，仅 3D 交互笔需要；当前默认禁用）
- 已启用插件：`ModelingToolsEditorMode`（仅编辑器）、`GeometryScripting`、`SoftBodyGPU`

> OpenHaptics SDK 未包含在本仓库（3D Systems 版权物）。如需 3D 交互笔，需自行从 3D Systems 下载 OpenHaptics SDK，把 `include/` 和 `lib/` 放到 `ThirdParty/OpenHaptics/` 下，再按下方说明启用。笔交互默认关闭（`SoftBody.Build.cs` 中 `bUseOpenHaptics = false`）。

## 快速开始

1. 用 UE 5.6 打开 `SoftBody_XPBDGPU.uproject`（首次会编译 C++ 模块）。
2. 在关卡中放置一个 Actor，添加 **`UMySoftBodyMeshComponent`**。
3. 在组件上指定 **Source Static Mesh**（软体的初始网格）。
4. 点击 **Build Cloth State**（或游戏运行时 `OnRegister` 自动构建）。
5. 勾选 **Simulate** 开始模拟，调整子步长、刚度等参数。

**3D 交互笔（可选，默认禁用）**：先下载 OpenHaptics SDK 放到 `ThirdParty/OpenHaptics/`（结构：`include/HD/hd.h`、`lib/x64/Release/hd.lib` 等），把 `SoftBody.Build.cs` 中 `bUseOpenHaptics` 改为 `true` 并重新构建；场景中放置带 **`UOpenHapticsComponent`** 的 Actor，软体组件会在 `BeginPlay` 自动查找并绑定；勾选软体组件的 **Enable Haptic Interaction** 即可用笔在三维空间中戳动软体（软体实时形变，非力反馈）。

**全局距离场碰撞（GPU）**：项目设置开启 **Generate Mesh Distance Fields**（`r.GenerateMeshDistanceFields=1`）并为静态网格体构建距离场，组件勾选 **`bUseDistanceFieldCollision`**（需 `bUseGPU`）。`Config/DefaultEngine.ini` 已配好 `r.AOGlobalDistanceField.DetailedNecessityCheck=0` 与 `r.DistanceFields.SupportEvenIfHardwareRayTracingSupported=1`。

## 关键参数（组件属性面板）

| 分类 | 参数 | 说明 |
|------|------|------|
| Soft Body Setup | SourceStaticMesh | 软体初始网格 |
| Soft Body Simulation | SubstepTime | 物理子步长（秒），值越小越稳定、开销越大 |
| | ConstraintIterations | XPBD 约束迭代次数 |
| | bUseGPU | 是否走 GPU 计算路径 |
| | bUseDistanceFieldCollision | 是否启用全局距离场碰撞（静态网格体） |
| XPBD Settings | XPBD_StretchStiffness | 拉伸刚度 |
| | XPBD_BendingStiffness | 弯曲刚度 |
| | XPBD_DihedralStiffness | 二面角刚度 |
| HighRes Physics | HighResStiffness / Damping | 高模形状匹配的刚度与阻尼 |
| Soft Body Proxy | bUseProxy | 是否启用低模代理 |
| | TargetProxyTriangleCount | 代理网格目标面数 |
| Soft Body Rendering | bUpdateViaGPU | 纯 GPU 材质驱动高模（VAT） |

## GPU 计算管线

GPU 求解在 `Plugins/SoftBodyGPU` 中实现，`SoftBodyCompute.usf` 包含约 18 个 Kernel，主流程：

```
IntegrateCS（重力积分）
  → ClearLambdasCS
  → 迭代 { SolveAndApplyCS（距离约束 Jacobi）/ SolveDihedralCS（二面角）/ UpdateLambdasCS }
  → CollideStickCS（交互笔）/ CollideGDFCS（全局距离场碰撞）
  → UpdateVelocityCS
  → 法线 / 体积 / 导出（ExportToTextureCS 等）
```

数据结构在 C++（`FGPUParticle` 等）与 HLSL（`FSoftBodyParticle` 等）之间严格对齐。

## 碰撞

- **全局距离场**：`CollideGDFCS`（采样 UE 全局距离场，静态网格体碰撞）。
- **3D 交互笔**：`CollideStickCS` / `CollideWithHapticStylus`（胶囊体 SDF，笔戳软体形变）。

> 已知限制：全局距离场碰撞只覆盖静态网格体，动态物体不在 GDF 中；分辨率随离相机距离下降。

## 版本管理

项目已接入 Git + GitHub，详见根目录 [`GIT_GUIDE.md`](GIT_GUIDE.md)。

- 推送 / 拉取前请先开启 VPN（git 已配置代理 `127.0.0.1:7890`）。
- `Binaries/`、`Intermediate/`、`Saved/`、`DerivedDataCache/` 等由 `.gitignore` 排除。
