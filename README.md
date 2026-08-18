# SoftBody_XPBDGPULOD8

基于 **Unreal Engine 5.6** 的软体形变模拟项目。物理求解采用 **XPBD（Extended Position Based Dynamics）**，支持 **CPU / GPU 双路径**、**高/低模代理 LOD**，GPU 路径支持 **全局距离场（GDF）静态网格体碰撞**，并集成 **Geomagic OpenHaptics** 触觉力反馈设备的双向耦合交互（当前默认禁用，见下文）。

## 功能特性

- **XPBD 求解器**：距离约束、弯曲约束、二面角弯曲约束、内部支撑约束（ISPC），以及基于散度定理的体积保持。
- **CPU / GPU 双路径**：可一键切换；GPU 路径基于 UE 的 RDG（Render Graph）+ Compute Shader，含异步双缓冲回读。
- **代理 LOD（Proxy）**：用网格简化生成低模 `ProxyMesh` 做物理计算，高模通过重心坐标 + 法线偏移（`FScaffoldBinding`）绑定到低模，低模驱动高模。
- **VAT 渲染（纯 GPU 变形）**：把高模顶点位移/法线导出到两张 RenderTarget 纹理，材质采样驱动变形，极大提升帧率。
- **全局距离场碰撞（GPU）**：`CollideGDFCS` 采样 UE 全局距离场（Page Atlas 稀疏 clipmap），粒子按 SDF 距离 + 梯度推出表面，含速度修正（恢复系数 0 + 摩擦）。
- **触觉交互（可选，默认禁用）**：集成 Geomagic OpenHaptics，1000Hz 线程读取硬件，双向耦合（软体推笔、笔反馈力）。由 `SoftBody.Build.cs` 里的 `bUseOpenHaptics` 开关控制。

## 目录结构

```
Source/SoftBody/                 # 游戏模块
  ├─ Public/MySoftBodyMeshComponent.h   # 软体组件（核心）
  ├─ Public/OpenHapticsComponent.h      # 触觉设备组件
  └─ Private/*.cpp
Plugins/SoftBodyGPU/             # GPU 计算插件（独立 Runtime 模块）
  ├─ Source/SoftBodyGPU/         # Compute Shader 定义与调度
  └─ Shaders/Private/SoftBodyCompute.usf  # HLSL 实现
ThirdParty/OpenHaptics/          # Geomagic OpenHaptics SDK（HD/HL）
```

## 环境要求

- **Unreal Engine 5.6**（源码版或安装版均可）
- **Visual Studio 2022**（C++ 编译）
- **Geomagic OpenHaptics** 触觉设备（可选，仅触觉交互需要；当前默认禁用）
- 已启用插件：`ModelingToolsEditorMode`（仅编辑器）、`GeometryScripting`、`SoftBodyGPU`

> OpenHaptics 的头文件与库已在 `ThirdParty/OpenHaptics` 内。触觉集成默认关闭（`SoftBody.Build.cs` 中 `bUseOpenHaptics = false`），启用时需额外把 `PhantomIoLib42.dll`（SDK bin 目录）放进 `ThirdParty/OpenHaptics/lib/x64/Release/`。

## 快速开始

1. 用 UE 5.6 打开 `SoftBody_XPBDGPULOD8.uproject`（首次会编译 C++ 模块）。
2. 在关卡中放置一个 Actor，添加 **`UMySoftBodyMeshComponent`**。
3. 在组件上指定 **Source Static Mesh**（软体的初始网格）。
4. 点击 **Build Cloth State**（或游戏运行时 `OnRegister` 自动构建）。
5. 勾选 **Simulate** 开始模拟，调整子步长、刚度等参数。

**触觉交互（可选，当前默认禁用）**：需先把 `SoftBody.Build.cs` 中 `bUseOpenHaptics` 改为 `true` 并重新构建；场景中放置带 **`UOpenHapticsComponent`** 的 Actor，软体组件会在 `BeginPlay` 自动查找并绑定；勾选软体组件的 **Enable Haptic Interaction** 即可用触觉笔戳软体。

**全局距离场碰撞（GPU）**：项目设置开启 **Generate Mesh Distance Fields**（`r.GenerateMeshDistanceFields=1`）并为静态网格体构建距离场，组件勾选 **`bUseDistanceFieldCollision`**（需 `bUseGPU`）。`Config/DefaultEngine.ini` 已配好 `r.AOGlobalDistanceField.DetailedNecessityCheck=0` 与 `r.DistanceFields.SupportEvenIfHardwareRayTracingSupported=1`。

## 关键参数（组件属性面板）

| 分类 | 参数 | 说明 |
|------|------|------|
| Soft Body Setup | SourceStaticMesh | 软体初始网格 |
| Soft Body Simulation | SubstepTime | 物理子步长（秒），值越小越稳定、开销越大 |
| | ConstraintIterations | XPBD 约束迭代次数 |
| | bUseGPU | 是否走 GPU 计算路径 |
| | bWorldCollision | 是否与场景碰撞体碰撞（CPU 路径） |
| | bUseDistanceFieldCollision | GPU 路径是否启用全局距离场碰撞（静态网格体） |
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
  → CollideStickCS（触觉笔）/ CollideGDFCS（全局距离场碰撞）
  → UpdateVelocityCS
  → 法线 / 体积 / 导出（ExportToTextureCS 等）
```

数据结构在 C++（`FGPUParticle` 等）与 HLSL（`FSoftBodyParticle` 等）之间严格对齐。

## 碰撞

- **全局距离场**：GPU 路径 `CollideGDFCS`（采样 UE 全局距离场，静态网格体碰撞）。
- **触觉笔**：`CollideStickCS` / `CollideWithHapticStylus`（胶囊体 SDF）。
- **场景静态网格体（CPU 路径）**：`ClothCollisionWorld()` 使用逐粒子球形扫掠（`SweepSingleByObjectType`）。

> 已知限制：GPU 路径的全局距离场碰撞只覆盖静态网格体，动态物体不在 GDF 中；分辨率随离相机距离下降。CPU 路径用 `ClothCollisionWorld()` 的扫掠检测，支持动态物体。

## 版本管理

项目已接入 Git + GitHub，详见根目录 [`GIT_GUIDE.md`](GIT_GUIDE.md)。

- 推送 / 拉取前请先开启 VPN（git 已配置代理 `127.0.0.1:7890`）。
- `Binaries/`、`Intermediate/`、`Saved/`、`DerivedDataCache/` 等由 `.gitignore` 排除。
