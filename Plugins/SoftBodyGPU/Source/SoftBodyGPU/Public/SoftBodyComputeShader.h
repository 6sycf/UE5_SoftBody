#pragma once
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"
#include "GlobalDistanceFieldConstants.h"

// =========================================================
// GPU 数据结构 (与 Shader 匹配)
// =========================================================

struct FGPUParticle
{
    FVector3f Position;
    FVector3f PrevPosition;
    FVector3f Velocity;
    float InvMass;
    int32 State;
};

struct FGPUConstraint
{
    int32 P0;
    int32 P1;
    float RestLength;
    float Stiffness;
};

struct FGPUNeighborInfo
{
    uint32 StartIndex;
    uint32 Count;
};

struct FGPUDihedralConstraint
{
    int32 P1;
    int32 P2;
    int32 P3;
    int32 P4;
    float RestAngle;
    float Stiffness;
};

struct FGPULambda
{
    float Value;
};
// 三角形结构体
struct FGPUTriangle
{
    int32 A; int32 B; int32 C;
};
// =========================================================
// Shader 基类
// =========================================================

class SOFTBODYGPU_API FSoftBodyShaderBase : public FGlobalShader
{
public:
    FSoftBodyShaderBase() = default;
    FSoftBodyShaderBase(const ShaderMetaType::CompiledShaderInitializerType& Initializer)
        : FGlobalShader(Initializer) {}

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

// =========================================================
// Kernel 1: IntegrateCS
// =========================================================

class SOFTBODYGPU_API FIntegrateCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FIntegrateCS);
    SHADER_USE_PARAMETER_STRUCT(FIntegrateCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPUParticle>, RWParticles)
        SHADER_PARAMETER(float, SubstepTime)
        SHADER_PARAMETER(uint32, ParticleCount)
    END_SHADER_PARAMETER_STRUCT()
};

// =========================================================
// Kernel 2: SolveAndApplyCS
// =========================================================

class SOFTBODYGPU_API FSolveAndApplyCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FSolveAndApplyCS);
    SHADER_USE_PARAMETER_STRUCT(FSolveAndApplyCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPUParticle>, RWParticles)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUConstraint>, Constraints)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPULambda>, LambdaBuffer)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUNeighborInfo>, ParticleConInfos)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint32>, AdjacencyIndices)
        SHADER_PARAMETER(float, SubstepTime)
        SHADER_PARAMETER(uint32, ParticleCount)
        SHADER_PARAMETER(float, Omega)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVector3f>, TargetPositions)
        SHADER_PARAMETER(float, AttachmentStiffness)
        SHADER_PARAMETER(int32, UseTargetPosition)
    END_SHADER_PARAMETER_STRUCT()
};

// =========================================================
// Kernel 3: UpdateLambdasCS
// =========================================================

class SOFTBODYGPU_API FUpdateLambdasCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FUpdateLambdasCS);
    SHADER_USE_PARAMETER_STRUCT(FUpdateLambdasCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUParticle>, RWParticles)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUConstraint>, Constraints)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPULambda>, LambdaBuffer)
        SHADER_PARAMETER(float, SubstepTime)
        SHADER_PARAMETER(uint32, ConstraintCount)
    END_SHADER_PARAMETER_STRUCT()
};

// =========================================================
// Kernel 4: CollideStickCS
// =========================================================

class SOFTBODYGPU_API FCollideStickCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FCollideStickCS);
    SHADER_USE_PARAMETER_STRUCT(FCollideStickCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPUParticle>, RWParticles)
        SHADER_PARAMETER(uint32, ParticleCount)
        SHADER_PARAMETER(float, SubstepTime)
        SHADER_PARAMETER(FVector3f, StickStart)
        SHADER_PARAMETER(FVector3f, StickEnd)
        SHADER_PARAMETER(float, StickRadius)
        SHADER_PARAMETER(float, ParticleRadius)
        SHADER_PARAMETER(int32, HasStickInput)
        SHADER_PARAMETER(FVector3f, StickVelStart)
        SHADER_PARAMETER(FVector3f, StickVelEnd)
        SHADER_PARAMETER(float, StickFriction)
        SHADER_PARAMETER(float, StickRestitution)
    END_SHADER_PARAMETER_STRUCT()
};

// =========================================================
// Kernel 5: UpdateVelocityCS
// =========================================================

class SOFTBODYGPU_API FUpdateVelocityCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FUpdateVelocityCS);
    SHADER_USE_PARAMETER_STRUCT(FUpdateVelocityCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPUParticle>, RWParticles)
        SHADER_PARAMETER(float, SubstepTime)
        SHADER_PARAMETER(uint32, ParticleCount)
    END_SHADER_PARAMETER_STRUCT()
};

// =========================================================
// Kernel 6: ClearLambdasCS
// =========================================================

class SOFTBODYGPU_API FClearLambdasCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FClearLambdasCS);
    SHADER_USE_PARAMETER_STRUCT(FClearLambdasCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPULambda>, LambdaBuffer)
        SHADER_PARAMETER(uint32, ConstraintCount)
    END_SHADER_PARAMETER_STRUCT()
};

// =========================================================
// Kernel 7: ExportPositionsCS
// =========================================================

class SOFTBODYGPU_API FExportPositionsCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FExportPositionsCS);
    SHADER_USE_PARAMETER_STRUCT(FExportPositionsCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUParticle>, ReadParticles)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FVector3f>, OutputPositions)
        SHADER_PARAMETER(uint32, ParticleCount)
    END_SHADER_PARAMETER_STRUCT()
};

// =========================================================
// Kernel 8: ClearNormalsCS (法线初始化)
// =========================================================
class SOFTBODYGPU_API FClearNormalsCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FClearNormalsCS);
    SHADER_USE_PARAMETER_STRUCT(FClearNormalsCS, FSoftBodyShaderBase);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FVector3f>, WeightedNormals)
        SHADER_PARAMETER(uint32, ParticleCount)
    END_SHADER_PARAMETER_STRUCT()
};
// =========================================================
// Kernel 9: ComputeWeightedNormalsCS (计算权重法线)
// =========================================================
class SOFTBODYGPU_API FComputeWeightedNormalsCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FComputeWeightedNormalsCS);
    SHADER_USE_PARAMETER_STRUCT(FComputeWeightedNormalsCS, FSoftBodyShaderBase);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUParticle>, Particles)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUTriangle>, Triangles)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FVector3f>, WeightedNormals)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, VertexTriangleOffsets)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, VertexTriangleCounts)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, VertexTriangleIndices)
        SHADER_PARAMETER(uint32, ParticleCount)
    END_SHADER_PARAMETER_STRUCT()
};
// =========================================================
// Kernel 10: ComputeVolumeCS (计算体积)
// =========================================================
class SOFTBODYGPU_API FComputeVolumeCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FComputeVolumeCS);
    SHADER_USE_PARAMETER_STRUCT(FComputeVolumeCS, FSoftBodyShaderBase);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUParticle>, Particles)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVector3f>, WeightedNormalsRead)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<int>, VolumeInt)
        SHADER_PARAMETER(uint32, ParticleCount)
    END_SHADER_PARAMETER_STRUCT()
};
// =========================================================
// Kernel 11: ApplyVolumePressureCS (应用体积压力)
// =========================================================
class SOFTBODYGPU_API FApplyVolumePressureCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FApplyVolumePressureCS);
    SHADER_USE_PARAMETER_STRUCT(FApplyVolumePressureCS, FSoftBodyShaderBase);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPUParticle>, RWParticles)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVector3f>, WeightedNormalsRead)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<int>, VolumeIntRead)
        SHADER_PARAMETER(uint32, ParticleCount)
        SHADER_PARAMETER(float, RestVolume)
        SHADER_PARAMETER(float, VolumePressureCoeff)
        SHADER_PARAMETER(float, SubstepTime)
    END_SHADER_PARAMETER_STRUCT()
};
// =========================================================
// Kernel 12: ClearVolumeCS (体积初始化)
// =========================================================
class SOFTBODYGPU_API FClearVolumeCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FClearVolumeCS);
    SHADER_USE_PARAMETER_STRUCT(FClearVolumeCS, FSoftBodyShaderBase);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<int>, VolumeInt)
    END_SHADER_PARAMETER_STRUCT()
};

// =========================================================
// Kernel 14: ExportNormalsCS (导出归一化法线)
// =========================================================

class SOFTBODYGPU_API FExportNormalsCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FExportNormalsCS);
    SHADER_USE_PARAMETER_STRUCT(FExportNormalsCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVector3f>, WeightedNormalsRead)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FVector3f>, OutputNormals)
        SHADER_PARAMETER(uint32, ParticleCount)
    END_SHADER_PARAMETER_STRUCT()
};
// =========================================================
// Kernel 15: 导出体积
// =========================================================
class SOFTBODYGPU_API FExportVolumeCS : public FSoftBodyShaderBase
{
    DECLARE_GLOBAL_SHADER(FExportVolumeCS);
    SHADER_USE_PARAMETER_STRUCT(FExportVolumeCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<int>, VolumeIntRead)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<int>, OutputVolume)
    END_SHADER_PARAMETER_STRUCT()
};
// =========================================================
// Kernel 16: 高模重心坐标蒙皮
// =========================================================
class SOFTBODYGPU_API FUpdateHighResTargetCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FUpdateHighResTargetCS);
    SHADER_USE_PARAMETER_STRUCT(FUpdateHighResTargetCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPUParticle>, RWTargetPositions)  // 高模粒子 (写)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUParticle>, ReadProxyParticles) // 低模粒子 (读)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FScaffoldBinding>, ScaffoldBindings) // 绑定参数 (读)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUTriangle>, ProxyTriangles)       // 低模三角形 (读)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVector3f>, ReadProxyNormals)
        SHADER_PARAMETER(uint32, HighResCount) // 高模顶点总数
        SHADER_PARAMETER(float, SubstepTime)
        SHADER_PARAMETER(float, HighResStiffness)
        SHADER_PARAMETER(float, HighResDamping)
    END_SHADER_PARAMETER_STRUCT()
};
// =========================================================
// Kernel: 二面角约束解算器
// =========================================================
class SOFTBODYGPU_API FSolveDihedralCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FSolveDihedralCS);
    SHADER_USE_PARAMETER_STRUCT(FSolveDihedralCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPUParticle>, RWParticles)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUDihedralConstraint>, DihedralConstraints)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPULambda>, DihedralLambdaBuffer)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUNeighborInfo>, DihedralConInfos)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint32>, DihedralAdjacencyIndices)
        SHADER_PARAMETER(float, SubstepTime)
        SHADER_PARAMETER(uint32, ParticleCount)
        SHADER_PARAMETER(float, Omega)
    END_SHADER_PARAMETER_STRUCT()
};
// =========================================================
// Kernel: 更新二面角Lambdas
// =========================================================
class SOFTBODYGPU_API FUpdateDihedralLambdasCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FUpdateDihedralLambdasCS);
    SHADER_USE_PARAMETER_STRUCT(FUpdateDihedralLambdasCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUParticle>, RWParticles)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUDihedralConstraint>, DihedralConstraints)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPULambda>, DihedralLambdaBuffer)
        SHADER_PARAMETER(float, SubstepTime)
        SHADER_PARAMETER(uint32, ConstraintCount)
    END_SHADER_PARAMETER_STRUCT()
};

// =========================================================
// Kernel 17: 导出高模数据到纹理 (VAT)
// =========================================================
class SOFTBODYGPU_API FExportToTextureCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FExportToTextureCS);
    SHADER_USE_PARAMETER_STRUCT(FExportToTextureCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        // 读取已经算好的高模粒子和法线
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGPUParticle>, HighResParticles)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVector3f>, HighResNormals)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVector3f>, HighResRestPositions)
    
        // 写入到两张 2D 纹理中 (UAV)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float4>, OutPositionTex)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float4>, OutNormalTex)
        
        // 控制参数
        SHADER_PARAMETER(uint32, VertexCount)
        SHADER_PARAMETER(uint32, TexDimension)
    END_SHADER_PARAMETER_STRUCT()
};

// =========================================================
// Kernel 18: 全局距离场碰撞 (GDF)
// 采样 UE 全局距离场 (Page Atlas 稀疏 clipmap)，把穿透粒子推出表面。
// =========================================================
class SOFTBODYGPU_API FCollideGDFCS : public FSoftBodyShaderBase
{
public:
    DECLARE_GLOBAL_SHADER(FCollideGDFCS);
    SHADER_USE_PARAMETER_STRUCT(FCollideGDFCS, FSoftBodyShaderBase);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGPUParticle>, RWParticles)
        SHADER_PARAMETER_RDG_TEXTURE(Texture3D, GDFPageAtlasTexture)
        SHADER_PARAMETER_RDG_TEXTURE(Texture3D<uint>, GDFPageTableTexture)
        SHADER_PARAMETER_SAMPLER(SamplerState, GDFPageAtlasSampler)
        SHADER_PARAMETER_ARRAY(FVector4f, GDFTranslatedCenterAndExtent, [GlobalDistanceField::MaxClipmaps])
        SHADER_PARAMETER_ARRAY(FVector4f, GDFTranslatedWorldToUVAddAndMul, [GlobalDistanceField::MaxClipmaps])
        SHADER_PARAMETER(FVector3f, GDFInvPageAtlasSize)
        SHADER_PARAMETER(uint32, GDFClipmapSizeInPages)
        SHADER_PARAMETER(float, GDFVolumeTexelSize)
        SHADER_PARAMETER(uint32, NumGDFClipmaps)
        SHADER_PARAMETER(uint32, ParticleCount)
        SHADER_PARAMETER(float, ParticleRadius)
        SHADER_PARAMETER(FVector3f, PreViewTranslation)
        SHADER_PARAMETER(float, SubstepTime)
        SHADER_PARAMETER(float, GDFFriction)
    END_SHADER_PARAMETER_STRUCT()
};