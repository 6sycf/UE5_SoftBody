#include "SoftBodyComputeShader.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "ShaderParameterStruct.h"

// =========================================================
// Shader 注册
// =========================================================

IMPLEMENT_GLOBAL_SHADER(FIntegrateCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "IntegrateCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FSolveAndApplyCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "SolveAndApplyCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FUpdateLambdasCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "UpdateLambdasCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FCollideStickCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "CollideStickCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FUpdateVelocityCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "UpdateVelocityCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FClearLambdasCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "ClearLambdasCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FExportPositionsCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "ExportPositionsCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FClearNormalsCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "ClearNormalsCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FComputeWeightedNormalsCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "ComputeWeightedNormalsCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FComputeVolumeCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "ComputeVolumeCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FApplyVolumePressureCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "ApplyVolumePressureCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FClearVolumeCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "ClearVolumeCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FCollideGroundCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "CollideGroundCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FExportNormalsCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "ExportNormalsCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FExportVolumeCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "ExportVolumeCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FUpdateHighResTargetCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "UpdateHighResTargetCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FSolveDihedralCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "SolveDihedralCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FUpdateDihedralLambdasCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "UpdateDihedralLambdasCS", SF_Compute); 
IMPLEMENT_GLOBAL_SHADER(FExportToTextureCS, "/SoftBodyGPU/Private/SoftBodyCompute.usf", "ExportToTextureCS", SF_Compute);