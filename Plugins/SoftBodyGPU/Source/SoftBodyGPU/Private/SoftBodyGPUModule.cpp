#include "SoftBodyGPUModule.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

void FSoftBodyGPUModule::StartupModule()
{
	FString PluginShaderDir = FPaths::Combine(
		IPluginManager::Get().FindPlugin(TEXT("SoftBodyGPU"))->GetBaseDir(), 
		TEXT("Shaders"));
	AddShaderSourceDirectoryMapping(TEXT("/SoftBodyGPU"), PluginShaderDir);
}

void FSoftBodyGPUModule::ShutdownModule()
{
}

IMPLEMENT_MODULE(FSoftBodyGPUModule, SoftBodyGPU)
