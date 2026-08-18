// Copyright Epic Games, Inc. All Rights Reserved.

using System.IO;
using UnrealBuildTool;
using EpicGames.Core;
using Microsoft.Extensions.Logging;

public class SoftBody : ModuleRules
{
	public SoftBody(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"RHI",
			"GeometryCore", 
			"GeometryFramework",
			"GeometryScriptingCore",
			"DynamicMesh",
			"ModelingComponents",
			"SoftBodyGPU",
			"RenderCore",   
			"Renderer"      
		});

		PrivateDependencyModuleNames.AddRange(new string[] { "SoftBodyGPU", "SoftBodyGPU" });

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
		// =========================================================
		// OpenHaptics 触觉集成开关
		// 设为 false 时不链接 hd/hl，OpenHapticsComponent 编译为空桩。
		// 需要触觉设备时改回 true，并确保 ThirdParty/OpenHaptics 里有 hd.dll/hl.dll/PhantomIoLib42.dll。
		// =========================================================
		bool bUseOpenHaptics = false;
		PublicDefinitions.Add("SOFTBODY_USE_OPENHAPTICS=" + (bUseOpenHaptics ? "1" : "0"));

		if (bUseOpenHaptics)
		{
			// 1. 自动定位 ThirdParty 目录
			// ModuleDirectory 是 Build.cs 所在的目录 (Source/SoftBody/)
			// 我们需要往上退两级找到 SoftBody/ThirdParty/OpenHaptics
			string OpenHapticsPath = Path.Combine(ModuleDirectory, "../../ThirdParty/OpenHaptics");

			// 2. 添加头文件包含路径
			// 这样你在代码里就可以写 #include <HD/hd.h>
			string IncludePath = Path.Combine(OpenHapticsPath, "include");
			PublicIncludePaths.Add(IncludePath);

			// 3. 确定库文件路径
			string LibPath = Path.Combine(OpenHapticsPath, "lib/x64/Release");

			// 4. 链接静态库 (.lib)
			PublicAdditionalLibraries.Add(Path.Combine(LibPath, "hd.lib"));
			PublicAdditionalLibraries.Add(Path.Combine(LibPath, "hl.lib"));

			// 5. 运行时 DLL 拷贝
			string DllSourcePath = Path.Combine(LibPath, "hd.dll");
			string HlDllSourcePath = Path.Combine(LibPath, "hl.dll");
			RuntimeDependencies.Add("$(BinaryOutputDir)/hd.dll", DllSourcePath);
			RuntimeDependencies.Add("$(BinaryOutputDir)/hl.dll", HlDllSourcePath);

			// 6. hd.dll 依赖 PhantomIoLib42.dll (Phantom 设备 I/O 库)
			// 它随 OpenHaptics SDK 的 bin 目录分发，不在 lib 目录里。
			string PhantomIoDllSourcePath = Path.Combine(LibPath, "PhantomIoLib42.dll");
			if (File.Exists(PhantomIoDllSourcePath))
			{
				RuntimeDependencies.Add("$(BinaryOutputDir)/PhantomIoLib42.dll", PhantomIoDllSourcePath);
			}
			else
			{
				Logger.LogWarning("PhantomIoLib42.dll not found in {0}. The SoftBody module will fail to load at runtime. Copy PhantomIoLib42.dll from the OpenHaptics SDK into this folder.", LibPath);
			}
		}
	}
}
