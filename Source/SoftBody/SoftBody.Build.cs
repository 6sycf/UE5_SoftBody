// Copyright Epic Games, Inc. All Rights Reserved.

using System.IO;
using UnrealBuildTool;

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
		// [新增] OpenHaptics 第三方库集成
		// =========================================================
       
		// 1. 自动定位 ThirdParty 目录
		// ModuleDirectory 是 Build.cs 所在的目录 (Source/SoftBody/)
		// 我们需要往上退两级找到 SoftBody/ThirdParty/OpenHaptics
		string OpenHapticsPath = Path.Combine(ModuleDirectory, "../../ThirdParty/OpenHaptics");

		// 2. 添加头文件包含路径
		// 这样你在代码里就可以写 #include <HD/hd.h>
		string IncludePath = Path.Combine(OpenHapticsPath, "include");
		PublicIncludePaths.Add(IncludePath);

		// 3. 确定库文件路径 (根据编译配置选择 Debug 或 Release)
		string LibPath = Path.Combine(OpenHapticsPath, "lib/x64/Release");

		// 4. 链接静态库 (.lib)
		// 只需要 hd.lib 和 hl.lib 
		PublicAdditionalLibraries.Add(Path.Combine(LibPath, "hd.lib"));
		PublicAdditionalLibraries.Add(Path.Combine(LibPath, "hl.lib"));

		// 5. [必须] 运行时 DLL 拷贝
		// 这里的路径要指向你截图里的 lib/x64/Release/hd.dll
		// 注意：需要区分 Debug 和 Release 文件夹
       
		string DllSourcePath = Path.Combine(LibPath, "hd.dll"); 
		string HlDllSourcePath = Path.Combine(LibPath, "hl.dll");

		// 告诉 UE：这个文件是运行时依赖，打包时请带上它
		// 第二个参数是 "Staged Path"，也就是打包后的目标路径
		// "$(BinaryOutputDir)/hd.dll" 表示复制到 exe 旁边
		RuntimeDependencies.Add("$(BinaryOutputDir)/hd.dll", DllSourcePath);
		RuntimeDependencies.Add("$(BinaryOutputDir)/hl.dll", HlDllSourcePath);
	}
}
