using System.IO;
using UnrealBuildTool;

public class SoftBodyGPU : ModuleRules
{
	public SoftBodyGPU(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] { 
			"Core", 
			"CoreUObject", 
			"Engine", 
			"RenderCore", 
			"RHI", 
			"Projects" 
		});
		string ShaderDir = Path.Combine(ModuleDirectory, "../../Shaders");
		if (Directory.Exists(ShaderDir))
		{
			RuntimeDependencies.Add(Path.Combine(ShaderDir, "*"), StagedFileType.NonUFS);
		}
	}
}
