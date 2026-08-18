// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class SoftBodyTarget : TargetRules
{
	public SoftBodyTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.V5;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_6;
		ExtraModuleNames.Add("SoftBody");
		// 项目 Content 里有中文路径，UBT 的 git 工作集解析会崩溃 (ArgumentException)。
		// 禁用自适应 unity 构建，跳过 git status 解析。
		bUseAdaptiveUnityBuild = false;
	}
}
