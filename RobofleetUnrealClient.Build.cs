// Copyright Epic Games, Inc. All Rights Reserved.
using System.IO;
using UnrealBuildTool;

public class RobofleetUnrealClient : ModuleRules
{
    // convenience properties
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    private string ThirdPartyPath
    {
        get { return Path.GetFullPath(Path.Combine(ModulePath, "./")); }
    }

    public RobofleetUnrealClient(ReadOnlyTargetRules Target) : base(Target)
    {
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] {"Core", "CoreUObject", "Engine", "WebSockets", "ImageWrapper" });
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Private/robofleet_client_lib/include"));
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Private/robofleet_client_lib"));
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Private"));

    }
}
