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

    private string PluginsPath
    {
        get { return Path.GetFullPath(Path.Combine(ModulePath, "../../Plugins/")); }
    }

    public void IncludeHololensResearchPlugin()
    {
        // Create Eigen Path 
        string HololensResearchModePath = Path.Combine(PluginsPath, "HololensResearchMode");

        //Add Include path 
        PublicIncludePaths.AddRange(new string[] { Path.Combine(HololensResearchModePath, "Public") });
    }

    public RobofleetUnrealClient(ReadOnlyTargetRules Target) : base(Target)
    {
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] {"Core", "CoreUObject", "Engine", "WebSockets", "HoloLensResearchMode"});
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Private/robofleet_client_lib/include"));
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Private/robofleet_client_lib"));
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Private"));

        //IncludeHololensResearchPlugin();
    }
}
