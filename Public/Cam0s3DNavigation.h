// Cam0's 3D Navigation 2023

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FCam0s3DNavigationModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
