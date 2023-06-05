// Copyright Epic Games, Inc. All Rights Reserved.

#include "Cam0s3DNavigation.h"

#define LOCTEXT_NAMESPACE "FCam0s3DNavigationModule"

void FCam0s3DNavigationModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
}

void FCam0s3DNavigationModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FCam0s3DNavigationModule, Cam0s3DNavigation)