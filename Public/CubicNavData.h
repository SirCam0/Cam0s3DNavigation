// Cam0s3DNavigation 2023

#pragma once

#include "CubicNavMeshNode.h"
#include "CubicNavData.generated.h"

/**
 * 
 */

USTRUCT()
struct CAM0S3DNAVIGATION_API FCubicNavData
{
	GENERATED_BODY()

	FCubicNavData() {};

	FCubicNavData(TArray<FIntVector> AllKeys, TMap<FIntVector, FCubicNavMeshNode> Nodes, TArray<FIntVector> LandmarkKeys)
	{
		this->AllKeys = AllKeys;
		this->Nodes = Nodes;
		this->LandmarkKeys = LandmarkKeys;
	};

	UPROPERTY()
	TArray<FIntVector> AllKeys;

	UPROPERTY()
	TMap<FIntVector, FCubicNavMeshNode> Nodes;

	UPROPERTY()
	TArray<FIntVector> LandmarkKeys;
};
