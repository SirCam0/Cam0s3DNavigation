// Cam0s3DNavigation 2023

#pragma once

#include "CubicNavMeshNode.generated.h"

/**
 * 
 */

USTRUCT()
struct CAM0S3DNAVIGATION_API FCubicNavMeshNode
{
	GENERATED_BODY()

	UPROPERTY()
	FVector WorldLocation = { 0,0,0 };

	UPROPERTY()
	FIntVector NavMeshLocationKey = { 0,0,0 };

	UPROPERTY()
	TArray<FIntVector> NeighbourKeys;

	UPROPERTY()
	float TraversalCost = 1.f;

	UPROPERTY()
	bool bOverlapping;

	UPROPERTY()
	TMap<FIntVector, int> PrecompiledDistances;
};
