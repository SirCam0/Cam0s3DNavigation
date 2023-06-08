// Cam0's 3D Navigation 2023

#pragma once

#include "CubicNavMeshNode.h"
#include "CubicNavData.generated.h"

/**
 * A Struct to store information representing a 3D nav mesh, to be stored.
 * @Author Cameron Greenway 2023
 */

USTRUCT()
struct CAM0S3DNAVIGATION_API FCubicNavData
{
	GENERATED_BODY()

	FCubicNavData() {};

	/*
	* Constructor to initialise with the following inputs.
	* @Param AllKeys - an array of all keys stored within the navmesh
	* @Param Nodes - a map containing all of the node instances
	* @Param LandmarkKeys - an array constaining the keys of all landmark nodes
	* @Author Cameron Greenway 2023
	*/
	FCubicNavData(TArray<FIntVector> AllKeys, TMap<FIntVector, FCubicNavMeshNode> Nodes, TArray<FIntVector> LandmarkKeys)
	{
		this->AllKeys = AllKeys;
		this->Nodes = Nodes;
		this->LandmarkKeys = LandmarkKeys;
	};

	/* an array of all keys stored within the navmesh */
	UPROPERTY()
	TArray<FIntVector> AllKeys;

	/* a map containing all of the node instances */
	UPROPERTY()
	TMap<FIntVector, FCubicNavMeshNode> Nodes;

	/* an array constaining the keys of all landmark nodes */
	UPROPERTY()
	TArray<FIntVector> LandmarkKeys;
};
