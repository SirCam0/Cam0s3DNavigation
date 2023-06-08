// Cam0's 3D Navigation 2023

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/SaveGame.h"
#include "Cam0s3DNavigationInterface.h"
#include "CubicNavData.h"
#include "CubicNavMeshNode.h"
#include "SaveDataCubicNavigation.generated.h"

/**
 * 
 */
UCLASS()
class CAM0S3DNAVIGATION_API USaveDataCubicNavigation : public USaveGame, public ICam0s3DNavigationInterface
{
	GENERATED_BODY()

	USaveDataCubicNavigation* GetCubicNavDataReference_Implementation() override;

public:

	UPROPERTY()
	TMap<FString, FCubicNavData> NavMeshes;

	UPROPERTY()
	TArray<FString> Names;

	void SaveNewNavMesh(const FString NavMesh, const TArray<FIntVector> AllKeys, const TMap<FIntVector, FCubicNavMeshNode> Nodes, const TArray<FIntVector> LandmarkKeys);

	void UpdateNavMesh(const FString NavMesh, const TArray<FIntVector> AllKeys, const TMap<FIntVector, FCubicNavMeshNode> Nodes, const TArray<FIntVector> LandmarkKeys);

	FCubicNavData LoadNavMesh(const FString NavMesh);

	bool CheckSaved(const FString NavMesh) const;
};
