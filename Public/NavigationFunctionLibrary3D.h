// Cam0's 3D Navigation 2023

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "NavigationFunctionLibrary3D.generated.h"

/**
 * 
 */

UCLASS()
class CAM0S3DNAVIGATION_API UNavigationFunctionLibrary3D : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	
	UFUNCTION(BlueprintCallable, Category = "Cam0's 3D Pathfinding")
	static TArray<FVector> GetPath3D(AActor* ActorToMove, FVector TargetLocation, bool DrawPath);
};
