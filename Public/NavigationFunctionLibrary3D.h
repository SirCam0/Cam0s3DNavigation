// Fill out your copyright notice in the Description page of Project Settings.

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
