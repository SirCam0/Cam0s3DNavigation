// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "Cam0s3DNavigationInterface.generated.h"

/**
 * 
 */

class USaveDataCubicNavigation;
class ANavMesh3DModifier;
class ACubicNavMesh3D;

UINTERFACE(MinimalAPI)
class UCam0s3DNavigationInterface : public UInterface
{
	GENERATED_BODY()
};

class CAM0S3DNAVIGATION_API ICam0s3DNavigationInterface
{
	GENERATED_BODY()

public:

	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = Cam0s3DNavigation)
	USaveDataCubicNavigation* GetCubicNavDataReference();

	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = Cam0s3DNavigation)
	ANavMesh3DModifier* GetNavMesh3DModifierReference();

	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = Cam0s3DNavigation)
	ACubicNavMesh3D* GetCubicNavMesh3DReference();
	
};
