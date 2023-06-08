// Cam0's 3D Navigation 2023

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "Cam0s3DNavigationInterface.generated.h"

/**
 * An interface for Cam0's 3D Navigation classes
 * allows these methods to be defined by any class which inherets this interface
 * @Author Cameron Greenway 2023
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
