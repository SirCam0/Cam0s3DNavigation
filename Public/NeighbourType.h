// Cam0's 3D Navigation 2023

#pragma once

#include "CoreMinimal.h"
#include "NeighbourType.generated.h"

/**
 * 
 */
UENUM(BlueprintType)
enum ENeighbourType
{
	SixSided  UMETA(DisplayName = "Six Sided"),
	EighteenSided	UMETA(DisplayName = "Eighteen Sided"),
};
