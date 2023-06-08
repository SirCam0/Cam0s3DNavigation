// Cam0's 3D Navigation 2023

#pragma once

#include "CoreMinimal.h"
//#include "PathfindType.generated.h"

/**
 * 
 */
UENUM(BlueprintType)
enum EPathfindType
{
	AStarLandmark	UMETA(DistplayName = "A Star Landmark"),
	AStarStraightLine	UMETA(DistplayName = "A Star Straight Line"),
};
