// Cam0's 3D Navigation 2023

#include "NavigationFunctionLibrary3D.h"
#include "CubicNavMesh3D.h"
#include <Kismet/GameplayStatics.h>
#include <Kismet/KismetMathLibrary.h>
#include "Cam0s3DNavigationInterface.h"

TArray<FVector> UNavigationFunctionLibrary3D::GetPath3D(AActor* ActorToMove, FVector TargetLocation, bool DrawPath)
{
	TArray<AActor*> NavMeshes;
	UGameplayStatics::GetAllActorsOfClass(ActorToMove, ACubicNavMesh3D::StaticClass(), NavMeshes);
	float ClosestDistance = INFINITY;
	ACubicNavMesh3D* ClosestNavMesh = nullptr;
	for (AActor* NavMesh : NavMeshes) {
		if (NavMesh->Implements<UCam0s3DNavigationInterface>()) {
			ACubicNavMesh3D* Nav = ICam0s3DNavigationInterface::Execute_GetCubicNavMesh3DReference(NavMesh);
			const float CurrentDistance = UKismetMathLibrary::Vector_Distance(Nav->GetActorLocation(), ActorToMove->GetActorLocation());
			if (CurrentDistance < ClosestDistance) {
				ClosestDistance = CurrentDistance;
				ClosestNavMesh = Nav;
			}
		}
	}
	if (ClosestNavMesh) {
		TArray<FVector> Path = ClosestNavMesh->GetPath(ActorToMove->GetActorLocation(), TargetLocation, DrawPath);
		return Path;
	}
	UE_LOG(LogTemp, Warning, TEXT("No Nav Mesh Found for pathfinding"));
	return {};
}
