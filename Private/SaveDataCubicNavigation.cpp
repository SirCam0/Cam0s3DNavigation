// Cam0's 3D Navigation 2023


#include "SaveDataCubicNavigation.h"
#include <Kismet/KismetSystemLibrary.h>

USaveDataCubicNavigation* USaveDataCubicNavigation::GetCubicNavDataReference_Implementation()
{
	return this;
}

void USaveDataCubicNavigation::SaveNewNavMesh(const FString NavMesh, const TArray<FIntVector> AllKeys, const TMap<FIntVector, FCubicNavMeshNode> Nodes, const TArray<FIntVector> LandmarkKeys)
{
	if (!CheckSaved(NavMesh)) {
		NavMeshes.Add(NavMesh, FCubicNavData(AllKeys, Nodes, LandmarkKeys));
		Names.AddUnique(NavMesh);
	}
	else {
		UpdateNavMesh(NavMesh, AllKeys, Nodes, LandmarkKeys);
	}
}

void USaveDataCubicNavigation::UpdateNavMesh(const FString NavMesh, const TArray<FIntVector> AllKeys, const TMap<FIntVector, FCubicNavMeshNode> Nodes, const TArray<FIntVector> LandmarkKeys)
{
	if (NavMeshes.Find(NavMesh)) {
		NavMeshes.Remove(NavMesh);
		NavMeshes.Add(NavMesh, FCubicNavData(AllKeys, Nodes, LandmarkKeys));
		Names.AddUnique(NavMesh);
	}
	else {
		SaveNewNavMesh(NavMesh, AllKeys, Nodes, LandmarkKeys);
	}
}

FCubicNavData USaveDataCubicNavigation::LoadNavMesh(const FString NavMesh)
{
	return *NavMeshes.Find(NavMesh);
}

bool USaveDataCubicNavigation::CheckSaved(const FString NavMesh) const
{
	if (!NavMeshes.IsEmpty()) {
		//for(FString Name: Names)
		//{
			//GEngine->AddOnScreenDebugMessage(-1,5,FColor::Red, Name);
		//}
		//GEngine->AddOnScreenDebugMessage(-1,5,FColor::Green, NavMesh);
		if (NavMeshes.Contains(NavMesh)) return true;
	}
	return false;
}
