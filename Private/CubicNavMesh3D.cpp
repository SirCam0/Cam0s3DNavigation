// Fill out your copyright notice in the Description page of Project Settings.


#include "CubicNavMesh3D.h"
#include "Components/BoxComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Misc/MessageDialog.h"
#include <Kismet/GameplayStatics.h>
#include "SaveDataCubicNavigation.h"
//#include "Cam0sPathfindingAlgorithms.h"
#include "NavMesh3DModifier.h"

#define RAND_INT_IN_RANGE UKismetMathLibrary::RandomIntegerInRange


// Sets default values
ACubicNavMesh3D::ACubicNavMesh3D()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	NavMeshBoundsBox = CreateDefaultSubobject<UBoxComponent>("NavMeshBox");
	RootComponent = NavMeshBoundsBox;
	NavMeshBoundsBox->SetBoxExtent(Bounds);
	NavMeshBoundsBox->SetLineThickness(3);
	NavMeshBoundsBox->ShapeColor = FColor::Cyan;

	Nodes.Empty();
}

FString ACubicNavMesh3D::GetNavMeshID() const
{
	FString Name = UKismetSystemLibrary::GetDisplayName(this);
	FString MapName = UGameplayStatics::GetCurrentLevelName(this, true);
	return Name.Append(MapName);
}

void ACubicNavMesh3D::LoadNavData() {
	if (UGameplayStatics::DoesSaveGameExist("NavData", 0)) {
		if (USaveGame* SaveData = UGameplayStatics::LoadGameFromSlot("NavData", 0); SaveData->Implements<UCam0s3DNavigationInterface>()) {
			if (USaveDataCubicNavigation* NavData = ICam0s3DNavigationInterface::Execute_GetCubicNavDataReference(SaveData); NavData->CheckSaved(GetNavMeshID())) {
				const FCubicNavData Data = NavData->LoadNavMesh(GetNavMeshID());
				this->AllKeys = Data.AllKeys;
				this->Nodes = Data.Nodes;
				this->Landmarks = Data.LandmarkKeys;
				return;
			}
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("No Save data found for this nav mesh"));
}

void ACubicNavMesh3D::SaveNavData() const
{
	if (UGameplayStatics::DoesSaveGameExist("NavData", 0)) {
		USaveGame* SaveData = UGameplayStatics::LoadGameFromSlot("NavData", 0);
		if (SaveData->Implements<UCam0s3DNavigationInterface>()) {
			USaveDataCubicNavigation* NavData = ICam0s3DNavigationInterface::Execute_GetCubicNavDataReference(SaveData);
			if (NavData->CheckSaved(GetNavMeshID())) {
				NavData->UpdateNavMesh(GetNavMeshID(), AllKeys, Nodes, Landmarks);
			}
			else{
				NavData->SaveNewNavMesh(GetNavMeshID(), AllKeys, Nodes, Landmarks);
			}
			UGameplayStatics::SaveGameToSlot(NavData, "NavData", 0);
		}
	}
	else{
		USaveGame* SaveGame = UGameplayStatics::CreateSaveGameObject(USaveDataCubicNavigation::StaticClass());
		if (SaveGame->Implements<UCam0s3DNavigationInterface>()) {
			USaveDataCubicNavigation* NavData = ICam0s3DNavigationInterface::Execute_GetCubicNavDataReference(SaveGame);
			NavData->SaveNewNavMesh(GetNavMeshID(), AllKeys, Nodes, Landmarks);
			UGameplayStatics::SaveGameToSlot(NavData, "NavData", 0);
		}
	}
}

ACubicNavMesh3D* ACubicNavMesh3D::GetCubicNavMesh3DReference_Implementation()
{
	return this;
}

void ACubicNavMesh3D::GenerateCubicNavMesh(){

	if(NumberOfNodes < 2)
	{
		FMessageDialog::Open(EAppMsgType::Ok, FText::FromString("Nav Mesh must have more than 1 Nodes"));
		return;
	}
	FMessageDialog::Open(EAppMsgType::Ok, FText::FromString("Nav Mesh Building, This may take a while"));
	
	FIntVector CurrentLocationKey = {0,0,0};
	
	Nodes.Empty();
	AllKeys.Empty();
	Landmarks.Empty();
	DefaultDistances.Empty();

	UKismetSystemLibrary::FlushPersistentDebugLines(this);
	
	const int ResX = floor(Bounds.X/Resolution);
	const float OffsetX = Bounds.X/Resolution - ResX;
	const int ResY= floor(Bounds.Y/Resolution);
	const float OffsetY = Bounds.Y/Resolution - ResY;
	const int ResZ = floor(Bounds.Z/Resolution);
	const float OffsetZ = Bounds.Z/Resolution - ResZ;

	const FVector BoundsRotated = NavMeshBoundsBox->GetComponentRotation().RotateVector(Bounds);
	const FVector Origin = NavMeshBoundsBox->GetComponentLocation() + FVector(BoundsRotated.X - (OffsetX * Resolution), BoundsRotated.Y- (OffsetY * Resolution), BoundsRotated.Z - (OffsetZ * Resolution));

	for(int x = 0; x < ResX; x++)
	{
		CurrentLocationKey = FIntVector(x, CurrentLocationKey.Y, CurrentLocationKey.Z);
		for(int y = 0; y < ResY; y++)
		{
			CurrentLocationKey = FIntVector(CurrentLocationKey.X, y, CurrentLocationKey.Z);
			for(int z = 0; z < ResZ; z++)
			{
				CurrentLocationKey = FIntVector(CurrentLocationKey.X, CurrentLocationKey.Y, z);
				AllKeys.AddUnique(CurrentLocationKey);
				FVector CurrentNodeWorldLocation = FVector(x,y,z) * Resolution * 2;
				FVector RotatedCurrentNodeWorldLocation = NavMeshBoundsBox->GetComponentRotation().RotateVector(CurrentNodeWorldLocation);

				FCubicNavMeshNode NewNode;
				NewNode.WorldLocation = Origin-Resolution-RotatedCurrentNodeWorldLocation;
				NewNode.NavMeshLocationKey = CurrentLocationKey;
				NewNode.NeighbourKeys = bNeighbourType ? GenerateNeighbourKeys18Sided(CurrentLocationKey) : GenerateNeighbourKeys6Sided(CurrentLocationKey);


				const TPair<bool, float> OverlapValues = UEBoxTraceOverlapCheck(NewNode.WorldLocation, NavMeshBoundsBox->GetComponentRotation());

				NewNode.TraversalCost = OverlapValues.Value;
				//UE_LOG(LogTemp, Display, TEXT("%f"), NewNode.TraversalCost);
				NewNode.bOverlapping = OverlapValues.Key;

				Nodes.Add(CurrentLocationKey, NewNode);
			}
		}
	}
	UKismetSystemLibrary::DrawDebugBox(this, this->GetActorLocation(), Bounds, FColor::White, this->GetActorRotation(), 10, 2);

	//GenerateWaypointsUniform(floor((Bounds / FVector(Resolution) / FVector(NumberOfPrecompiledWaypoints)).Length()));
	GenerateWaypointsRandom();

	SaveNavData();

	UE_LOG(LogTemp, Display, TEXT("Nav Mesh finished compiling"));

	//ShowNeighbours({ 2, 2, 2 });

	//DrawLandmarkPairs();
}

void ACubicNavMesh3D::TestBasicPathFind() {
	BruteForcePathFind(this->GetActorLocation() + Bounds, this->GetActorLocation() - Bounds);
}

void ACubicNavMesh3D::TestComplexPathFind() {
	AStarPathFind(this->GetActorLocation() + Bounds, this->GetActorLocation() - Bounds, true);
}

void ACubicNavMesh3D::UpdateBounds() const
{
	NavMeshBoundsBox->SetBoxExtent(Bounds);
}

void ACubicNavMesh3D::UpdateNumberOfNodes()
{
	const int MaxX = floor(Bounds.X/Resolution) -1;
	const int MaxY = floor(Bounds.Y/Resolution) -1;
	const int MaxZ = floor(Bounds.Z/Resolution) -1;
	this->NumberOfNodes = MaxX * MaxY * MaxZ;
}

void ACubicNavMesh3D::UpdateNumberOfLandmarks()
{
	this->NumberOfLandmarkNodes = this->NumberOfNodes * PercentageOfNodesAsLandmarks/100 > 2 ? this->NumberOfNodes * PercentageOfNodesAsLandmarks/100 : 2;  
}

TArray<FVector> ACubicNavMesh3D::GetPath(FVector StartLocation, FVector TargetLocation, bool DrawPath) {
	TArray<FIntVector> KeyPath = AStarPathFind(StartLocation, TargetLocation, DrawPath);
	TArray<FVector> Path;
	for (int i = KeyPath.Num() -1; i >= 0; i--) {
		Path.Add(Nodes.Find(KeyPath[i])->WorldLocation);
	}
	return Path;
}

void ACubicNavMesh3D::DrawLandmarkPairs() {

	TArray<TPair<FVector, FVector>> DrawnPairs;

	for (FIntVector Landmark : Landmarks) {
		FCubicNavMeshNode* CurrentLandmark = Nodes.Find(Landmark);
		for (FIntVector Lmk : Landmarks) {
			if (Lmk != Landmark) {
				TPair<FVector, FVector> Pair1 = { FVector(Landmark.X, Landmark.Y, Landmark.Z), FVector(Lmk.X, Lmk.Y, Lmk.Z) };
				TPair<FVector, FVector> Pair2 = { FVector(Lmk.X, Lmk.Y, Lmk.Z), FVector(Landmark.X, Landmark.Y, Landmark.Z) };
				if (!(DrawnPairs.Contains(Pair1) || DrawnPairs.Contains(Pair2))){
					DrawnPairs.Add(Pair1);
					DrawnPairs.Add(Pair2);
					const FColor Colour = FColor::MakeRandomColor();
					DrawLine(CurrentLandmark->NavMeshLocationKey, Lmk, Colour);
					DrawFloat({0,0,0}, 1000, Colour, 10);
					if (!CurrentLandmark->bOverlapping) {
						float distance = *CurrentLandmark->PrecompiledDistances.Find(Lmk);
						UE_LOG(LogTemp, Display, TEXT("%d, %d, %d - %d, %d, %d = %f"), Landmark.X, Landmark.Y, Landmark.Z, Lmk.X, Lmk.Y, Lmk.Z, distance);
					}
				}
			}
		}
	}
}

TPair<bool, float> ACubicNavMesh3D::UEBoxTraceOverlapCheck(FVector NodeLocation, FRotator NodeRotation) const
{
	TArray<FHitResult> Hits;
	TArray<TEnumAsByte<EObjectTypeQuery>> ObjectTypes = {
		EObjectTypeQuery::ObjectTypeQuery1,   //Static
		EObjectTypeQuery::ObjectTypeQuery2};  //Dynamic
	TArray<AActor*> IgnoredActors = {};
	UKismetSystemLibrary::BoxTraceMultiForObjects(this,
		NodeLocation,
		NodeLocation,
		FVector(Resolution),
		NodeRotation,
		ObjectTypes,
		false,
		IgnoredActors,
		NumberOfNodes < 3000 ? EDrawDebugTrace::ForDuration : EDrawDebugTrace::None,
		Hits,
		true,
		FColor::White,
		FColor::Red,
		10);

	bool Overlapping = false;
	float Cost = 1;
	for (FHitResult Hit : Hits) {
		if (Hit.bBlockingHit && !Hit.GetActor()->ActorHasTag("NavMeshModifierVolume"))
		{
			Overlapping = true;
			UKismetSystemLibrary::DrawDebugBox(this,NodeLocation, FVector(Resolution),FColor::Red, NodeRotation, 10, 1);
		}
		if (Hit.GetActor()->Implements<UCam0s3DNavigationInterface>() && Hit.GetActor()->ActorHasTag("NavMeshModifierVolume")) {
			if (ANavMesh3DModifier* Vol = ICam0s3DNavigationInterface::Execute_GetNavMesh3DModifierReference(Hit.GetActor())) {
				Cost = Vol->Cost;
				FColor Colour;
				if(Cost < 1)
				{
					uint8 G = 255 - (Cost * 255);
					Colour = { 100, G, 100, 1 };
				}
				else if(Cost > 1)
				{

					float Alpha = UKismetMathLibrary::MapRangeClamped(Cost, 1, 10, 0, 1);
					FVector Red = FVector(255, 100, 100);
					FVector Final = UKismetMathLibrary::VLerp(FVector(0,0,0), Red, Alpha);
					uint8 R = Final.X;
					uint8 G = Final.Y;
					uint8 B = Final.Z;
					uint8 A = 1;
					Colour = { R, G, B, A };
				}
				else
				{
					Colour = FColor::White;
				}
				UKismetSystemLibrary::DrawDebugBox(this, NodeLocation, FVector(Resolution), Colour, NodeRotation, 10, 1);
			}
		}
	}

	TPair<bool, float> ReturnVal;
	ReturnVal.Key = Overlapping;
	ReturnVal.Value = Cost;
	return ReturnVal;
}

TArray<FIntVector> ACubicNavMesh3D::GenerateNeighbourKeys6Sided(FIntVector InputKey) const
{
	TArray<FIntVector> Neighbours = {};
	
	const int MaxX = floor(Bounds.X / Resolution);
	const int MaxY = floor(Bounds.Y / Resolution);
	const int MaxZ = floor(Bounds.Z / Resolution);
	
	if(!(InputKey.X + 1 >= MaxX))
	{
		Neighbours.Add(FIntVector(InputKey.X+1, InputKey.Y, InputKey.Z));
	}
	if(!(InputKey.X - 1 < 0))
	{
		Neighbours.Add(FIntVector(InputKey.X-1, InputKey.Y, InputKey.Z));
	}
	if(!(InputKey.Y + 1 >= MaxY))
	{
		Neighbours.Add(FIntVector(InputKey.X, InputKey.Y+1, InputKey.Z));
	}
	if(!(InputKey.Y - 1 < 0))
	{
		Neighbours.Add(FIntVector(InputKey.X, InputKey.Y-1, InputKey.Z));
	}
	if(!(InputKey.Z + 1 >= MaxZ))
	{
		Neighbours.Add(FIntVector(InputKey.X, InputKey.Y, InputKey.Z+1));
	}
	if(!(InputKey.Z - 1 < 0))
	{
		Neighbours.Add(FIntVector(InputKey.X, InputKey.Y, InputKey.Z-1));
	}
	
	return Neighbours;
}

TArray<FIntVector> ACubicNavMesh3D::GenerateNeighbourKeys18Sided(FIntVector InputKey) const
{
	//UE_LOG(LogTemp, Display, TEXT("Generating 18 neighbours"));

	TArray<FIntVector> Neighbours = {};

	Neighbours.Append(GenerateNeighbourKeys6Sided(InputKey));

	const int MaxX = floor(Bounds.X / Resolution);
	const int MaxY = floor(Bounds.Y / Resolution);
	const int MaxZ = floor(Bounds.Z / Resolution);


	//Front
	if (!(InputKey.X + 1 >= MaxX || InputKey.Y + 1 >= MaxY))
	{
		Neighbours.Add(FIntVector(InputKey.X + 1, InputKey.Y + 1, InputKey.Z));
	}
	if (!(InputKey.X + 1 >= MaxX || InputKey.Z + 1 >= MaxZ))
	{
		Neighbours.Add(FIntVector(InputKey.X + 1, InputKey.Y, InputKey.Z + 1));
	}
	if (!(InputKey.X + 1 >= MaxX || InputKey.Y - 1 < 0))
	{
		Neighbours.Add(FIntVector(InputKey.X + 1, InputKey.Y - 1, InputKey.Z));
	}
	if (!(InputKey.X + 1 >= MaxX || InputKey.Z - 1 < 0))
	{
		Neighbours.Add(FIntVector(InputKey.X + 1, InputKey.Y, InputKey.Z - 1));
	}
	//Back
	if (!(InputKey.X - 1 < 0 || InputKey.Y + 1 >= MaxY))
	{
		Neighbours.Add(FIntVector(InputKey.X - 1, InputKey.Y + 1, InputKey.Z));
	}
	if (!(InputKey.X - 1 < 0 || InputKey.Z + 1 >= MaxZ))
	{
		Neighbours.Add(FIntVector(InputKey.X - 1, InputKey.Y, InputKey.Z + 1));
	}
	if (!(InputKey.X - 1 < 0 || InputKey.Y - 1 < 0))
	{
		Neighbours.Add(FIntVector(InputKey.X - 1, InputKey.Y - 1, InputKey.Z));
	}
	if (!(InputKey.X - 1 < 0 || InputKey.Z - 1 < 0))
	{
		Neighbours.Add(FIntVector(InputKey.X - 1, InputKey.Y, InputKey.Z - 1));
	}
	//Left
	if (!(InputKey.Y + 1 >= MaxY || InputKey.Z + 1 >= MaxZ))
	{
		Neighbours.Add(FIntVector(InputKey.X, InputKey.Y + 1, InputKey.Z + 1));
	}
	if (!(InputKey.Y + 1 >= MaxY || InputKey.Z - 1 < 0))
	{
		Neighbours.Add(FIntVector(InputKey.X, InputKey.Y + 1, InputKey.Z - 1));
	}
	//Right
	if (!(InputKey.Y - 1 < 0 || InputKey.Z + 1 >= MaxZ))
	{
		Neighbours.Add(FIntVector(InputKey.X, InputKey.Y - 1, InputKey.Z + 1));
	}
	if (!(InputKey.Y - 1 < 0 || InputKey.Z - 1 < 0))
	{
		Neighbours.Add(FIntVector(InputKey.X, InputKey.Y - 1, InputKey.Z - 1));
	}

	return Neighbours;
}

void ACubicNavMesh3D::ShowNeighbours(FIntVector Key) {
	for (FIntVector Neighbour : Nodes.Find(Key)->NeighbourKeys) {
		UKismetSystemLibrary::DrawDebugBox(this, Nodes.Find(Neighbour)->WorldLocation, FVector(Resolution), FColor::Green, this->GetActorRotation(), 10, 5);
	}
}

FCubicNavMeshNode* ACubicNavMesh3D::GetBestNextNodeStraightLineDistance(FCubicNavMeshNode* StartNode, FVector TargetLocation, const TArray<FIntVector>& VisitedNodes)
{
	float BestDistance = INFINITY;
	FIntVector BestNextNodeKey = FIntVector(-1);
	for(int i = 0; i < StartNode->NeighbourKeys.Num(); i++)
	{
		if(const FCubicNavMeshNode* CurrentNode = Nodes.Find(StartNode->NeighbourKeys[i]))
		{
			if(!(CurrentNode->bOverlapping) && !VisitedNodes.Contains(CurrentNode->NavMeshLocationKey))
			{
				const float CurrentDistance = UKismetMathLibrary::Vector_Distance(CurrentNode->WorldLocation,TargetLocation) //h(n)
				+ UKismetMathLibrary::Vector_Distance(StartNode->WorldLocation, CurrentNode->WorldLocation) * CurrentNode->TraversalCost; // g(n)
				if(CurrentDistance < BestDistance)
				{
					BestDistance = CurrentDistance;
					BestNextNodeKey = StartNode->NeighbourKeys[i];
				}
			}
		}
	}
	if(BestNextNodeKey == FIntVector(-1))
	{
		return nullptr;
	}
	return Nodes.Find(BestNextNodeKey);
}

USTRUCT()
struct FAStarNodeInfo {
	FIntVector PreviousNodeKey = { 0,0,0 };
	float G = 0;
	float H = 0;
	float F = 0;

	FAStarNodeInfo() {
		this->PreviousNodeKey = { 0,0,0 };
		this->G = INFINITY;
		this->H = INFINITY;
		this->F = INFINITY;
	}

	FAStarNodeInfo(FIntVector PreviousNodeKey, float g, float h, float f) {
		this->PreviousNodeKey = PreviousNodeKey;
		this->G = g;
		this->H = h;
		this->F = f;
	}
};

bool ACubicNavMesh3D::CheckRelativeNeighbourOverlap(const FIntVector Key, const FIntVector RelativeNeighbour) {
	for (FIntVector Neighbour: Nodes.Find(Key)->NeighbourKeys) {
		if (Key - Neighbour == RelativeNeighbour) {
			return Nodes.Find(Neighbour)->bOverlapping;
		}
	}
	return false;
}

bool ACubicNavMesh3D::CheckCorner(const FIntVector Key, const FIntVector NeighbourKeyCheck) {
	const FIntVector Relative = Key - NeighbourKeyCheck;
	bool bIsCorner = false;
	if (Relative.X == 1) {
		if (Relative.Y == 1) {
			if(CheckRelativeNeighbourOverlap(Key, { 1,0,0 })) bIsCorner = true;
			if(CheckRelativeNeighbourOverlap(Key, { 0,1,0 })) bIsCorner = true;
		}
		if (Relative.Y == -1) {
			if (CheckRelativeNeighbourOverlap(Key, { 1,0,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,-1,0 })) bIsCorner = true;
		}
		if (Relative.Z == 1) {
			if (CheckRelativeNeighbourOverlap(Key, { 1,0,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,0,1 })) bIsCorner = true;
		}
		if (Relative.Z == -1) {
			if (CheckRelativeNeighbourOverlap(Key, { 1,0,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,0,-1 })) bIsCorner = true;
		}
	}
	if (Relative.X == -1) {
		if (Relative.Y == 1) {
			if (CheckRelativeNeighbourOverlap(Key, { -1,0,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,1,0 })) bIsCorner = true;
		}
		if (Relative.Y == -1) {
			if (CheckRelativeNeighbourOverlap(Key, { -1,0,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,-1,0 })) bIsCorner = true;
		}
		if (Relative.Z == 1) {
			if (CheckRelativeNeighbourOverlap(Key, { -1,0,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,0,1 })) bIsCorner = true;
		}
		if (Relative.Z == -1) {
			if (CheckRelativeNeighbourOverlap(Key, { -1,0,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,0,-1 })) bIsCorner = true;
		}
	}
	if (Relative.Y == 1) {
		if (Relative.Z == 1) {
			if (CheckRelativeNeighbourOverlap(Key, { 0,1,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,0,1 })) bIsCorner = true;
		}
		if (Relative.Z == -1) {
			if (CheckRelativeNeighbourOverlap(Key, { 0,1,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,0,-1 })) bIsCorner = true;
		}
	}
	if (Relative.Y == -1) {
		if (Relative.Z == 1) {
			if (CheckRelativeNeighbourOverlap(Key, { 0,-1,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,0,1 })) bIsCorner = true;
		}
		if (Relative.Z == -1) {
			if (CheckRelativeNeighbourOverlap(Key, { 0,-1,0 })) bIsCorner = true;
			if (CheckRelativeNeighbourOverlap(Key, { 0,0,-1 })) bIsCorner = true;
		}
	}
	return bIsCorner;
}

TArray<FIntVector> ACubicNavMesh3D::AStarPathFind(const FVector StartPos, const FVector TargetPos, bool DrawPath) {
	
	if(Nodes.IsEmpty() || AllKeys.IsEmpty() || Landmarks.IsEmpty())
	{
		LoadNavData();	
	}

	FDateTime startT = FDateTime::UtcNow();
	int32 startMs = startT.GetMillisecond();

	TArray<FIntVector> Queue {};
	TArray<FIntVector> Visited {};
	TMap<FIntVector, FAStarNodeInfo> Distances {};
	FCubicNavMeshNode* StartNode = GetClosestNode(StartPos);
	FCubicNavMeshNode* EndNode = GetClosestNode(TargetPos);
	FIntVector CurrentNode {};

	if (StartNode && EndNode) {
		if (AllKeys.Num()) {
			for (FIntVector Key : AllKeys) {
				if (Key != StartNode->NavMeshLocationKey) {
					Distances.Add(Key, FAStarNodeInfo());
				}
				else {
					FCubicNavMeshNode* ClosestLandmark = GetClosestLandmark(StartNode->WorldLocation);
					FCubicNavMeshNode* TargetLandmark = GetClosestLandmark(TargetPos);
					float DistanceToClosestLandmark = UKismetMathLibrary::Vector_Distance(StartNode->WorldLocation, ClosestLandmark->WorldLocation);
					float DistanceBetweenLandmarks = *ClosestLandmark->PrecompiledDistances.Find(TargetLandmark->NavMeshLocationKey);
					float DistanceFromEndLandmark = UKismetMathLibrary::Vector_Distance(TargetLandmark->WorldLocation, TargetPos);
					float H = DistanceToClosestLandmark + DistanceBetweenLandmarks + DistanceFromEndLandmark;
					Distances.Add(Key, FAStarNodeInfo(FIntVector(0), 0.f, H, H));
					CurrentNode = Key;
				}
			}

			bool bPathFound = false;

			do {
				if (CurrentNode == EndNode->NavMeshLocationKey) {
					bPathFound = true;
					break;
				}

				if (Queue.Contains(CurrentNode)) {
					Queue.Remove(CurrentNode);
				}
				FCubicNavMeshNode* CurrentNavNode = Nodes.Find(CurrentNode);
				for (FIntVector Neighbour : CurrentNavNode->NeighbourKeys) {
					if (!Visited.Contains(Neighbour) && !(Nodes.Find(Neighbour)->bOverlapping) && !CheckCorner(CurrentNode, Neighbour)) {
						FCubicNavMeshNode* NeighbourNode = Nodes.Find(Neighbour);
						FCubicNavMeshNode* ClosestLandmark = GetClosestLandmark(NeighbourNode->WorldLocation);
						FCubicNavMeshNode* TargetLandmark = GetClosestLandmark(TargetPos);
						float G = Distances.Find(CurrentNode)->G//distance to get to previous node
							+ (UKismetMathLibrary::Vector_Distance(CurrentNavNode->WorldLocation, NeighbourNode->WorldLocation) * NeighbourNode->TraversalCost); // distance to get to node from previous
						float H = UKismetMathLibrary::Vector_Distance(NeighbourNode->WorldLocation, ClosestLandmark->WorldLocation) +
							*ClosestLandmark->PrecompiledDistances.Find(TargetLandmark->NavMeshLocationKey) +
							UKismetMathLibrary::Vector_Distance(TargetLandmark->WorldLocation, TargetPos);
						float F = G + H;

						if (F < Distances.Find(Neighbour)->F) {
							Distances.Remove(Neighbour);
							Distances.Add(Neighbour, FAStarNodeInfo(CurrentNode, G, H, F));
						}
						Queue.AddUnique(Neighbour);
					}
				}
				Visited.AddUnique(CurrentNode);

				if (!Queue.Num()) {
					break;
				}

				float SmallestF = INFINITY;
				FIntVector SmallestDistanceNode = {};
				for (FIntVector Key : Queue) {
					if (Distances.Find(Key)->F <= SmallestF) {
						SmallestF = Distances.Find(Key)->F;
						SmallestDistanceNode = Key;
					}
				}
				CurrentNode = SmallestDistanceNode;

			} while (Queue.Num() > 1);

			if (bPathFound) {

				TArray<FIntVector> PathKeys = {};
				if (Distances.Contains(EndNode->NavMeshLocationKey)) {
					FIntVector CurrentPathKey = EndNode->NavMeshLocationKey;
					PathKeys.AddUnique(CurrentPathKey);
					while (CurrentPathKey != StartNode->NavMeshLocationKey) {
						CurrentPathKey = Distances.Find(CurrentPathKey)->PreviousNodeKey;
						PathKeys.AddUnique(CurrentPathKey);
					}
				}

				if (DrawPath) {
					for (int i = 0; i < PathKeys.Num(); i++) {
						if (i + 1 < PathKeys.Num()) {
							DrawLine(PathKeys[i], PathKeys[i + 1], FColor::Green);
						}
					}
				}
				
				UE_LOG(LogTemp, Display, TEXT("A* Successful"));
				UE_LOG(LogTemp, Display, TEXT("Nodes Expanded: %d"), Visited.Num());
				FDateTime endT = FDateTime::UtcNow();
				int32 endMs = endT.GetMillisecond();
				UE_LOG(LogTemp, Display, TEXT("Execution time: %dms"), endMs - startMs);

				return PathKeys;

			}
			else {
				UE_LOG(LogTemp, Warning, TEXT("No Possible Path"));
			}
		}
		else {
			UE_LOG(LogTemp, Warning, TEXT("Nav Mesh Not Built"));
		}
	}
	return TArray<FIntVector> {};
}

void ACubicNavMesh3D::BruteForcePathFind(const FVector StartPos, const FVector TargetPos)
{
	TArray<FIntVector> PathKeys = {};
	FCubicNavMeshNode* StartNode = GetClosestNode(StartPos);
	const FCubicNavMeshNode* EndNode = GetClosestNode(TargetPos);
	if(StartNode && EndNode)
	{
		FCubicNavMeshNode* CurrentNode = StartNode;
		PathKeys.Add(CurrentNode->NavMeshLocationKey);
		while(CurrentNode != EndNode)
		{
			CurrentNode = GetBestNextNodeStraightLineDistance(CurrentNode, TargetPos, PathKeys);
			if(CurrentNode == nullptr)
			{
				break;
			}
			PathKeys.Add(CurrentNode->NavMeshLocationKey);
		}
		for(int i = 0; i < PathKeys.Num(); i++)
		{
			if(i-1 >= 0)
			{
				UKismetSystemLibrary::DrawDebugLine(this, Nodes[PathKeys[i-1]].WorldLocation, Nodes[PathKeys[i]].WorldLocation, FColor::Red, 10, 10);
			}
		}
	}
}

FCubicNavMeshNode* ACubicNavMesh3D::GetClosestNode(const FVector Location)
{
	float ShortestDistance = INFINITY;
	FIntVector BestKey = FIntVector(-1);
	bool ClosestNodeFound = false;
	for(FIntVector Key: AllKeys)
	{
		const float CurrentDistance = UKismetMathLibrary::Vector_Distance(Nodes.Find(Key)->WorldLocation, Location);
		if(CurrentDistance <= floor(Resolution / 2))
		{
			BestKey = Key;
			ClosestNodeFound = true;
			break;
		}
		if(CurrentDistance < ShortestDistance)
		{
			ShortestDistance = CurrentDistance;
			BestKey = Key;
			ClosestNodeFound = true;
		}
	}
	if (!ClosestNodeFound) {
		UE_LOG(LogTemp, Warning, TEXT("No Closest Node"));
	}
	return Nodes.Find(BestKey);
}

FCubicNavMeshNode* ACubicNavMesh3D::GetClosestLandmark(const FVector Location)
{
	float ShortestDistance = INFINITY;
	FIntVector BestKey = FIntVector(-1);
	bool ClosestLandmarkFound = false;
	for(FIntVector Key: Landmarks)
	{
		if(!Nodes.Find(Key)->bOverlapping)
		{
			const float CurrentDistance = UKismetMathLibrary::Vector_Distance(Nodes.Find(Key)->WorldLocation, Location);
			if(CurrentDistance <= floor(Resolution / 2))
			{
				BestKey = Key;
				ClosestLandmarkFound = true;
				break;
			}
			if(CurrentDistance < ShortestDistance)
			{
				ShortestDistance = CurrentDistance;
				BestKey = Key;
				ClosestLandmarkFound = true;
			}
		}
	}
	if (!ClosestLandmarkFound) {
		UE_LOG(LogTemp, Warning, TEXT("No Closest Landmark"));
	}
	return Nodes.Find(BestKey);
}


void ACubicNavMesh3D::GenerateWaypointsRandom()
{
	Landmarks.Empty();
	DefaultDistances.Empty();
	if(Nodes.Num())
	{
		for (FIntVector Node : AllKeys)
		{
			DefaultDistances.Add(Node, -1);
		}
		const int MaxX = floor(Bounds.X/Resolution) -1;
		const int MaxY = floor(Bounds.Y/Resolution) -1;
		const int MaxZ = floor(Bounds.Z/Resolution) -1;

		for(int i = 0; i < NumberOfLandmarkNodes; i++)
		{
			//const FColor Colour = FColor::MakeRandomColor();
			const FIntVector Waypoint = FIntVector(RAND_INT_IN_RANGE(0,MaxX),RAND_INT_IN_RANGE(0,MaxY),RAND_INT_IN_RANGE(0,MaxZ));
			Landmarks.AddUnique(Waypoint);
			//DrawNode(Waypoint, Colour);
		}
		for(const FIntVector Landmark: Landmarks)
		{
			//DrawNode(Landmark, FColor::Purple);
			if(!(Nodes.Find(Landmark)->bOverlapping))
			{
				GetPathLengthToOtherLandmarks(Landmark);
			}
		}
	}
}

void ACubicNavMesh3D::GenerateWaypointsUniform(int Factor) {
	Landmarks.Empty();
	DefaultDistances.Empty();
	if (Nodes.Num())
	{
		int Iterator = 1;
		for (FIntVector Node : AllKeys)
		{
			Iterator++;
			if (Iterator >= Factor) {
				Iterator = 1;
				if (!(Nodes.Find(Node)->bOverlapping)) {
					Landmarks.AddUnique(Node);
					DrawNode(Node, FColor::Magenta);
				}
			}
			DefaultDistances.Add(Node, -1);
		}
		
		for (const FIntVector Landmark : Landmarks)
		{
			if (!(Nodes.Find(Landmark)->bOverlapping))
			{
				GetPathLengthToOtherLandmarks(Landmark);
			}
		}
	}
}

void ACubicNavMesh3D::GetPathLengthToOtherLandmarks(FIntVector StartNodeKey) //Dijkstra's Shortest path algorithm
{
	TMap<FIntVector, float> Distances = DefaultDistances;
	float MaxDistance = 0;
	FCubicNavMeshNode* CurrentNode = Nodes.Find(StartNodeKey);
	Distances.Remove(StartNodeKey);
	Distances.Add(StartNodeKey, 0);
	TArray<FIntVector> Queue = {};
	TArray<FIntVector> Visited = {};
	for (FIntVector Neighbour : CurrentNode->NeighbourKeys) {
		if (!(Nodes.Find(Neighbour)->bOverlapping)) {
			Queue.AddUnique(Neighbour);
		}
	}
	while (Queue.Num() >= 1)
	{
		for (FIntVector Neighbour : CurrentNode->NeighbourKeys)
		{
			if (!(Nodes.Find(Neighbour)->bOverlapping)) {
				if (!Visited.Contains(Neighbour)) {
					Queue.AddUnique(Neighbour);
				}
				float CurrentDistanceCheck = UKismetMathLibrary::Vector_Distance(CurrentNode->WorldLocation, Nodes.Find(Neighbour)->WorldLocation) * Nodes.Find(Neighbour)->TraversalCost;
				if (*Distances.Find(Neighbour) == -1)
				{
					Distances.Remove(Neighbour);
					Distances.Add(Neighbour, *Distances.Find(CurrentNode->NavMeshLocationKey) + CurrentDistanceCheck);
					//UE_LOG(LogTemp, Warning, TEXT("Dist: %f"), *Distances.Find(CurrentNode->NavMeshLocationKey) + CurrentDistanceCheck);
					if (*Distances.Find(CurrentNode->NavMeshLocationKey) + CurrentDistanceCheck > MaxDistance) {
						MaxDistance = *Distances.Find(CurrentNode->NavMeshLocationKey) + CurrentDistanceCheck;
					}
				}
				else
				{
					float PreviousDistanceForNode = *Distances.Find(Neighbour);
					if (PreviousDistanceForNode > *Distances.Find(CurrentNode->NavMeshLocationKey) + CurrentDistanceCheck) {
						Distances.Remove(Neighbour);
						Distances.Add(Neighbour, *Distances.Find(CurrentNode->NavMeshLocationKey) + CurrentDistanceCheck);
					}
					//UE_LOG(LogTemp, Warning, TEXT("Dist: %f"), *Distances.Find(CurrentNode->NavMeshLocationKey) + CurrentDistanceCheck);
					if (*Distances.Find(CurrentNode->NavMeshLocationKey) + CurrentDistanceCheck > MaxDistance) {
						MaxDistance = *Distances.Find(CurrentNode->NavMeshLocationKey) + CurrentDistanceCheck;
					}
				}
			}
			else {
				Distances.Remove(Neighbour);
				Distances.Add(Neighbour, INFINITY);
			}
		}
		Queue.Remove(CurrentNode->NavMeshLocationKey);
		Visited.AddUnique(CurrentNode->NavMeshLocationKey);
		//UE_LOG(LogTemp, Warning, TEXT("%f, %f, %f Checked"), CurrentNode->NavMeshLocationKey.X, CurrentNode->NavMeshLocationKey.Y, CurrentNode->NavMeshLocationKey.Z);
		if (Queue.Num() <= 0) {
			break;
		}
		FIntVector SmallestDistNode = Queue[0];
		for (FIntVector Next : Queue) {
			if (Distances.Find(Next) < Distances.Find(SmallestDistNode)) {
				SmallestDistNode = Next;
			}
		}
		CurrentNode = Nodes.Find(SmallestDistNode);
	}
	//UE_LOG(LogTemp, Warning, TEXT("MaxDistance: %f"), MaxDistance);
	if (DrawDijkstra) {
		for (FIntVector Loc : AllKeys)
		{
			float distance = *Distances.Find(Loc);
			float Alpha = UKismetMathLibrary::MapRangeClamped(distance, 0, MaxDistance, 0, 1);
			UE_LOG(LogTemp, Warning, TEXT("Dist: %f"), distance);
			FVector Red = FVector(255, 0, 0);
			FVector Green = FVector(0, 255, 0);
			FVector Final = UKismetMathLibrary::VLerp(Green, Red, Alpha);
			uint8 R = Final.X;
			uint8 G = Final.Y;
			uint8 B = Final.Z;
			uint8 A = 0;
			FColor Colour = { R, G, B, A };
			if (distance == 0) {
				Colour = FColor::Magenta;
			}
			UKismetSystemLibrary::DrawDebugPoint(this, Nodes.Find(Loc)->WorldLocation, 10, Colour, 10);
		}
	}
	for (FIntVector Key : Landmarks)
	{
		Nodes.Find(StartNodeKey)->PrecompiledDistances.Add(Key, *Distances.Find(Key));
	}
}


void ACubicNavMesh3D::DrawNode(FIntVector Key, FColor Colour)
{
	if(Nodes.Find(Key))
	{
		UKismetSystemLibrary::DrawDebugPoint(this, Nodes.Find(Key)->WorldLocation, 20, Colour, 10);
	}
}

void ACubicNavMesh3D::DrawLine(FIntVector Key1, FIntVector Key2, FColor Colour)
{
	if(Nodes.Find(Key1) && Nodes.Find(Key2))
	{
		UKismetSystemLibrary::DrawDebugLine(this, Nodes.Find(Key1)->WorldLocation, Nodes.Find(Key2)->WorldLocation, Colour, 10, 5);
	}
}

void ACubicNavMesh3D::DrawFloat(FVector Location, float Num, FColor Colour, float Duration) const
{
	UKismetSystemLibrary::DrawDebugString(this, Location, *FString::Printf(TEXT("%f"), Num), nullptr, Colour, Duration);
}

// Called when the game starts or when spawned
void ACubicNavMesh3D::BeginPlay()
{
	Super::BeginPlay();
	LoadNavData();
}

// Called every frame
void ACubicNavMesh3D::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

