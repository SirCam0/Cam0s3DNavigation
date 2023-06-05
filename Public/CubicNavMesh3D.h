// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/BoxComponent.h"
#include "GameFramework/Actor.h"
#include "CubicNavMeshNode.h"
#include "Cam0s3DNavigationInterface.h"
#include "CubicNavMesh3D.generated.h"


/**
 * 
 */
UCLASS()
class CAM0S3DNAVIGATION_API ACubicNavMesh3D : public AActor, public ICam0s3DNavigationInterface
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ACubicNavMesh3D();

	UFUNCTION()
	FString GetNavMeshID() const;

	UFUNCTION()
	void LoadNavData();

	UFUNCTION()
	void SaveNavData() const;

	/**
	* True for 18, false for 6
	*/
	UPROPERTY(EditAnywhere, Category = "Cubic Nav Mesh 3D")
	bool bNeighbourType = true;

	UPROPERTY(VisibleAnywhere, Category = "Cubic Nav Mesh 3D", meta = (ClampMin = 1))
	int NumberOfNodes;
	
	UPROPERTY(EditAnywhere, Category = "Cubic Nav Mesh 3D")
	FVector Bounds = {200,200,200};

	UPROPERTY(EditAnywhere, meta = (ClampMin = 15), Category = "Cubic Nav Mesh 3D")
	int Resolution = 50;

	UPROPERTY(VisibleAnywhere, Category = "Cubic Nav Mesh 3D|Landmarks", meta = (ClampMin = 2))
	int NumberOfLandmarkNodes;

	UPROPERTY(EditAnywhere, meta = (ClampMax = 50, ClampMin = 1), Category = "Cubic Nav Mesh 3D|Landmarks")
	int PercentageOfNodesAsLandmarks = 10;
	
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "Cubic Nav Mesh 3D")
	void TestComplexPathFind();

	UFUNCTION(CallInEditor, BlueprintCallable, Category = "Cubic Nav Mesh 3D")
	void UpdateBounds() const;

	UFUNCTION(BlueprintCallable, Category = "Cubic Nav Mesh 3D")
	void UpdateNumberOfNodes();

	UFUNCTION(BlueprintCallable, Category = "Cubic Nav Mesh 3D")
	void UpdateNumberOfLandmarks();

	UFUNCTION(CallInEditor, Category = "Cubic Nav Mesh 3D")
	void GenerateCubicNavMesh();

	ACubicNavMesh3D* GetCubicNavMesh3DReference_Implementation() override;

	UPROPERTY(BlueprintReadWrite, Category = "3DNavMesh")
	UBoxComponent* NavMeshBoundsBox;

	UPROPERTY()
	bool DrawDijkstra = false;
	
	UFUNCTION()
	void TestBasicPathFind();
	
	UFUNCTION(BlueprintCallable, Category = "3DNavMesh")
	TArray<FVector> GetPath(FVector StartLocation, FVector TargetLocation, bool DrawPath);

	UFUNCTION()
	void DrawLandmarkPairs();

	UFUNCTION()
	bool CheckRelativeNeighbourOverlap(FIntVector Key, FIntVector RelativeNeighbour);

	UFUNCTION()
	bool CheckCorner(FIntVector Key, FIntVector NeighbourKeyCheck);

	UFUNCTION()
	TArray<FIntVector> AStarPathFind(FVector StartPos, FVector TargetPos, bool DrawPath);
	
	TPair<bool, float> UEBoxTraceOverlapCheck(FVector NodeLocation, FRotator NodeRotation) const;

	UFUNCTION()
	TArray<FIntVector> GenerateNeighbourKeys6Sided(FIntVector InputKey) const;

	UFUNCTION()
	TArray<FIntVector> GenerateNeighbourKeys18Sided(FIntVector InputKey) const;

	UFUNCTION()
	void ShowNeighbours(FIntVector Key);

	UPROPERTY()
	TMap<FIntVector,FCubicNavMeshNode> Nodes;
	
	FCubicNavMeshNode* GetBestNextNodeStraightLineDistance(FCubicNavMeshNode* StartNode, FVector TargetLocation, const TArray<FIntVector>& VisitedNodes);

	UFUNCTION()
	void BruteForcePathFind(FVector StartPos, FVector TargetPos);
	
	FCubicNavMeshNode* GetClosestNode(FVector Location);
	
	FCubicNavMeshNode* GetClosestLandmark(FVector Location);

	UPROPERTY()
	TArray<FIntVector> AllKeys;

	UPROPERTY()
	TArray<FIntVector> Landmarks;

	UFUNCTION()
	void GenerateWaypointsRandom();

	UFUNCTION()
	void GenerateWaypointsUniform(int Factor);

	UFUNCTION()
	void GetPathLengthToOtherLandmarks(FIntVector StartNodeKey);

	TMap<FIntVector, float> DefaultDistances;

	UFUNCTION()
	void DrawNode(FIntVector Key, FColor Colour);

	UFUNCTION()
	void DrawLine(FIntVector Key1, FIntVector Key2, FColor Colour);

	UFUNCTION()
	void DrawFloat(FVector Location, float Num, FColor Colour, float Duration) const;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
