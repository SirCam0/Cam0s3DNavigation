// Cam0's 3D Navigation 2023

#pragma once

#include "CoreMinimal.h"
#include "Components/BoxComponent.h"
#include "GameFramework/Actor.h"
#include "CubicNavMeshNode.h"
#include "Cam0s3DNavigationInterface.h"
#include "NeighbourType.h"
#include "PathfindType.h"
#include "CubicNavMesh3D.generated.h"


/**
 * A class containing all of the functionallity for building a 3D NavMesh and conducting pathfinding opperations
 * @Author Cameron Greenway 2023
 */

UCLASS()
class CAM0S3DNAVIGATION_API ACubicNavMesh3D : public AActor, public ICam0s3DNavigationInterface
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ACubicNavMesh3D();

	/*
	* This function will be used to generate a unique string id to represent a specific nav mesh instance,
	* this ID can then be used to save all the built nav mesh data and be able to access it again later 
	* as the ID will always stay the same for a specific instance.
	* @Return FString a unique id of this nav mesh
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	FString GetNavMeshID() const;

	/*
	* Used to load all the previously compiled nav data from a save game slot such as the nodes map, 
	* landmarks array and AllKeys array.
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	void LoadNavData();

	/* 
	* Used to save all the currently compiled nav data to a save game slot.
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	void SaveNavData() const;

	/**
	* Set the type of neighbour generation the be used
	* 6 neighbours will provide faster results
	* 18 nighbours will provide smoother paths
	*/
	UPROPERTY(EditAnywhere, Category = "Cubic Nav Mesh 3D")
	TEnumAsByte<ENeighbourType> NeighbourType = ENeighbourType::SixSided;

	/**
	* Set the type of Pathfinding heuristic to be used
	* for best results A Star Stright Line is recommened
	*/
	UPROPERTY(EditAnywhere, Category = "Cubic Nav Mesh 3D")
	TEnumAsByte<EPathfindType> PathfindType = EPathfindType::AStarLandmark;

	/**
	* Use all nodes as landmark nodes for landmark Heuristic pathfinding,
	* Not recommended on NavMeshes larger than 800 Nodes
	* will only be used if A Star Landmark Pathfind type is selected
	*/
	UPROPERTY(EditAnywhere, Category = "Cubic Nav Mesh 3D")
	bool bPrecomputeAllNodes = false;

	/**
	* the current number of nodes that will be used to build the nav mesh
	*/
	UPROPERTY(VisibleAnywhere, Category = "Cubic Nav Mesh 3D", meta = (ClampMin = 1))
	int NumberOfNodes;
	
	/**
	* The Extent of the Nav Mesh Bounds Volume
	*/
	UPROPERTY(EditAnywhere, Category = "Cubic Nav Mesh 3D")
	FVector Bounds = {200,200,200};

	/**
	* The resolution of pathfinding nodes
	* for best results this value must be equal to your character size
	*/
	UPROPERTY(EditAnywhere, meta = (ClampMin = 15), Category = "Cubic Nav Mesh 3D")
	int Resolution = 50;

	UPROPERTY(EditAnywhere, Category = "Neighbours")
	FIntVector NeighbourToDisplay = { 0,0,0 };

	UFUNCTION(CallInEditor, Category = "Neighbours")
	void DisplayNeighbours();

	/**
	* the current number of landmark nodes that will be compiled if landmark pathfinding is selected
	*/
	UPROPERTY(VisibleAnywhere, Category = "Cubic Nav Mesh 3D|Landmarks", meta = (ClampMin = 2))
	int NumberOfLandmarkNodes;


	/**
	* the current percentage of nodes to use as landmarks that will be compiled if landmark pathfinding is selected
	*/
	UPROPERTY(EditAnywhere, meta = (ClampMax = 50, ClampMin = 1), Category = "Cubic Nav Mesh 3D|Landmarks")
	int PercentageOfNodesAsLandmarks = 10;
	

	/**
	* this function will test the current selected pathfinding method from one corner of the nav mesh to another
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "Cubic Nav Mesh 3D")
	void TestComplexPathFind();

	/**
	* called to set the extent of NavMeshBoundsBox using the bounds value
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "Cubic Nav Mesh 3D")
	void UpdateBounds() const;

	/**
	* called to calculate the number of nodes to be generated using Resolution and Bounds
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION(BlueprintCallable, Category = "Cubic Nav Mesh 3D")
	void UpdateNumberOfNodes();

	/**
	* called to update the number of nodes value using percentage of nodes as landmarks
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION(BlueprintCallable, Category = "Cubic Nav Mesh 3D")
	void UpdateNumberOfLandmarks();

	/**
	* Build the nav mesh using the current settings
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION(CallInEditor, Category = "Cubic Nav Mesh 3D")
	void GenerateCubicNavMesh();

	/*
	* Get a reference to a ACubicNavMesh3D object without casting
	* if the object inherets from ICam0s3DNavigationInterface
	* @Author Cameron Greenway 2023
	*/
	ACubicNavMesh3D* GetCubicNavMesh3DReference_Implementation() override;

	/**
	* A visual representation of the Nav Mesh Bounds Property
	*/
	UPROPERTY(BlueprintReadWrite, Category = "3DNavMesh")
	UBoxComponent* NavMeshBoundsBox;

	/**
	* Used to visually repersent the distances calculated from a landmark node using Dijkstra's Algorithm
	*/
	UPROPERTY()
	bool DrawDijkstra = false;
	
	/*
	* calculate and return a path using the currently selected Heuristic type
	* @Param StartLocation the Start location of the calculated path
	* @Param TargetLocation the End location of the calculated path
	* @Param bool DrawPath draw the path
	* @Return TArray<FVector> an array of world vectors Path
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION(BlueprintCallable, Category = "3DNavMesh")
	TArray<FVector> GetPath(FVector StartLocation, FVector TargetLocation, bool DrawPath);

	/**
	* Draw lines beteen each landmark to ever other landmark
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	void DrawLandmarkPairs();

	/**
	* A function to check if a nighour of a node is overlapping an object
	* @Param Key the node thas neighbour needs to be checked
	* @Param the relative adjacent neighbour to check in range {0,0,0} to {1,1,1}
	* @Return bool is the neighbour overlapping something
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	bool CheckRelativeNeighbourOverlap(FIntVector Key, FIntVector RelativeNeighbour);

	/**
	* A function to check if a nighour of a node requires passage fthrough a blocked corner
	* @Param Key the node thas neighbour needs to be checked
	* @Param Key of the neighbour to check
	* @Return bool does the path haver to go through a corner to reach this node
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	bool CheckCorner(FIntVector Key, FIntVector NeighbourKeyCheck);

	/*
	* calculate and return a path using a landmark based heuristic
	* @Param StartPos the Start location of the calculated path
	* @Param TargetPos the End location of the calculated path
	* @Param bool DrawPath draw the path
	* @Return TArray<FIntVector> an array of Node Keys Path
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	TArray<FIntVector> AStarPathFind(FVector StartPos, FVector TargetPos, bool DrawPath);

	/*
	* calculate and return a path using a Straight line based Heuristic
	* @Param StartPos the Start location of the calculated path
	* @Param TargetPos the End location of the calculated path
	* @Param bool DrawPath draw the path
	* @Return TArray<FIntVector> an array of Node Keys Path
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	TArray<FIntVector> AStarPathFindStrightLine(const FVector StartPos, const FVector TargetPos, bool DrawPath);
	
	/*
	* Do a box trace at the input location
	* @Param NodeLocation World location of the node
	* @Param TargetPos World rotation of the node
	* @Return TPair<bool, float> bOverlapping, TraversalCost
	* @Author Cameron Greenway 2023
	*/
	TPair<bool, float> UEBoxTraceOverlapCheck(FVector NodeLocation, FRotator NodeRotation) const;

	/*
	* Generate neighbours for a node 6 sided
	* @Param InputKey the key of the node to calculate neighbour keys for
	* @Return TArray<FIntVector> an array of Node Keys Neighbours
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	TArray<FIntVector> GenerateNeighbourKeys6Sided(FIntVector InputKey) const;

	/*
	* Generate neighbours for a node 18 sided
	* @Param InputKey the key of the node to calculate neighbour keys for
	* @Return TArray<FIntVector> an array of Node Keys Neighbours
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	TArray<FIntVector> GenerateNeighbourKeys18Sided(FIntVector InputKey) const;

	/*
	* Display the neighbouring nodes of input node
	* @Param InputKey the key of the node to display neighbours for
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	void ShowNeighbours(FIntVector Key);


	/*
	* This is a map data structure used for storing instances of the cubic
	* nav mesh node struct with a key of FIntVector
	* @Author Cameron Greenway 2023
	*/
	UPROPERTY()
	TMap<FIntVector,FCubicNavMeshNode> Nodes;
	
	/*
	* Get the closest node to the input vector
	* @Param FVector location to check
	* @Return FCubicNavMeshNode* the resulting node
	* @Author Cameron Greenway 2023
	*/
	FCubicNavMeshNode* GetClosestNode(FVector Location);
	
	/*
	* Get the closest landmark node to the input vector
	* @Param FVector location to check
	* @Return FCubicNavMeshNode* the resulting node
	* @Author Cameron Greenway 2023
	*/
	FCubicNavMeshNode* GetClosestLandmark(FVector Location);

	/*
	* helper array storing all the keys for the map Nodes
	*/
	UPROPERTY()
	TArray<FIntVector> AllKeys;

	/*
	* helper array storing all the landmark keys for the map Nodes
	*/
	UPROPERTY()
	TArray<FIntVector> Landmarks;

	/*
	* Generate n number of random keys 
	* n from percentage of nodes to use as landmarks
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	void GenerateWaypointsRandom();

	/*
	* Generate every single node as a landmark
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	void GenerateWaypointsAll();

	/*
	* Using the input landmark, precompute distances to all other landmarks
	* results stored within the precompiled distances of that node.
	* @Param the landmark node key to start from
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	void GetPathLengthToOtherLandmarks(FIntVector StartNodeKey);

	/*
	* Helper array for setting the default distances in get path length to other landmarks
	*/
	UPROPERTY()
	TMap<FIntVector, float> DefaultDistances;

	/*
	* Function for displaying a node
	* @Param Key the landmark node key
	* @Param Colour to display
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	void DrawNode(FIntVector Key, FColor Colour);

	/*
	* Function for displaying line between two nodes
	* @Param Key1 the landmark node key to start from
	* @Param Key2 the landmark node key to end on
	* @Param Colour to display
	* @Author Cameron Greenway 2023
	*/
	UFUNCTION()
	void DrawLine(FIntVector Key1, FIntVector Key2, FColor Colour);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
