// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <Components/BoxComponent.h>
#include "Cam0s3DNavigationInterface.h"
#include "NavMesh3DModifier.generated.h"

/**
 * 
 */
UCLASS()
class CAM0S3DNAVIGATION_API ANavMesh3DModifier : public AActor, public ICam0s3DNavigationInterface
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ANavMesh3DModifier();

	ANavMesh3DModifier* GetNavMesh3DModifierReference_Implementation() override;

	UBoxComponent* Volume;

	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, ClampMax = 10), Category = "Modifiers")
	float Cost = 1;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
