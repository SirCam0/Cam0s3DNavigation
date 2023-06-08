// Cam0's 3D Navigation 2023


#include "NavMesh3DModifier.h"

ANavMesh3DModifier::ANavMesh3DModifier()
{
	PrimaryActorTick.bCanEverTick = true;

	Volume = CreateDefaultSubobject<UBoxComponent>("Volume");
	this->RootComponent = Volume;
	Volume->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	Volume->SetCollisionObjectType(ECollisionChannel::ECC_WorldStatic);
	this->Tags.Add("NavMeshModifierVolume");
}

ANavMesh3DModifier* ANavMesh3DModifier::GetNavMesh3DModifierReference_Implementation()
{
	return this;
}

// Called when the game starts or when spawned
void ANavMesh3DModifier::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ANavMesh3DModifier::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}
