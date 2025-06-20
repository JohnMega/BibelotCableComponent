
#pragma once

#include "CoreMinimal.h"
#include "Components/MeshComponent.h"
#include "GameplayTagContainer.h"
#include "BibelotCableComponent.generated.h"

class FPrimitiveSceneProxy;
class AIWCharacter;
class UAbilitySystemComponent;

/** Struct containing information about a point along the cable */
struct FBibelotCableParticle
{
	FBibelotCableParticle()
		: bFree(true)
		, Position(0, 0, 0)
		, OldPosition(0, 0, 0)
	{
	}

	/** If this point is free (simulating) or fixed to something */
	bool bFree;
	/** Current position of point */
	FVector Position;
	/** Position of point on previous iteration */
	FVector OldPosition;
};

UCLASS(hidecategories = (Object, Physics, Activation, "Components|Activation"), editinlinenew, meta = (BlueprintSpawnableComponent), ClassGroup = Rendering)
class IRONWASTE_API UBibelotCableComponent : public UMeshComponent
{
	GENERATED_UCLASS_BODY()

public:
	void BibelotBeginPlay();

	//~ Begin UActorComponent Interface.
	virtual void OnRegister() override;
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void SendRenderDynamicData_Concurrent() override;
	virtual void CreateRenderState_Concurrent(FRegisterComponentContext* Context) override;
	virtual void ApplyWorldOffset(const FVector& InOffset, bool bWorldShift) override;
	//~ End UActorComponent Interface.

	//~ Begin USceneComponent Interface.
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
	virtual void QuerySupportedSockets(TArray<FComponentSocketDescription>& OutSockets) const override;
	virtual bool HasAnySockets() const override;
	virtual bool DoesSocketExist(FName InSocketName) const override;
	virtual FTransform GetSocketTransform(FName InSocketName, ERelativeTransformSpace TransformSpace = RTS_World) const override;
	virtual void OnVisibilityChanged() override;
	//~ End USceneComponent Interface.

	//~ Begin UPrimitiveComponent Interface.
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	//~ End UPrimitiveComponent Interface.

	//~ Begin UMeshComponent Interface.
	virtual int32 GetNumMaterials() const override;
	//~ End UMeshComponent Interface.

	void TickBody(float DeltaTime);

#if WITH_EDITOR
	virtual void OnUpdateTransform(EUpdateTransformFlags UpdateTransformFlags, ETeleportType Teleport) override;
#endif

	/**
	 *	Should we fix the start to something, or leave it free.
	 *	If false, component transform is just used for initial location of start of cable
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bAttachStart;

	/**
	 *	Should we fix the end to something (using AttachEndTo and EndLocation), or leave it free.
	 *	If false, AttachEndTo and EndLocation are just used for initial location of end of cable
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bAttachEnd;

	/** Actor or Component that the defines the end position of the cable */
	UPROPERTY(EditAnywhere, Category = "Cable")
	FComponentReference AttachEndTo;

	/** Socket name on the AttachEndTo component to attach to */
	UPROPERTY(EditAnywhere, Category = "Cable")
	FName AttachEndToSocketName;

	/** End location of cable, relative to AttachEndTo (or AttachEndToSocketName) if specified, otherwise relative to cable component. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (MakeEditWidget = true))
	FVector EndLocation;

	/** Attaches the end of the cable to a specific Component **/
	UFUNCTION(BlueprintCallable, Category = "Cable")
	void SetAttachEndToComponent(USceneComponent* Component, FName SocketName = NAME_None);

	/** Attaches the end of the cable to a specific Component within an Actor **/
	UFUNCTION(BlueprintCallable, Category = "Cable")
	void SetAttachEndTo(AActor* Actor, FName ComponentProperty, FName SocketName = NAME_None);

	/** Gets the Actor that the cable is attached to **/
	UFUNCTION(BlueprintCallable, Category = "Cable")
	AActor* GetAttachedActor() const;

	/** Gets the specific USceneComponent that the cable is attached to **/
	UFUNCTION(BlueprintCallable, Category = "Cable")
	USceneComponent* GetAttachedComponent() const;

	UFUNCTION(BlueprintCallable, Category = "Cable")
	void FixEndParticleLocation();

	UFUNCTION(BlueprintCallable, Category = "Cable")
	FRotator GetXZCableRotation();

	/** Get array of locations of particles (in world space) making up the cable simulation. */
	UFUNCTION(BlueprintCallable, Category = "Cable")
	void GetCableParticleLocations(TArray<FVector>& Locations);

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float omg = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimize")
	int32 CableFPS = 45;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float AShotChange = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float AVelChangeVelocity = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float AVelChangeNoVelocity = 250.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float grav = 9.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float R = 2.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float b = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float m = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float A = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float k = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float h = 0.025f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float GeneralForceInterpSpeed = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float OmegaLimitMin = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ForcedPendulum")
	float OmegaLimitB = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	FGameplayTagContainer RecoilForceTagsContainer;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	float InterpToSpeed = 1;

	/** Rest length of the cable */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "1000.0"))
	float CableLength;

	/** How many segments the cable has */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cable", meta = (ClampMin = "1", UIMin = "1", UIMax = "20"))
	int32 NumSegments;

	/** Controls the simulation substep time for the cable */
	UPROPERTY(EditAnywhere, AdvancedDisplay, BlueprintReadOnly, Category = "Cable", meta = (ClampMin = "0.005", UIMin = "0.005", UIMax = "0.1"))
	float SubstepTime;

	/** The number of solver iterations controls how 'stiff' the cable is */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1", ClampMax = "16"))
	int32 SolverIterations;

	/** Add stiffness constraints to cable. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Cable")
	bool bEnableStiffness;

	/** When false, will still wait for SubstepTime to elapse before updating, but will only run the cable simulation once using all of accumulated simulation time */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Cable")
	bool bUseSubstepping = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Cable")
	bool bSkipCableUpdateWhenNotVisible = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Cable")
	bool bSkipCableUpdateWhenNotOwnerRecentlyRendered = false;

	/**
	 *	EXPERIMENTAL. Perform sweeps for each cable particle, each substep, to avoid collisions with the world.
	 *	Uses the Collision Preset on the component to determine what is collided with.
	 *	This greatly increases the cost of the cable simulation.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Cable")
	bool bEnableCollision;

	/** If collision is enabled, control how much sliding friction is applied when cable is in contact. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bEnableCollision"))
	float CollisionFriction;

	/** Force vector (world space) applied to all particles in cable. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable Forces")
	FVector CableForce;

	/** Scaling applied to world gravity affecting this cable. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable Forces")
	float CableGravityScale;

	/** How wide the cable geometry is */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable Rendering", meta = (ClampMin = "0.01", UIMin = "0.01", UIMax = "50.0"))
	float CableWidth;

	/** Number of sides of the cable geometry */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cable Rendering", meta = (ClampMin = "1", ClampMax = "16"))
	int32 NumSides;

	/** How many times to repeat the material along the length of the cable */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable Rendering", meta = (UIMin = "0.1", UIMax = "8"))
	float TileMaterial;

private:
	FVector RotateToXAxis(FVector RotVector);
	FVector BackRotateToXAxis(FVector StartAxis, FVector RotVector);
	/** Initialize the particles */
	void InitParticles();
	/** Solve the cable spring constraints */
	void SolveConstraints();
	/** Integrate cable point positions */
	void VerletIntegrate(float InSubstepTime, const FVector& Gravity, float DeltaTime);
	/** Perform collision traces for particles */
	void PerformCableCollision();
	/** Perform a simulation substep */
	void PerformSubstep(float InSubstepTime, const FVector& Gravity, float DeltaTime);
	/** Get start and end position for the cable */
	void GetEndPositions(FVector& OutStartPosition, FVector& OutEndPosition);
	/** Amount of time 'left over' from last tick */
	float TimeRemainder;
	/** Array of cable particles */
	TArray<FBibelotCableParticle> Particles;

	FTimerHandle BibelotBlockTimer;
	TWeakObjectPtr<AIWCharacter> CachedCharacter;
	TWeakObjectPtr<UAbilitySystemComponent> CachedOwnerASC;
	FVector NeedToDetachFromCharacterPos;

	FVector GeneralForce = { 0, 0, 0 };
	float theta = 0;
	float time = 0;
	float FinalB = b;
	float k1, k2, k3, k4, l1, l2, l3, l4;

	FVector OldBibelotLoc;
	FVector VelForce;
	FVector Vel;

	friend class FCableSceneProxy;
};

//#if UE_ENABLE_INCLUDE_ORDER_DEPRECATED_IN_5_2
//#include "CoreMinimal.h"
//#endif
