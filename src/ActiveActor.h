#pragma once

#include "AttackCollision.h"
#include "Havok/Scaling.h"

struct ActiveActor
{
public:
	ActiveActor(RE::ActorHandle a_actorHandle, RE::NiAVObject* a_root, RE::NiAVObject* a_clone, RE::NiAVObject* a_template, uint16_t a_currentCollisionGroup, CollisionLayer a_collisionLayer);

	~ActiveActor();

	void Update(float a_deltaTime);
	bool UpdateClone();
	void LogScales(std::string_view a_reason) const;
	bool CheckInCombat();

	RE::NiAVObject* GetOriginalFromClone(RE::NiAVObject* a_clone);
	RE::hkpRigidBody* GetOriginalFromClone(RE::hkpRigidBody* a_clone);

	void AddHitstop(float a_hitstopLength, bool a_bReceived);
	void UpdateHitstop(float a_deltaTime);
	bool HasHitstop() const;
	bool HasDriveToPoseHitstop() const;
	[[nodiscard]] float GetHitstopMultiplier(float a_deltaTime);
	[[nodiscard]] float GetDriveToPoseHitstopMultiplier(float a_deltaTime);

	RE::ActorHandle actorHandle;
	RE::NiPointer<RE::NiAVObject> root;
	RE::NiPointer<RE::NiAVObject> clone;
	RE::NiPointer<RE::NiAVObject> templateRoot;
	uint16_t collisionGroup;
	CollisionLayer collisionLayer;

	std::unordered_map<RE::NiAVObject*, RE::NiAVObject*> cloneToOriginalMap;
	std::vector<Scaling::Body> cloneBodies;
	std::vector<Scaling::Body> ragdollBodies;
	float ragdollScale = 1.f;
	float loggedRagdollScale = 1.f;

	AttackCollisions attackCollisions;

	std::atomic<float> inCombat = 0.f;

	std::atomic<float> hitstop = 0.f;
	std::atomic<float> driveToPoseHitstop = 0.f;
	std::atomic<uint8_t> receivedHitstopCount = 0;
	std::atomic<float> receivedHitstopCooldown = 0.f;

private:
	void FillCloneMap(RE::NiAVObject* a_clone, RE::NiAVObject* a_original);
	bool UpdateScale();
	bool IsNodeDeadEnd(RE::NiAVObject* a_object);
};
