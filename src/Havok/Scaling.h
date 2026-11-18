#pragma once

namespace Scaling
{
	struct Body
	{
		RE::bhkRigidBody* wrapper = nullptr;
		const RE::hkpShape* templateShape = nullptr;
		std::optional<RE::hkVector4> templateTranslation;
		const RE::hkpShape* installed = nullptr;
		RE::NiAVObject* node = nullptr;
		float templateWorldScale = 1.f;
		float applied = 1.f;
		bool bIsRagdoll = false;
		RE::hkVector4 baseInertiaAndMassInv{};
		RE::hkVector4 baseCenterOfMass{};
		float baseObjectRadius = 0.f;
		float basePenetrationDepth = 0.f;
	};

	[[nodiscard]] float GetShapeSize(const RE::hkpShape* a_shape);
	[[nodiscard]] float GetTemplateWorldScale(const RE::NiAVObject* a_node);
	[[nodiscard]] std::optional<Body> MakeBody(RE::NiAVObject* a_node, RE::NiAVObject* a_templateNode, bool a_bIsRagdoll);

	void Apply(Body& a_body, float a_scale);
	std::pair<std::size_t, std::size_t> ScaleConstraints(std::span<Body> a_ragdollBodies, float a_ratio);

	[[nodiscard]] RE::hkVector4 MapPointBetweenBodies(const RE::hkpRigidBody* a_from, const RE::hkpRigidBody* a_to, const RE::hkVector4& a_point);
}
