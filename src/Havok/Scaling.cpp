#include "Havok/Scaling.h"

#include "Havok/Havok.h"
#include "Offsets.h"
#include "Utils.h"

namespace
{
	using ShapeType = RE::hkpShapeType;

	constexpr std::ptrdiff_t kConvexTransformChild = 0x30;
	constexpr std::ptrdiff_t kConvexTransformTranslation = 0x70;
	constexpr std::ptrdiff_t kConvexTranslateTranslation = 0x40;
	constexpr std::ptrdiff_t kTransformChild = 0x28;
	constexpr std::ptrdiff_t kTransformTranslation = 0x80;
	constexpr std::ptrdiff_t kCompressedMeshScale = 0x120;

	template <class T>
	T& At(const void* a_object, std::ptrdiff_t a_offset)
	{
		return *reinterpret_cast<T*>(reinterpret_cast<std::uintptr_t>(a_object) + a_offset);
	}

	bool HasVtable(const void* a_object, REL::VariantID a_vtable)
	{
		return *reinterpret_cast<const std::uintptr_t*>(a_object) == REL::Relocation<std::uintptr_t>{ a_vtable }.address();
	}

	RE::bhkRigidBodyT* AsRigidBodyT(RE::bhkRigidBody* a_body)
	{
		static REL::Relocation<const RE::NiRTTI*> rtti{ RE::NiRTTI_bhkRigidBodyT };
		return a_body->GetRTTI() == rtti.get() ? reinterpret_cast<RE::bhkRigidBodyT*>(a_body) : nullptr;
	}

	const RE::hkpShape* GetChild(const RE::hkpShape* a_shape)
	{
		switch (a_shape->type) {
		case ShapeType::kConvexTransform:
		case ShapeType::kConvexTranslate:
			return At<const RE::hkpShape*>(a_shape, kConvexTransformChild);
		case ShapeType::kTransform:
			return At<const RE::hkpShape*>(a_shape, kTransformChild);
		case ShapeType::kMOPP:
			return static_cast<const RE::hkpMoppBvTreeShape*>(a_shape)->child.childShape;
		default:
			return nullptr;
		}
	}

	bool Matches(const RE::hkpShape* a_shape, const RE::hkpShape* a_template)
	{
		if (!a_shape || !a_template || a_shape->type != a_template->type) {
			return false;
		}

		switch (a_shape->type) {
		case ShapeType::kSphere:
		case ShapeType::kCapsule:
		case ShapeType::kBox:
			return true;
		case ShapeType::kConvexVertices:
			{
				auto shape = static_cast<const RE::hkpConvexVerticesShape*>(a_shape);
				auto templateShape = static_cast<const RE::hkpConvexVerticesShape*>(a_template);
				return shape->rotatedVertices.size() == templateShape->rotatedVertices.size() && shape->planeEquations.size() == templateShape->planeEquations.size();
			}
		case ShapeType::kList:
			{
				auto& children = static_cast<const RE::hkpListShape*>(a_shape)->childInfo;
				auto& templateChildren = static_cast<const RE::hkpListShape*>(a_template)->childInfo;
				if (children.size() != templateChildren.size()) {
					return false;
				}
				for (int i = 0; i < children.size(); ++i) {
					if (!Matches(children[i].shape, templateChildren[i].shape)) {
						return false;
					}
				}
				return true;
			}
		case ShapeType::kConvexTransform:
		case ShapeType::kConvexTranslate:
		case ShapeType::kTransform:
			return Matches(GetChild(a_shape), GetChild(a_template));
		case ShapeType::kMOPP:
			return HasVtable(a_shape, RE::VTABLE_hkScaledMoppBvTreeShape[0]) && HasVtable(a_template, RE::VTABLE_hkScaledMoppBvTreeShape[0]) && Matches(GetChild(a_shape), GetChild(a_template));
		case ShapeType::kCompressedMesh:
			return HasVtable(a_shape, RE::VTABLE___hkpCompressedMeshShapeShared[0]) && HasVtable(a_template, RE::VTABLE___hkpCompressedMeshShapeShared[0]);
		default:
			return false;
		}
	}

	void Write(const RE::hkpShape* a_shape, const RE::hkpShape* a_template, float a_scale)
	{
		auto shape = const_cast<RE::hkpShape*>(a_shape);

		switch (a_shape->type) {
		case ShapeType::kSphere:
			static_cast<RE::hkpSphereShape*>(shape)->radius = static_cast<const RE::hkpSphereShape*>(a_template)->radius * a_scale;
			break;
		case ShapeType::kCapsule:
			{
				auto capsule = static_cast<RE::hkpCapsuleShape*>(shape);
				auto templateCapsule = static_cast<const RE::hkpCapsuleShape*>(a_template);
				capsule->radius = templateCapsule->radius * a_scale;
				capsule->vertexA = templateCapsule->vertexA * a_scale;
				capsule->vertexB = templateCapsule->vertexB * a_scale;
				break;
			}
		case ShapeType::kBox:
			static_cast<RE::hkpBoxShape*>(shape)->halfExtents = static_cast<const RE::hkpBoxShape*>(a_template)->halfExtents * a_scale;
			break;
		case ShapeType::kConvexVertices:
			{
				auto convex = static_cast<RE::hkpConvexVerticesShape*>(shape);
				auto templateConvex = static_cast<const RE::hkpConvexVerticesShape*>(a_template);
				for (int i = 0; i < convex->rotatedVertices.size(); ++i) {
					auto& points = convex->rotatedVertices[i];
					auto& templatePoints = templateConvex->rotatedVertices[i];
					points.x = templatePoints.x * a_scale;
					points.y = templatePoints.y * a_scale;
					points.z = templatePoints.z * a_scale;
				}
				for (int i = 0; i < convex->planeEquations.size(); ++i) {
					convex->planeEquations[i] = templateConvex->planeEquations[i];
					convex->planeEquations[i].quad.m128_f32[3] *= a_scale;
				}
				convex->aabbHalfExtents = templateConvex->aabbHalfExtents * a_scale;
				convex->aabbCenter = templateConvex->aabbCenter * a_scale;
				break;
			}
		case ShapeType::kList:
			{
				auto list = static_cast<RE::hkpListShape*>(shape);
				auto templateList = static_cast<const RE::hkpListShape*>(a_template);
				for (int i = 0; i < list->childInfo.size(); ++i) {
					Write(list->childInfo[i].shape, templateList->childInfo[i].shape, a_scale);
				}
				list->aabbHalfExtents = templateList->aabbHalfExtents * a_scale;
				list->aabbCenter = templateList->aabbCenter * a_scale;
				break;
			}
		case ShapeType::kConvexTransform:
		case ShapeType::kConvexTranslate:
			{
				const auto offset = a_shape->type == ShapeType::kConvexTransform ? kConvexTransformTranslation : kConvexTranslateTranslation;
				At<RE::hkVector4>(shape, offset) = At<RE::hkVector4>(a_template, offset) * a_scale;
				Write(GetChild(a_shape), GetChild(a_template), a_scale);
				static_cast<RE::hkpConvexShape*>(shape)->radius = static_cast<const RE::hkpConvexShape*>(GetChild(a_shape))->radius;
				break;
			}
		case ShapeType::kTransform:
			At<RE::hkVector4>(shape, kTransformTranslation) = At<RE::hkVector4>(a_template, kTransformTranslation) * a_scale;
			Write(GetChild(a_shape), GetChild(a_template), a_scale);
			break;
		case ShapeType::kMOPP:
			{
				auto mopp = static_cast<RE::hkpMoppBvTreeShape*>(shape);
				RE::hkVector4 codeInfo = static_cast<const RE::hkpMoppBvTreeShape*>(a_template)->code->info.offset;
				codeInfo.quad.m128_f32[0] *= a_scale;
				codeInfo.quad.m128_f32[1] *= a_scale;
				codeInfo.quad.m128_f32[2] *= a_scale;
				codeInfo.quad.m128_f32[3] /= a_scale;
				const_cast<RE::hkpMoppCode*>(mopp->code)->info.offset = codeInfo;
				mopp->codeInfoCopy = codeInfo;
				Write(GetChild(a_shape), GetChild(a_template), a_scale);
				break;
			}
		case ShapeType::kCompressedMesh:
			{
				auto mesh = static_cast<RE::hkpCompressedMeshShape*>(shape);
				auto templateMesh = static_cast<const RE::hkpCompressedMeshShape*>(a_template);
				At<RE::hkVector4>(mesh, kCompressedMeshScale) = At<RE::hkVector4>(templateMesh, kCompressedMeshScale) * a_scale;
				const RE::hkVector4 center = (templateMesh->bounds.min + templateMesh->bounds.max) * 0.5f * a_scale;
				const RE::hkVector4 radius{ templateMesh->radius };
				const RE::hkVector4 halfExtents = ((templateMesh->bounds.max - templateMesh->bounds.min) * 0.5f - radius) * a_scale + radius;
				mesh->bounds.min = center - halfExtents;
				mesh->bounds.max = center + halfExtents;
				break;
			}
		default:
			break;
		}
	}

	class ShapeScaler : public RE::hkReferencedObject
	{
	public:
		ShapeScaler(const RE::hkpShape* a_template, float a_scale) :
			templateShape(a_template), scale(a_scale) {}

		virtual void ModifyShape(RE::hkpShape* a_shape) { Write(a_shape, templateShape, scale); }

	private:
		const RE::hkpShape* templateShape;
		float scale;
	};

	void SwapShape(RE::hkpRigidBody* a_body, const RE::hkpShape* a_shape)
	{
		RE::NiPointer<RE::bhkShape> oldWrapper{ a_body->collidable.shape ? a_body->collidable.shape->userData : nullptr };
		if (a_shape->userData) {
			a_shape->userData->IncRefCount();
		}
		a_body->SetShape(a_shape);
		if (oldWrapper) {
			oldWrapper->DecRefCount();
		}
	}

	const RE::hkpShape* Own(RE::hkpRigidBody* a_body, const RE::hkpShape* a_template, float a_scale)
	{
		if (!a_template->userData) {
			return nullptr;
		}

		RE::NiPointer<RE::bhkShape> wrapper{ Utils::Clone<RE::bhkShape>(a_template->userData, { a_scale, a_scale, a_scale }) };
		auto shape = wrapper ? static_cast<const RE::hkpShape*>(wrapper->referencedObject.get()) : nullptr;
		if (!Matches(shape, a_template)) {
			return nullptr;
		}

		SwapShape(a_body, shape);
		return shape;
	}

	const RE::hkpMotion& GetDynamicMotion(const RE::hkpRigidBody* a_body)
	{
		const auto saved = a_body->motion.savedMotion;
		return a_body->motion.inertiaAndMassInv.quad.m128_f32[3] <= 0.f && saved ? *saved : a_body->motion;
	}

	void WriteMotion(RE::hkpMotion& a_motion, const Scaling::Body& a_body, float a_scale)
	{
		auto& inertiaAndMassInv = a_motion.inertiaAndMassInv.quad.m128_f32;
		const auto& base = a_body.baseInertiaAndMassInv.quad.m128_f32;
		if (inertiaAndMassInv[3] > 0.f && base[3] > 0.f) {
			const float scaleCubed = a_scale * a_scale * a_scale;
			inertiaAndMassInv[0] = base[0] / scaleCubed;
			inertiaAndMassInv[1] = base[1] / scaleCubed;
			inertiaAndMassInv[2] = base[2] / scaleCubed;
			inertiaAndMassInv[3] = base[3] / a_scale;
		}
		a_motion.motionState.objectRadius = a_body.baseObjectRadius * a_scale;
		a_motion.SetCenterOfMassInLocal(a_body.baseCenterOfMass * a_scale);
	}

	template <class T>
	T* CopyConstraintData(const T* a_data)
	{
		const auto size = std::max<std::int32_t>(a_data->GetAllocatedSize(), sizeof(T));
		auto copy = static_cast<T*>(hkHeapAlloc(size));
		std::memcpy(copy, a_data, size);
		copy->memSizeAndFlags = static_cast<std::uint16_t>(size);
		copy->referenceCount = 1;
		return copy;
	}

	RE::hkpConstraintData* OwnConstraintData(const RE::hkpConstraintData* a_data)
	{
		if (a_data->GetType() == RE::hkpConstraintData::ConstraintType::kRagdoll) {
			auto copy = CopyConstraintData(static_cast<const RE::hkpRagdollConstraintData*>(a_data));
			for (auto motor : copy->atoms.ragdollMotors.motors) {
				if (motor) {
					motor->AddReference();
				}
			}
			return copy;
		}

		auto copy = CopyConstraintData(static_cast<const RE::hkpLimitedHingeConstraintData*>(a_data));
		if (copy->atoms.angMotor.motor) {
			copy->atoms.angMotor.motor->AddReference();
		}
		return copy;
	}

	RE::hkpSetLocalTransformsConstraintAtom& GetTransforms(RE::hkpConstraintData* a_data)
	{
		if (a_data->GetType() == RE::hkpConstraintData::ConstraintType::kRagdoll) {
			return static_cast<RE::hkpRagdollConstraintData*>(a_data)->atoms.transforms;
		}
		return static_cast<RE::hkpLimitedHingeConstraintData*>(a_data)->atoms.transforms;
	}

	bool ScaleConstraint(RE::bhkConstraint* a_constraint, RE::hkpConstraintInstance* a_instance, float a_ratio)
	{
		const auto type = a_instance->data->GetType();
		if (type != RE::hkpConstraintData::ConstraintType::kRagdoll && type != RE::hkpConstraintData::ConstraintType::kLimitedHinge) {
			static std::unordered_set<std::int32_t> logged;
			if (logged.insert(type).second) {
				logger::info("Scaling: unsupported constraint type {}", type);
			}
			return false;
		}

		if (a_instance->data->GetReferenceCount() > 1) {
			auto world = a_constraint->GetWorld2();
			if (world) {
				a_constraint->RemoveFromCurrentWorld();
			}
			auto shared = a_instance->data;
			a_instance->data = OwnConstraintData(shared);
			shared->RemoveReference();
			if (world) {
				a_constraint->MoveToWorld(reinterpret_cast<RE::bhkWorld*>(world->userData));
			}
		}

		auto& transforms = GetTransforms(a_instance->data);
		transforms.transformA.translation = transforms.transformA.translation * a_ratio;
		transforms.transformB.translation = transforms.transformB.translation * a_ratio;
		return true;
	}
}

namespace Scaling
{
	float GetShapeSize(const RE::hkpShape* a_shape)
	{
		if (!a_shape) {
			return 0.f;
		}

		switch (a_shape->type) {
		case ShapeType::kSphere:
		case ShapeType::kCapsule:
			return static_cast<const RE::hkpConvexShape*>(a_shape)->radius;
		case ShapeType::kBox:
			return static_cast<const RE::hkpBoxShape*>(a_shape)->halfExtents.Length3();
		case ShapeType::kConvexVertices:
			return static_cast<const RE::hkpConvexVerticesShape*>(a_shape)->aabbHalfExtents.Length3();
		case ShapeType::kList:
			return static_cast<const RE::hkpListShape*>(a_shape)->aabbHalfExtents.Length3();
		case ShapeType::kCompressedMesh:
			return HasVtable(a_shape, RE::VTABLE___hkpCompressedMeshShapeShared[0]) ? At<RE::hkVector4>(a_shape, kCompressedMeshScale).quad.m128_f32[0] : 0.f;
		case ShapeType::kConvexTransform:
		case ShapeType::kConvexTranslate:
		case ShapeType::kTransform:
		case ShapeType::kMOPP:
			return GetShapeSize(GetChild(a_shape));
		default:
			return 0.f;
		}
	}

	float GetTemplateWorldScale(const RE::NiAVObject* a_node)
	{
		float scale = 1.f;
		for (auto node = a_node; node && node->parent; node = node->parent) {
			scale *= node->local.scale;
		}
		return scale;
	}

	std::optional<Body> MakeBody(RE::NiAVObject* a_node, RE::NiAVObject* a_templateNode, bool a_bIsRagdoll)
	{
		auto wrapper = Utils::GetRigidBody(a_node);
		auto templateWrapper = Utils::GetRigidBody(a_templateNode);
		auto body = wrapper ? wrapper->GetRigidBody() : nullptr;
		auto templateBody = templateWrapper ? templateWrapper->GetRigidBody() : nullptr;
		if (!body || !templateBody) {
			return std::nullopt;
		}

		auto templateShape = templateBody->collidable.shape;
		if (!Matches(templateShape, templateShape)) {
			static std::unordered_set<std::string> logged;
			if (logged.emplace(a_node->name.c_str()).second) {
				logger::info("Scaling: unsupported shape type {} on {}", static_cast<int>(templateShape ? templateShape->type : ShapeType::kInvalid), a_node->name.c_str());
			}
			return std::nullopt;
		}

		Body ret{ .wrapper = wrapper, .templateShape = templateShape, .bIsRagdoll = a_bIsRagdoll };

		auto templateT = AsRigidBodyT(templateWrapper);
		if (templateT && AsRigidBodyT(wrapper)) {
			ret.templateTranslation = templateT->translation;
		}

		const float templateSize = GetShapeSize(templateShape);
		const float size = GetShapeSize(body->collidable.shape);
		if (templateSize > 0.f && size > 0.f) {
			ret.applied = size / templateSize;
		}

		if (a_bIsRagdoll) {
			const auto& motion = GetDynamicMotion(body);
			const float scale = ret.applied;
			const auto& inertiaAndMassInv = motion.inertiaAndMassInv.quad.m128_f32;
			const float scaleCubed = scale * scale * scale;
			ret.baseInertiaAndMassInv = RE::hkVector4(inertiaAndMassInv[0] * scaleCubed, inertiaAndMassInv[1] * scaleCubed, inertiaAndMassInv[2] * scaleCubed, inertiaAndMassInv[3] * scale);
			ret.baseCenterOfMass = motion.motionState.sweptTransform.centerOfMassLocal * (1.f / scale);
			ret.baseObjectRadius = motion.motionState.objectRadius / scale;
			ret.basePenetrationDepth = body->collidable.allowedPenetrationDepth / scale;
		}

		return ret;
	}

	void Apply(Body& a_body, float a_scale)
	{
		auto body = a_body.wrapper ? a_body.wrapper->GetRigidBody() : nullptr;
		if (!body || std::abs(a_scale - a_body.applied) < 1e-4f) {
			return;
		}

		const bool bIsUnit = std::abs(a_scale - 1.f) < 1e-4f;
		if (bIsUnit) {
			a_scale = 1.f;
		}

		if (a_body.installed && body->collidable.shape == a_body.installed) {
			ShapeScaler scaler{ a_body.templateShape, a_scale };
			body->UpdateShape(reinterpret_cast<RE::hkpShapeModifier*>(&scaler));
		} else if (bIsUnit) {
			if (body->collidable.shape != a_body.templateShape) {
				SwapShape(body, a_body.templateShape);
			}
		} else {
			a_body.installed = Own(body, a_body.templateShape, a_scale);
			if (!a_body.installed) {
				a_body.wrapper = nullptr;
				return;
			}
		}

		if (a_body.templateTranslation) {
			AsRigidBodyT(a_body.wrapper)->translation = *a_body.templateTranslation * a_scale;
		}

		if (a_body.bIsRagdoll) {
			WriteMotion(body->motion, a_body, a_scale);
			if (auto saved = body->motion.savedMotion) {
				WriteMotion(*saved, a_body, a_scale);
			}
			body->collidable.allowedPenetrationDepth = a_body.basePenetrationDepth * a_scale;
		}

		a_body.applied = a_scale;
	}

	std::pair<std::size_t, std::size_t> ScaleConstraints(std::span<Body> a_ragdollBodies, float a_ratio)
	{
		std::size_t scaled = 0;
		std::unordered_set<RE::hkpConstraintInstance*> visited;
		for (auto& body : a_ragdollBodies) {
			if (!body.wrapper) {
				continue;
			}
			for (auto entry : body.wrapper->unk28) {
				auto constraint = static_cast<RE::bhkConstraint*>(entry);
				auto instance = constraint ? static_cast<RE::hkpConstraintInstance*>(constraint->referencedObject.get()) : nullptr;
				if (instance && instance->data && visited.insert(instance).second && ScaleConstraint(constraint, instance, a_ratio)) {
					++scaled;
				}
			}
		}
		return { scaled, visited.size() };
	}

	RE::hkVector4 MapPointBetweenBodies(const RE::hkpRigidBody* a_from, const RE::hkpRigidBody* a_to, const RE::hkVector4& a_point)
	{
		const float fromSize = GetShapeSize(a_from->collidable.shape);
		const float toSize = GetShapeSize(a_to->collidable.shape);
		if (fromSize <= 0.f || toSize <= 0.f || std::abs(toSize / fromSize - 1.f) < 0.01f) {
			return a_point;
		}

		RE::hkVector4 local;
		hkVector4_setTransformedInversePos(local, a_from->motion.motionState.transform, a_point);
		local = local * (toSize / fromSize);

		RE::hkVector4 ret;
		hkVector4_setTransformedPos(ret, a_to->motion.motionState.transform, local);
		return ret;
	}
}
