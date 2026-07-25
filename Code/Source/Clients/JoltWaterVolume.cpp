#include <Clients/JoltWaterVolume.h>

#include <AzCore/Interface/Interface.h>
#include <AzCore/std/parallel/lock.h>

#include <JoltPhysics/JoltPhysicsBus.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltBuoyancy
{
    namespace
    {
        JPH::Vec3 ToJolt(const AZ::Vector3& v)
        {
            return JPH::Vec3(v.GetX(), v.GetY(), v.GetZ());
        }

        JPH::RVec3 ToJoltR(const AZ::Vector3& v)
        {
            return JPH::RVec3(v.GetX(), v.GetY(), v.GetZ());
        }
    }

    JoltWaterVolume::~JoltWaterVolume()
    {
        Detach();
    }

    bool JoltWaterVolume::Attach(AzPhysics::SceneHandle sceneHandle)
    {
        Detach();

        // Asking the Jolt gem rather than casting the scene's native pointer: it answers
        // with null when the scene belongs to a different physics backend, so this gem is
        // inert rather than dangerous in a project that is not running Jolt.
        JPH::PhysicsSystem* physicsSystem = nullptr;
        JoltPhysics::JoltPhysicsSystemRequestBus::BroadcastResult(
            physicsSystem, &JoltPhysics::JoltPhysicsSystemRequests::GetNativePhysicsSystem, sceneHandle);

        AZ_WarningOnce("JoltBuoyancy", physicsSystem != nullptr,
            "No Jolt physics scene is available, so water volumes do nothing. This gem requires the JoltPhysics "
            "gem to be the active physics backend.");

        return AttachToPhysicsSystem(physicsSystem);
    }

    bool JoltWaterVolume::AttachToPhysicsSystem(JPH::PhysicsSystem* physicsSystem)
    {
        Detach();

        if (!physicsSystem)
        {
            return false;
        }

        m_physicsSystem = physicsSystem;
        m_physicsSystem->AddStepListener(this);
        return true;
    }

    void JoltWaterVolume::Detach()
    {
        if (m_physicsSystem)
        {
            m_physicsSystem->RemoveStepListener(this);
            m_physicsSystem = nullptr;
        }
        m_submergedBodyCount.store(0, AZStd::memory_order_relaxed);
    }

    void JoltWaterVolume::SetVolume(const AZ::Transform& worldTransform, const AZ::Vector3& dimensions)
    {
        AZStd::lock_guard lock(m_settingsMutex);
        m_worldTransform = worldTransform;
        m_dimensions = dimensions.GetMax(AZ::Vector3(0.001f));

        // The broadphase is queried with an axis-aligned box, so a rotated volume needs
        // the bounds of its corners rather than of its dimensions.
        const AZ::Vector3 halfExtents = m_dimensions * 0.5f;
        m_worldBounds = AZ::Aabb::CreateNull();
        for (int corner = 0; corner < 8; ++corner)
        {
            const AZ::Vector3 localCorner(
                (corner & 1) ? halfExtents.GetX() : -halfExtents.GetX(),
                (corner & 2) ? halfExtents.GetY() : -halfExtents.GetY(),
                (corner & 4) ? halfExtents.GetZ() : -halfExtents.GetZ());
            m_worldBounds.AddPoint(m_worldTransform.TransformPoint(localCorner));
        }
    }

    void JoltWaterVolume::SetSettings(const JoltWaterVolumeSettings& settings)
    {
        AZStd::lock_guard lock(m_settingsMutex);
        m_settings = settings;
    }

    JoltWaterVolumeSettings JoltWaterVolume::GetSettings() const
    {
        AZStd::lock_guard lock(m_settingsMutex);
        return m_settings;
    }

    void JoltWaterVolume::OnStep(const JPH::PhysicsStepListenerContext& inContext)
    {
        if (!IsEnabled() || inContext.mDeltaTime <= 0.0f)
        {
            m_submergedBodyCount.store(0, AZStd::memory_order_relaxed);
            return;
        }

        // Copy what gameplay may be writing, so the rest of the step works from a
        // consistent snapshot.
        JoltWaterVolumeSettings settings;
        AZ::Transform worldTransform;
        AZ::Vector3 dimensions;
        AZ::Aabb worldBounds;
        {
            AZStd::lock_guard lock(m_settingsMutex);
            settings = m_settings;
            worldTransform = m_worldTransform;
            dimensions = m_dimensions;
            worldBounds = m_worldBounds;
        }

        if (!worldBounds.IsValid())
        {
            return;
        }

        // The surface is the volume's local +Z face, so a tilted volume gives a tilted
        // water surface (a sloped river, say) rather than always a horizontal one.
        const AZ::Vector3 surfaceNormal = worldTransform.TransformVector(AZ::Vector3::CreateAxisZ()).GetNormalizedSafe();
        const AZ::Vector3 surfacePosition =
            worldTransform.TransformPoint(AZ::Vector3(0.0f, 0.0f, dimensions.GetZ() * 0.5f));

        const JPH::AABox queryBox(ToJolt(worldBounds.GetMin()), ToJolt(worldBounds.GetMax()));
        JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
        m_physicsSystem->GetBroadPhaseQuery().CollideAABox(queryBox, collector);

        const JPH::Vec3 gravity = m_physicsSystem->GetGravity();

        // Step listeners run with every body mutex already held, so bodies are read
        // through the no-lock interface; taking a lock here would deadlock.
        const JPH::BodyLockInterface& bodyLockInterface = m_physicsSystem->GetBodyLockInterfaceNoLock();

        int submergedCount = 0;
        for (const JPH::BodyID& bodyId : collector.mHits)
        {
            JPH::Body* body = bodyLockInterface.TryGetBody(bodyId);
            if (!body || !body->IsDynamic() || !body->IsActive())
            {
                continue;
            }

            // Jolt's buoyancy factor is relative: 1 is neutral, above 1 floats. Deriving
            // it from the body's own density is what makes a dense body sink and a light
            // one bob, instead of every body behaving the same way.
            float buoyancy = 1.0f;
            const JPH::Shape* shape = body->GetShape();
            const float shapeVolume = shape ? shape->GetVolume() : 0.0f;
            const float inverseMass = body->GetMotionProperties()->GetInverseMass();
            if (shapeVolume > 0.0f && inverseMass > 0.0f)
            {
                const float bodyDensity = (1.0f / inverseMass) / shapeVolume;
                buoyancy = settings.m_fluidDensity / AZStd::max(bodyDensity, 0.001f);
            }

            if (body->ApplyBuoyancyImpulse(
                    ToJoltR(surfacePosition), ToJolt(surfaceNormal), buoyancy, settings.m_linearDrag,
                    settings.m_angularDrag, ToJolt(settings.m_fluidVelocity), gravity, inContext.mDeltaTime))
            {
                ++submergedCount;
            }
        }

        m_submergedBodyCount.store(submergedCount, AZStd::memory_order_relaxed);
    }

} // namespace JoltBuoyancy
