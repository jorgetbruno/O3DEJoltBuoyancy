// Reflection for the gem's public types and buses.
//
// The buses are reflected to the BehaviorContext as well as the SerializeContext, so Lua
// and Script Canvas can change water at runtime. The JoltPhysics gem reflects every one of
// its gameplay buses and pins them with a script reflection test; this gem follows that.

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <JoltBuoyancy/JoltBuoyancyBus.h>

namespace JoltBuoyancy
{
    //! Forwards notifications to script, so Lua and Script Canvas can react to a splash.
    class JoltWaterVolumeNotificationBusHandler
        : public JoltWaterVolumeNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            JoltWaterVolumeNotificationBusHandler, "{6D2C8B4A-1F3E-4A57-9C8D-2B4E6F1A3C57}", AZ::SystemAllocator,
            OnBodyEnteredWater, OnBodyExitedWater);

        void OnBodyEnteredWater(AZ::EntityId bodyEntityId, float speed) override
        {
            Call(FN_OnBodyEnteredWater, bodyEntityId, speed);
        }

        void OnBodyExitedWater(AZ::EntityId bodyEntityId) override
        {
            Call(FN_OnBodyExitedWater, bodyEntityId);
        }
    };

    void JoltWaterVolumeSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltWaterVolumeSettings>()
                ->Version(2)
                ->Field("FluidDensity", &JoltWaterVolumeSettings::m_fluidDensity)
                ->Field("LinearDrag", &JoltWaterVolumeSettings::m_linearDrag)
                ->Field("AngularDrag", &JoltWaterVolumeSettings::m_angularDrag)
                ->Field("FluidVelocity", &JoltWaterVolumeSettings::m_fluidVelocity)
                ->Field("WavesEnabled", &JoltWaterVolumeSettings::m_wavesEnabled)
                ->Field("WaveAmplitude", &JoltWaterVolumeSettings::m_waveAmplitude)
                ->Field("WaveLength", &JoltWaterVolumeSettings::m_waveLength)
                ->Field("WaveSpeed", &JoltWaterVolumeSettings::m_waveSpeed)
                ->Field("WaveDirection", &JoltWaterVolumeSettings::m_waveDirection)
                ->Field("CollisionGroupId", &JoltWaterVolumeSettings::m_collisionGroupId)
                ->Field("ReportSubmergedFraction", &JoltWaterVolumeSettings::m_reportSubmergedFraction)
                ->Field("Shape", &JoltWaterVolumeSettings::m_shape)
                ->Field("OwnershipHysteresis", &JoltWaterVolumeSettings::m_ownershipHysteresis)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltWaterVolumeSettings>("Water Settings", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltWaterVolumeSettings::m_shape,
                        "Shape", "The region the water fills. A sphere is sized by its X dimension and its surface "
                        "sits at the top of the sphere.")
                        ->EnumAttribute(JoltWaterVolumeShape::Box, "Box")
                        ->EnumAttribute(JoltWaterVolumeShape::Sphere, "Sphere")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_fluidDensity,
                        "Fluid density", "Density of the fluid. A body floats when it is less dense than this and "
                        "sinks when it is denser. Fresh water is 1000.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " kg/m^3")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_linearDrag,
                        "Linear drag", "How strongly the water slows a body down. Higher feels thicker.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_angularDrag,
                        "Angular drag", "How strongly the water damps a body's spin.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_fluidVelocity,
                        "Fluid velocity", "Velocity of the water itself: a current that carries bodies along.")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltWaterVolumeSettings::m_wavesEnabled,
                        "Waves", "Ripple the surface instead of leaving it flat. The wave rides the volume's own "
                        "surface, so a tilted volume gets a tilted moving surface.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_waveAmplitude,
                        "Wave amplitude", "Crest height above the flat surface.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltWaterVolumeSettings::m_wavesEnabled)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_waveLength,
                        "Wave length", "Distance between crests.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltWaterVolumeSettings::m_wavesEnabled)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_waveSpeed,
                        "Wave speed", "How fast crests travel.")
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m/s")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltWaterVolumeSettings::m_wavesEnabled)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_waveDirection,
                        "Wave direction", "Travel direction, in the volume's local XY plane.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltWaterVolumeSettings::m_wavesEnabled)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_collisionGroupId,
                        "Collides with", "Which bodies this water affects. Bodies the group excludes are skipped "
                        "before any impulse is computed.")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltWaterVolumeSettings::m_reportSubmergedFraction,
                        "Report submerged fraction", "Work out how much of each body is under the surface and publish "
                        "it on the bus. Costs an extra pass over each body's shape, so it is off by default.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_ownershipHysteresis,
                        "Ownership hysteresis", "How much deeper an overlapping volume must hold a body before it "
                        "takes it over. Stops a body on the seam between two volumes changing hands repeatedly.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ;
            }
        }

        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<JoltWaterVolumeSettings>("JoltWaterVolumeSettings")
                ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Property("fluidDensity", BehaviorValueProperty(&JoltWaterVolumeSettings::m_fluidDensity))
                ->Property("linearDrag", BehaviorValueProperty(&JoltWaterVolumeSettings::m_linearDrag))
                ->Property("angularDrag", BehaviorValueProperty(&JoltWaterVolumeSettings::m_angularDrag))
                ->Property("fluidVelocity", BehaviorValueProperty(&JoltWaterVolumeSettings::m_fluidVelocity))
                ->Property("wavesEnabled", BehaviorValueProperty(&JoltWaterVolumeSettings::m_wavesEnabled))
                ->Property("waveAmplitude", BehaviorValueProperty(&JoltWaterVolumeSettings::m_waveAmplitude))
                ->Property("waveLength", BehaviorValueProperty(&JoltWaterVolumeSettings::m_waveLength))
                ->Property("waveSpeed", BehaviorValueProperty(&JoltWaterVolumeSettings::m_waveSpeed))
                ->Property("waveDirection", BehaviorValueProperty(&JoltWaterVolumeSettings::m_waveDirection))
                ->Property("reportSubmergedFraction", BehaviorValueProperty(&JoltWaterVolumeSettings::m_reportSubmergedFraction))
                ;

            // Exposed as plain constants: script has no enum type of its own, and these are
            // what SetBuoyancyMode expects.
            behaviorContext->Enum<static_cast<int>(JoltBuoyancyMode::Automatic)>("JoltBuoyancyMode_Automatic");
            behaviorContext->Enum<static_cast<int>(JoltBuoyancyMode::Explicit)>("JoltBuoyancyMode_Explicit");
            behaviorContext->Enum<static_cast<int>(JoltWaterVolumeShape::Box)>("JoltWaterVolumeShape_Box");
            behaviorContext->Enum<static_cast<int>(JoltWaterVolumeShape::Sphere)>("JoltWaterVolumeShape_Sphere");

            behaviorContext->EBus<JoltWaterVolumeRequestBus>("JoltWaterVolumeRequestBus")
                ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "physics")
                ->Event("SetFluidDensity", &JoltWaterVolumeRequestBus::Events::SetFluidDensity)
                ->Event("GetFluidDensity", &JoltWaterVolumeRequestBus::Events::GetFluidDensity)
                ->Event("SetLinearDrag", &JoltWaterVolumeRequestBus::Events::SetLinearDrag)
                ->Event("GetLinearDrag", &JoltWaterVolumeRequestBus::Events::GetLinearDrag)
                ->Event("SetAngularDrag", &JoltWaterVolumeRequestBus::Events::SetAngularDrag)
                ->Event("GetAngularDrag", &JoltWaterVolumeRequestBus::Events::GetAngularDrag)
                ->Event("SetFluidVelocity", &JoltWaterVolumeRequestBus::Events::SetFluidVelocity)
                ->Event("GetFluidVelocity", &JoltWaterVolumeRequestBus::Events::GetFluidVelocity)
                ->Event("SetDimensions", &JoltWaterVolumeRequestBus::Events::SetDimensions)
                ->Event("GetDimensions", &JoltWaterVolumeRequestBus::Events::GetDimensions)
                ->Event("SetWaterSettings", &JoltWaterVolumeRequestBus::Events::SetWaterSettings)
                ->Event("GetWaterSettings", &JoltWaterVolumeRequestBus::Events::GetWaterSettings)
                ->Event("SetWavesEnabled", &JoltWaterVolumeRequestBus::Events::SetWavesEnabled)
                ->Event("GetWavesEnabled", &JoltWaterVolumeRequestBus::Events::GetWavesEnabled)
                ->Event("SetWaveAmplitude", &JoltWaterVolumeRequestBus::Events::SetWaveAmplitude)
                ->Event("GetWaveAmplitude", &JoltWaterVolumeRequestBus::Events::GetWaveAmplitude)
                ->Event("SetWaveLength", &JoltWaterVolumeRequestBus::Events::SetWaveLength)
                ->Event("GetWaveLength", &JoltWaterVolumeRequestBus::Events::GetWaveLength)
                ->Event("SetWaveSpeed", &JoltWaterVolumeRequestBus::Events::SetWaveSpeed)
                ->Event("GetWaveSpeed", &JoltWaterVolumeRequestBus::Events::GetWaveSpeed)
                ->Event("SetWaveDirection", &JoltWaterVolumeRequestBus::Events::SetWaveDirection)
                ->Event("GetWaveDirection", &JoltWaterVolumeRequestBus::Events::GetWaveDirection)
                ->Event("SetEnabled", &JoltWaterVolumeRequestBus::Events::SetEnabled)
                ->Event("IsEnabled", &JoltWaterVolumeRequestBus::Events::IsEnabled)
                ->Event("GetSubmergedBodyCount", &JoltWaterVolumeRequestBus::Events::GetSubmergedBodyCount)
                ->Event("GetSubmergedFraction", &JoltWaterVolumeRequestBus::Events::GetSubmergedFraction)
                ->Event("IsPointUnderwater", &JoltWaterVolumeRequestBus::Events::IsPointUnderwater)
                ->Event("GetSurfacePositionAt", &JoltWaterVolumeRequestBus::Events::GetSurfacePositionAt)
                ->Event("GetSurfaceNormalAt", &JoltWaterVolumeRequestBus::Events::GetSurfaceNormalAt)
                ->Event("GetDepthAt", &JoltWaterVolumeRequestBus::Events::GetDepthAt)
                ;

            behaviorContext->EBus<JoltWaterVolumeNotificationBus>("JoltWaterVolumeNotificationBus")
                ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "physics")
                ->Handler<JoltWaterVolumeNotificationBusHandler>()
                ;

            behaviorContext->EBus<JoltBuoyancyOverrideRequestBus>("JoltBuoyancyOverrideRequestBus")
                ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "physics")
                ->Event("SetExcludedFromWater", &JoltBuoyancyOverrideRequestBus::Events::SetExcludedFromWater)
                ->Event("IsExcludedFromWater", &JoltBuoyancyOverrideRequestBus::Events::IsExcludedFromWater)
                ->Event("SetBuoyancyFactor", &JoltBuoyancyOverrideRequestBus::Events::SetBuoyancyFactor)
                ->Event("GetBuoyancyFactor", &JoltBuoyancyOverrideRequestBus::Events::GetBuoyancyFactor)
                ->Event("SetBuoyancyMode", &JoltBuoyancyOverrideRequestBus::Events::SetBuoyancyMode)
                ->Event("GetBuoyancyMode", &JoltBuoyancyOverrideRequestBus::Events::GetBuoyancyMode)
                ->Event("SetLinearDragMultiplier", &JoltBuoyancyOverrideRequestBus::Events::SetLinearDragMultiplier)
                ->Event("GetLinearDragMultiplier", &JoltBuoyancyOverrideRequestBus::Events::GetLinearDragMultiplier)
                ->Event("SetAngularDragMultiplier", &JoltBuoyancyOverrideRequestBus::Events::SetAngularDragMultiplier)
                ->Event("GetAngularDragMultiplier", &JoltBuoyancyOverrideRequestBus::Events::GetAngularDragMultiplier)
                ;
        }
    }
} // namespace JoltBuoyancy
