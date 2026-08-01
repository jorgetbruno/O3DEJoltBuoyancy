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

    void JoltWaterSpectrum::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltWaterSpectrum>()
                ->Version(1)
                ->Field("Beaufort", &JoltWaterSpectrum::m_beaufort)
                ->Field("Fetch", &JoltWaterSpectrum::m_fetch)
                ->Field("WindDirection", &JoltWaterSpectrum::m_windDirection)
                ->Field("DirectionalSpread", &JoltWaterSpectrum::m_directionalSpread)
                ->Field("ComponentCount", &JoltWaterSpectrum::m_componentCount)
                ->Field("AmplitudeScale", &JoltWaterSpectrum::m_amplitudeScale)
                ->Field("Steepness", &JoltWaterSpectrum::m_steepness)
                ->Field("SpeedScale", &JoltWaterSpectrum::m_speedScale)
                ->Field("WaterDepth", &JoltWaterSpectrum::m_waterDepth)
                ->Field("Seed", &JoltWaterSpectrum::m_seed)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltWaterSpectrum>("Sea State", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Slider, &JoltWaterSpectrum::m_beaufort,
                        "Beaufort", "0 is glassy calm, 4 a moderate breeze, 8 a gale, 12 a hurricane. One number for "
                        "the whole sea, so a weather change is a single lerp.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 12.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterSpectrum::m_fetch,
                        "Fetch", "How far the wind has blown across open water. A short fetch gives a choppy, "
                        "short-wavelength sea however hard the wind blows.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterSpectrum::m_windDirection,
                        "Wind direction", "In the volume's local XY plane. Waves travel along it.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterSpectrum::m_directionalSpread,
                        "Directional spread", "How far waves fan out either side of the wind. Zero gives parallel "
                        "ridges, which no real sea has.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 3.14f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " rad")
                    ->DataElement(AZ::Edit::UIHandlers::Slider, &JoltWaterSpectrum::m_componentCount,
                        "Components", "How many waves are synthesised. More is smoother and costs linearly on every "
                        "surface sample.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1)
                        ->Attribute(AZ::Edit::Attributes::Max, 32)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterSpectrum::m_amplitudeScale,
                        "Amplitude scale", "Art control on wave height, on top of the physical spectrum.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Slider, &JoltWaterSpectrum::m_steepness,
                        "Steepness", "Sharpens crests and broadens troughs. 1 is the limit before the surface folds "
                        "over itself; the synthesis scales it down if the components together would exceed that.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 1.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterSpectrum::m_speedScale,
                        "Speed scale", "Scales the whole sea's motion without changing its shape. Wave speeds "
                        "themselves come from the dispersion relation and are not authored.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterSpectrum::m_waterDepth,
                        "Water depth", "0 is deep water. A finite depth slows and shortens the longer waves, which "
                        "is swell steepening as it runs into a beach. One depth for the whole volume: real shoaling "
                        "needs a depth that varies with the sea floor.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterSpectrum::m_seed,
                        "Seed", "Makes the sea repeatable run to run.")
                    ;
            }
        }

        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<JoltWaterSpectrum>("JoltWaterSpectrum")
                ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Property("beaufort", BehaviorValueProperty(&JoltWaterSpectrum::m_beaufort))
                ->Property("fetch", BehaviorValueProperty(&JoltWaterSpectrum::m_fetch))
                ->Property("windDirection", BehaviorValueProperty(&JoltWaterSpectrum::m_windDirection))
                ->Property("directionalSpread", BehaviorValueProperty(&JoltWaterSpectrum::m_directionalSpread))
                ->Property("componentCount", BehaviorValueProperty(&JoltWaterSpectrum::m_componentCount))
                ->Property("amplitudeScale", BehaviorValueProperty(&JoltWaterSpectrum::m_amplitudeScale))
                ->Property("steepness", BehaviorValueProperty(&JoltWaterSpectrum::m_steepness))
                ->Property("speedScale", BehaviorValueProperty(&JoltWaterSpectrum::m_speedScale))
                ->Property("waterDepth", BehaviorValueProperty(&JoltWaterSpectrum::m_waterDepth))
                ->Property("seed", BehaviorValueProperty(&JoltWaterSpectrum::m_seed))
                ;
        }
    }

    AZ::u32 JoltWaterVolumeSettings::GetPlaneDepthVisibility() const
    {
        // A box and a sphere have a floor of their own, so showing the field for them
        // would be a control that silently does nothing.
        return m_shape == JoltWaterVolumeShape::Plane
            ? AZ::Edit::PropertyVisibility::Show
            : AZ::Edit::PropertyVisibility::Hide;
    }

    void JoltWaterVolumeSettings::Reflect(AZ::ReflectContext* context)
    {
        JoltWaterSpectrum::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltWaterVolumeSettings>()
                ->Version(3)
                ->Field("FluidDensity", &JoltWaterVolumeSettings::m_fluidDensity)
                ->Field("LinearDrag", &JoltWaterVolumeSettings::m_linearDrag)
                ->Field("AngularDrag", &JoltWaterVolumeSettings::m_angularDrag)
                ->Field("FluidVelocity", &JoltWaterVolumeSettings::m_fluidVelocity)
                ->Field("WavesEnabled", &JoltWaterVolumeSettings::m_wavesEnabled)
                ->Field("Spectrum", &JoltWaterVolumeSettings::m_spectrum)
                ->Field("SurfaceSamplesPerBody", &JoltWaterVolumeSettings::m_surfaceSamplesPerBody)
                ->Field("CollisionGroupId", &JoltWaterVolumeSettings::m_collisionGroupId)
                ->Field("ReportSubmergedFraction", &JoltWaterVolumeSettings::m_reportSubmergedFraction)
                ->Field("Shape", &JoltWaterVolumeSettings::m_shape)
                ->Field("MaxDepth", &JoltWaterVolumeSettings::m_maxDepth)
                ->Field("OwnershipHysteresis", &JoltWaterVolumeSettings::m_ownershipHysteresis)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltWaterVolumeSettings>("Water Settings", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltWaterVolumeSettings::m_shape,
                        "Shape", "The region the water fills. A sphere is sized by its X dimension and its surface "
                        "sits at the top of the sphere. A plane is everything below the surface within the "
                        "horizontal extent, with no floor at the Z dimension.")
                        ->EnumAttribute(JoltWaterVolumeShape::Box, "Box")
                        ->EnumAttribute(JoltWaterVolumeShape::Sphere, "Sphere")
                        ->EnumAttribute(JoltWaterVolumeShape::Plane, "Plane (open water)")
                        // Maximum depth appears and disappears with this, so the whole tree
                        // has to be refreshed rather than just the one widget's value.
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_maxDepth,
                        "Maximum depth", "How far below the surface a plane reaches. It has no floor at the Z "
                        "dimension, but the broadphase is still queried with a finite box and this is where its "
                        "bottom goes - a body below it is not affected at all. The default is deeper than any "
                        "playable world; lower it if the extent is large and many bodies sit below sea level that "
                        "never need to float, since every body inside the box is examined each step.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltWaterVolumeSettings::GetPlaneDepthVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_fluidDensity,
                        "Fluid density", "Density of the fluid. A body floats when it is less dense than this and "
                        "sinks when it is denser. Fresh water is 1000.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " kg/m^3")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_linearDrag,
                        "Linear drag", "How strongly the water slows a body down. Higher feels thicker. A drag "
                        "coefficient, not a fraction of velocity: the force is quadratic in speed and scales with "
                        "how much of the body is under the surface, so doubling this does not halve the time to "
                        "stop. 0.5 is a reasonable starting point.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_angularDrag,
                        "Angular drag", "How strongly the water damps a body's spin.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_fluidVelocity,
                        "Fluid velocity", "Velocity of the water itself: a current that carries bodies along.")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltWaterVolumeSettings::m_wavesEnabled,
                        "Waves", "Ripple the surface instead of leaving it flat. The wave rides the volume's own "
                        "surface, so a tilted volume gets a tilted moving surface.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_spectrum,
                        "Sea state", "The spectrum the waves are synthesised from")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltWaterVolumeSettings::m_wavesEnabled)
                    ->DataElement(AZ::Edit::UIHandlers::Slider, &JoltWaterVolumeSettings::m_surfaceSamplesPerBody,
                        "Surface samples per body", "How many points across a body the surface is sampled at before "
                        "fitting a plane. One means a hull only sees the water under its centre, so a boat long "
                        "enough to straddle a crest never pitches on it.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1)
                        ->Attribute(AZ::Edit::Attributes::Max, 16)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltWaterVolumeSettings::m_wavesEnabled)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_collisionGroupId,
                        "Collides with", "Which bodies this water affects. Bodies the group excludes are skipped "
                        "before any impulse is computed.")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltWaterVolumeSettings::m_reportSubmergedFraction,
                        "Report submerged fraction", "Publish how much of each body is under the surface on the bus. "
                        "The number is worked out for every body anyway, because the drag needs it, so this only "
                        "decides whether it is kept.")
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
                ->Property("spectrum", BehaviorValueProperty(&JoltWaterVolumeSettings::m_spectrum))
                ->Property("surfaceSamplesPerBody", BehaviorValueProperty(&JoltWaterVolumeSettings::m_surfaceSamplesPerBody))
                ->Property("reportSubmergedFraction", BehaviorValueProperty(&JoltWaterVolumeSettings::m_reportSubmergedFraction))
                ->Property("maxDepth", BehaviorValueProperty(&JoltWaterVolumeSettings::m_maxDepth))
                ;

            // Exposed as plain constants: script has no enum type of its own, and these are
            // what SetBuoyancyMode expects.
            behaviorContext->Enum<static_cast<int>(JoltBuoyancyMode::Automatic)>("JoltBuoyancyMode_Automatic");
            behaviorContext->Enum<static_cast<int>(JoltBuoyancyMode::Explicit)>("JoltBuoyancyMode_Explicit");
            behaviorContext->Enum<static_cast<int>(JoltWaterVolumeShape::Box)>("JoltWaterVolumeShape_Box");
            behaviorContext->Enum<static_cast<int>(JoltWaterVolumeShape::Sphere)>("JoltWaterVolumeShape_Sphere");
            behaviorContext->Enum<static_cast<int>(JoltWaterVolumeShape::Plane)>("JoltWaterVolumeShape_Plane");

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
                ->Event("SetSpectrum", &JoltWaterVolumeRequestBus::Events::SetSpectrum)
                ->Event("GetSpectrum", &JoltWaterVolumeRequestBus::Events::GetSpectrum)
                ->Event("SetSeaState", &JoltWaterVolumeRequestBus::Events::SetSeaState)
                ->Event("GetSeaState", &JoltWaterVolumeRequestBus::Events::GetSeaState)
                ->Event("SetWindDirection", &JoltWaterVolumeRequestBus::Events::SetWindDirection)
                ->Event("GetWindDirection", &JoltWaterVolumeRequestBus::Events::GetWindDirection)
                ->Event("GetSignificantWaveHeight", &JoltWaterVolumeRequestBus::Events::GetSignificantWaveHeight)
                ->Event("GetWaterVelocityAt", &JoltWaterVolumeRequestBus::Events::GetWaterVelocityAt)
                ->Event("SetEnabled", &JoltWaterVolumeRequestBus::Events::SetEnabled)
                ->Event("IsEnabled", &JoltWaterVolumeRequestBus::Events::IsEnabled)
                ->Event("GetSubmergedBodyCount", &JoltWaterVolumeRequestBus::Events::GetSubmergedBodyCount)
                ->Event("GetSubmergedFraction", &JoltWaterVolumeRequestBus::Events::GetSubmergedFraction)
                ->Event("IsPointUnderwater", &JoltWaterVolumeRequestBus::Events::IsPointUnderwater)
                ->Event("GetSurfacePositionAt", &JoltWaterVolumeRequestBus::Events::GetSurfacePositionAt)
                ->Event("GetSurfaceNormalAt", &JoltWaterVolumeRequestBus::Events::GetSurfaceNormalAt)
                ->Event("GetDepthAt", &JoltWaterVolumeRequestBus::Events::GetDepthAt)
                ->Event("GetFoamAt", &JoltWaterVolumeRequestBus::Events::GetFoamAt)
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
                ->Event("SetDirectionalDrag", &JoltBuoyancyOverrideRequestBus::Events::SetDirectionalDrag)
                ->Event("GetDirectionalDrag", &JoltBuoyancyOverrideRequestBus::Events::GetDirectionalDrag)
                ->Event("SetAddedMass", &JoltBuoyancyOverrideRequestBus::Events::SetAddedMass)
                ->Event("GetAddedMass", &JoltBuoyancyOverrideRequestBus::Events::GetAddedMass)
                ;
        }
    }
} // namespace JoltBuoyancy
