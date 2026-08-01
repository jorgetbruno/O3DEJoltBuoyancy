-- Reacts to bodies entering and leaving a water volume.
--
-- Attach this to the same entity as a Jolt Water Volume, using the Lua Script component.
-- The notification bus is addressed by the water volume's entity, so `self.entityId` is
-- the right address and no wiring is needed beyond attaching the script.
--
-- Both callbacks are raised after the physics step has finished, never from inside it, so
-- it is safe to spawn effects, start sounds, or touch the body you were handed.

local WaterSplash =
{
    Properties =
    {
        -- Entering slowly is a body drifting in, not a splash. The speed handed to
        -- OnBodyEnteredWater is measured along the surface normal, so this threshold is
        -- "how hard did it hit the surface", not "how fast was it going".
        MinimumSplashSpeed = { default = 2.0, suffix = " m/s",
            description = "Ignore entries slower than this." },

        LogSplashes = { default = true,
            description = "Print each splash to the console. Turn off once the effects are wired up." },
    },
}

function WaterSplash:OnActivate()
    self.waterHandler = JoltWaterVolumeNotificationBus.Connect(self, self.entityId)
end

function WaterSplash:OnDeactivate()
    if self.waterHandler ~= nil then
        self.waterHandler:Disconnect()
        self.waterHandler = nil
    end
end

function WaterSplash:OnBodyEnteredWater(bodyEntityId, speed)
    if speed < self.Properties.MinimumSplashSpeed then
        return
    end

    if self.Properties.LogSplashes then
        Debug.Log("Splash: entity entered the water at " .. tostring(speed) .. " m/s")
    end

    -- Where to hang a particle effect or a sound. The surface position under the body is
    -- available from the water volume itself, and follows the waves:
    --
    --   local where = JoltWaterVolumeRequestBus.Event.GetSurfacePositionAt(
    --       self.entityId, TransformBus.Event.GetWorldTranslation(bodyEntityId))
end

function WaterSplash:OnBodyExitedWater(bodyEntityId)
    if self.Properties.LogSplashes then
        Debug.Log("A body left the water")
    end
end

return WaterSplash
