--Author: Emma Boulay
--SYSC 4805 L2-G3

--The safety light will blink amber (blue is a protected 
--colour only for city sanctioned vehicles

function sysCall_init()
    --Initialize the snow clearing safety light
    snowClearingLight = sim.getObjectAssociatedWithScript(sim.handle_self)
    sim.setShapeColor(snowClearingLight, '', sim.colorcomponent_ambient_diffuse, {0.85,0.73,0.63})
    lastTime=sim.getSimulationTime()
    lightOn = false
end

function sysCall_actuation()
    --The light will continue to blink with a speed of 0.7 seconds
    currentTime = sim.getSimulationTime()
    if(currentTime - lastTime > 0.7) then
        if (lightOn) then 
            sim.setShapeColor(snowClearingLight, '', sim.colorcomponent_ambient_diffuse, {0.85,0.73,0.63})
        else
            sim.setShapeColor(snowClearingLight, '', sim.colorcomponent_ambient_diffuse, {1,0.75,0})
        end
        lightOn = not lightOn
        lastTime = sim.getSimulationTime()
    end
end

--This ensures that the light is turned off at the end of the simulation
function sysCall_cleanup()
    sim.setShapeColor(snowClearingLight, '', sim.colorcomponent_ambient_diffuse, {0.85,0.73,0.63})
end
