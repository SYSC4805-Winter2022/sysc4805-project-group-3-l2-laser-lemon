--Author: Emma Boulay
--SYSC 4805 L2-G3

--This program starts the Remote API Server
--It also provides Lua functions for the Python client
--to use. Because of the delay in the communication, functions
--such as going straight will have a noticable delay in when each wheel
--is modified if executed in Python.
function sysCall_init() 
    simRemoteApi.start(19997)
    
    --Modified Braitenberg Algorithm Initialization Code from PioneerP3dx Robot
    local robot=sim.getObject(".")
    local obstacles=sim.createCollection(0)
    sim.addItemToCollection(obstacles,sim.handle_all,-1,0)
    sim.addItemToCollection(obstacles,sim.handle_tree,robot,1)
    usensors={}
    for i=1,8,1 do
        usensors[i]=sim.getObject("./ultrasonicSensor",{index=i-1})
        sim.setObjectInt32Param(usensors[i],sim.proxintparam_entity_to_detect,obstacles)
    end
    noDetectionDist=0.5
    maxDetectionDist=0.2
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6}
    braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2}
    v0=2
    prevDetect = true
    
    --Initialize Motors
    rightJoint = sim.getObjectHandle("rightJoint")
    leftJoint = sim.getObjectHandle("leftJoint")
    
    --Initialize Front and Back Proximity Sensors
    frontSensor = sim.getObjectHandle("Proximity_sensorFront")
    backSensor = sim.getObjectHandle("Proximity_sensorBack")

end

--This allows the python client to set both wheels with 
--low latency
function straight(inInts,inFloats,inStrings,inBuffer)
    velocity = inFloats[1]
    sim.setJointTargetVelocity(rightJoint, velocity)
    sim.setJointTargetVelocity(leftJoint, velocity)
end

--This function checks if any obstacles are detected. If there
--is an obstacles it will modify the wheel velocities to avoid
--that obstacle
function detectObstacle(inInts,inFloats,inStrings,inBuffer)
    v0 = inFloats[1]
    vLeft=v0
    vRight=v0
    objectDetected = 0
    for i=1,8,1 do
        res,dist=sim.readProximitySensor(usensors[i])
        if (res>0) and (dist<noDetectionDist) then
            objectDetected = 1
            prevDetect = true
            vLeft=v0/8
            vRight=v0/8
            if (dist<maxDetectionDist) then
                dist=maxDetectionDist
            end
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else
            detect[i]=0
        end
    end
    
    
    for i=1,8,1 do
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
         
    end
    
    if objectDetected == 1 then
        print("detected")
        sim.setJointTargetVelocity(leftJoint,vLeft)
        sim.setJointTargetVelocity(rightJoint,vRight)
        return {1}, {},{}, '' 
    elseif (objectDetected == 0) and (prevDetect == true) then
        sim.setJointTargetVelocity(leftJoint,vLeft)
        sim.setJointTargetVelocity(rightJoint,vRight)
        prevDetect = false
        return {0}, {},{}, '' 
    else 
        return {0}, {},{}, ''
    end

end 

--This function checks if the back proximity sensor detects an object.
--It is called when the robot is reversing
function detectObstacleBackwards(inInts,inFloats,inStrings,inBuffer)
    res,dist=sim.readProximitySensor(backSensor)
    if (res > 0) then
        return {1}, {},{}, ''
    end
    
    return {0}, {},{}, ''
end

--This function checks if the front proximity sensor detects an object
--It is called when the robot is turning
function detectObstacleTurning(inInts,inFloats,inStrings,inBuffer)
    res,dist=sim.readProximitySensor(frontSensor)
    if (res > 0) then
        return {1}, {},{}, ''
    end
    
    return {0}, {},{}, ''
end