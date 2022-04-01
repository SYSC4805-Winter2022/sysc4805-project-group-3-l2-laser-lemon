--Author: Emma Boulay
--SYSC 4805 L2-G3

--This Program handles the plow opening and closing operations
--The python program can call onOpen or onClose to begin the process
--The program will send a signal back to the Python Remote API client
--when the plow is finished openning or closing.
--This ensures that the robot is not driving while the plow is actuating

function onOpen()
    sim.setStringSignal('openFlag', -1)
    open = true
    leftPanelBool = true
    rightPanelBool = true
    print("Beginning to Open Plow!")
    
end

function onClose()
    sim.setStringSignal('closeFlag', -1)
    close = true
    leftPanelBool = true
    rightPanelBool = true
    print("Beginning to Close Plow!")
end

function sysCall_init()
    --Initialize the plow hinges
    open = false
    close = false
    rightPanelBool, leftPanelBool = false
    rightHinge = sim.getObjectHandle("plow_right_hinge")
    leftHinge = sim.getObjectHandle("plow_left_hinge")
    rightPanel = sim.getObjectHandle("plow_right_panel")
    leftPanel = sim.getObjectHandle("plow_left_panel")
    middlePanel = sim.getObjectHandle("plow_middle_panel")
    setObjectToDynamic(leftPanel)
    setObjectToDynamic(rightPanel)
    --ui=simUI.create('<ui enabled="true" modal="false" title="Plow Control Module" closeable="true" resizable="false" layout="grid" placement="relative" position="20,420"><button enabled="true" text="Open" on-click="onOpen"></button><button enabled="true" text="Close" on-click="onClose"></button></ui>')
end

function sysCall_actuation()
    -- If the Lua program recieves the "open" signal it will start opening the plow
    if open then
        if rightPanel then
            sim.setJointTargetVelocity(rightHinge, -1.5)
        end
        if leftPanel then
            sim.setJointTargetVelocity(leftHinge, 1.5)
        end
        rotRight = sim.getObjectOrientation(rightPanel, middlePanel)[3] * 57.2957795
        if rotRight > 135.0 then
            sim.setJointTargetVelocity(rightHinge, 0)
            rightPanelBool = false
        end
        rotLeft = sim.getObjectOrientation(leftPanel, middlePanel)[3] * 57.2957795
        if rotLeft < -135.0 then
            sim.setJointTargetVelocity(leftHinge, 0)
            leftPanelBool = false
        end
        if not leftPanelBool and not rightPanelBool then
            open = false
            sim.setStringSignal('openFlag', 1)
            print("Plow is finished opening!")
            setObjectToStatic(leftPanel)
            setObjectToStatic(rightPanel)
        end
    end
    -- If the Lua program recieves the "open" signal it will start opening the plow
    if close then
        if rightPanel then
            sim.setJointTargetVelocity(rightHinge, 0.5)
        end
        if leftPanel then
            sim.setJointTargetVelocity(leftHinge, -0.5)
        end
        rotRight = sim.getObjectOrientation(rightPanel, middlePanel)[3] * 57.2957795
        if rotRight <= 2 then
            sim.setJointTargetVelocity(rightHinge, 0)
            rightPanelBool = false
        end
        rotLeft = sim.getObjectOrientation(leftPanel, middlePanel)[3] * 57.2957795
        if rotLeft >= -2 then
            sim.setJointTargetVelocity(leftHinge, 0)
            leftPanelBool = false
        end
        if not leftPanelBool and not rightPanelBool then
            close = false
            sim.setStringSignal('closeFlag', 1)
        end
    end
    
end


function sysCall_cleanup()
    -- do some clean-up here
    setObjectToDynamic(leftPanel)
    setObjectToDynamic(rightPanel)
    
end

function setObjectToStatic(handle)
    sim.setObjectInt32Parameter(handle,sim.shapeintparam_static,1)
    sim.resetDynamicObject(handle)
    return {},{},{},''
end

function setObjectToDynamic(handle)
    sim.setObjectInt32Parameter(handle,sim.shapeintparam_static,0)
    sim.resetDynamicObject(handle)
    return {},{},{},''
end

