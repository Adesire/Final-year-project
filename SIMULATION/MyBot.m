vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected')
    %Handling code
    %[returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking); 
    %[returnCode,front_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking); 
    sensorHandles = zeros(16,1);
    sensorValues = zeros(16,3);
    
    %sensor handles
    for i=1:16
        [returnCode, sensorHandle] = vrep.simxGetObjectHandle(clientID, strcat('Pioneer_p3dx_ultrasonicSensor',num2str(i)), vrep.simx_opmode_blocking);  
        sensorHandles(i) = sensorHandle;
        
        [returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector] = vrep.simxReadProximitySensor(clientID, sensorHandle, vrep.simx_opmode_oneshot_wait);
    end
    
    %motor handles
    [returnCodeA, Left_Motor]  = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait);
    [returnCodeB, Right_Motor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait);

    %Fuzzy Controller
    fismat = readfis('FuzzyLogic2.fis');
    
    %run 1000 iterations
    for t = 1:1000
        for i=1:16
        sensorHandle = sensorHandles(i);
        [returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector] = vrep.simxReadProximitySensor(clientID, sensorHandle, vrep.simx_opmode_oneshot_wait);
        
            if detectedPoint(3)<0.01
                sensorValues(i,3) = 1;
            else
                sensorValues(i,3) = detectedPoint(3);
            end
        end
        %Foward Sensors
        fwdDist = mean([sensorValues(4,3),sensorValues(5,3)]);
        
        %Left and Right Sensors
        leftDist = sensorValues(2,3);
        rightDist = sensorValues(7,3);
        
        %send sensor data as input to FuzzyController and return speed
        %values
        velocity = evalfis([fwdDist,leftDist,rightDist],fismat);
        
        %set robot left and right speeds with returned values
        Left_Velocity = vrep.simxSetJointTargetVelocity(clientID, Left_Motor, velocity(1), vrep.simx_opmode_streaming);
        Right_Velocity = vrep.simxSetJointTargetVelocity(clientID, Right_Motor, velocity(2), vrep.simx_opmode_streaming);
        
    end
    
    %At the end of simulation
    Left_Velocity = vrep.simxSetJointTargetVelocity(clientID, Left_Motor, 0, vrep.simx_opmode_streaming);
    Right_Velocity = vrep.simxSetJointTargetVelocity(clientID, Right_Motor, 0, vrep.simx_opmode_streaming);
   
    %Other Code
    %[returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0.1,vrep.simx_opmode_blocking);
    %[returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor, vrep.simx_opmode_streaming);
    %pause(5) 
%     for i=1:50
%         %[returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor, vrep.simx_opmode_buffer);
%         disp(norm(detectedPoint));
%         pause(0.1);
%     end
    %[returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
    
    vrep.simxFinish(-1);
end  
vrep.delete();  