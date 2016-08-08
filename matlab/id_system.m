clc;
clear;
rosinit('10.120.2.127');
% rosinit('10.120.0.73');

% setpoint
size_prbs = 1300;
if 0
u1 = idinput(size_prbs,'PRBS');
u2 = zeros(size_prbs,1);%u1(end:-1:1);
u3 = ones(size_prbs,1)*5;
out_x = zeros(size_prbs,1);
out_y = zeros(size_prbs,1);
out_z = zeros(size_prbs,1);
setpointmsg = rosmessage('geometry_msgs/PoseStamped');
setpointpub = rospublisher('/mavros/setpoint_position/local', 'geometry_msgs/PoseStamped');
end
% setpoint end

% thrust
thrustmsg = rosmessage('std_msgs/Float64');
thrustpub = rospublisher('/mavros/setpoint_attitude/att_throttle', 'std_msgs/Float64');
% thrust end

vehicle_setpoint_data = rosmessage('fmaros_msgs/VehiclePose');
vehiclesub = rossubscriber('/FMA/Vehicle/Pose');
%for i=1:(size_prbs*2) 

thrust2 = 0;
i = 1;
out_z=[];
out_thrust=[];
while 1
   
    % thrust
    if i/200 > 15
        break;
    end
    
    if i/200 <= 5
    thrust2 = 0.6;
    elseif i/200 <= 10
    thrust2 = 0.7;
    else
    thrust2 = 0.65;
    end
    
    thrustmsg.Data = thrust2;
    
    pause(1/200);
    send(thrustpub, thrustmsg);
    % thrust end

    vehicle_setpoint_data = receive(vehiclesub,1);
    out_z(i) = vehicle_setpoint_data.Position.Z;
    out_thrust(i) = thrust2;
    i = i + 1;
 
end

% save('/home/pengyinhuang/work/matlab_pen/data.mat','u1', 'u2', 'u3', 'out_x', 'out_y', 'out_z')

rosshutdown;

