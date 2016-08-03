clc;
clear;
rosinit('10.120.2.127');
% rosinit('10.120.0.73');

% setpoint
size_prbs = 1023;
u1 = idinput(size_prbs,'PRBS');
u2 = zeros(size_prbs,1);%u1(end:-1:1);
u3 = ones(size_prbs,1)*5;
out_x = zeros(size_prbs,1);
out_y = zeros(size_prbs,1);
out_z = zeros(size_prbs,1);
setpointmsg = rosmessage('geometry_msgs/PoseStamped');
setpointpub = rospublisher('/mavros/setpoint_position/local', 'geometry_msgs/PoseStamped');
% setpoint end

% thrust
thrust_time = zeros(size_prbs,1);
dd_time = zeros(size_prbs,1);
thrust = 0.5 - 0.0001*randi([-1000 1000],size_prbs,1);
thrustmsg = rosmessage('std_msgs/Float64');
thrustpub = rospublisher('/mavros/setpoint_attitude/att_throttle', 'std_msgs/Float64');
% thrust end

vehicle_setpoint_data = rosmessage('fmaros_msgs/VehiclePose');
vehiclesub = rossubscriber('/FMA/Vehicle/Pose');
for i=1:size_prbs 
    time_sys = rostime('now');
    % setpoint
    if 0
    setpointmsg.Pose.Position.X = u1(i);
    setpointmsg.Pose.Position.Y = u2(i);
    setpointmsg.Pose.Position.Z = u3(i);    
    
    setpointmsg.Header.Stamp.Sec = time_sys.Sec;
    setpointmsg.Header.Stamp.Nsec = time_sys.Nsec;    
    pause(1/200);
    send(setpointpub,setpointmsg);
    end
    % setpoint end
    
    % thrust
    thrustmsg.Data = double(thrust(i));
    pause(1/1000);
    thrust_time(i)= double(time_sys.Sec)+double(time_sys.Nsec)*10^-9;
    send(thrustpub, thrustmsg);
    % thrust end
    
    vehicle_setpoint_data = receive(vehiclesub,2);
    veh_time1 = double(vehicle_setpoint_data.Header.Stamp.Sec)+double(vehicle_setpoint_data.Header.Stamp.Nsec )*10^-9;
    
    t = rostime('now');
    % Converts ROS time to a double in seconds
    secondtime = double(t.Sec)+double(t.Nsec)*10^-9;
    
    dd = secondtime - veh_time1   
    out_x(i) = vehicle_setpoint_data.Position.X;
    out_y(i) = vehicle_setpoint_data.Position.Y;
    out_z(i) = vehicle_setpoint_data.Position.Z;
end
% save('/home/pengyinhuang/work/matlab_pen/data.mat','u1', 'u2', 'u3', 'out_x', 'out_y', 'out_z')
U=iddata(out_z, thrust, 0.005);
% % na=4*on;
% % nb=na;
% % nk=ones(3);
na=4;
nb=na;
nk=1;
T=arx(U,[na nb nk])
rosshutdown;

