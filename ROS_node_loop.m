%Start Node
addpath(genpath(pwd));
clear 
clc
close all

%% Starts ROS through MATLAB
rosshutdown
matlabIP = 'Kenneths-MacBook-Pro.local'; % IP address of MATLAB host PC
rosIP = "http://relay.local:11311/";    % IP address of ROS enabled machine
setenv('ROS_IP',matlabIP); 
setenv('ROS_MASTER_URI',rosIP); 
rosinit
%% Ser up relevant Parameters
%Drone Name
drone_id = "drone1";
offset = .060;
% t_final (max time the trajectory can take I guess)
global t_final
t_final = rostime(16);
% Topic Names

 targetPose_topic = drone_id + "/command/pose";
 targetTwist_topic = drone_id + "/command/twist";

%subscriber
subTarget = rossubscriber('/gtarg/mavros/vision_pose/pose_EKF','geometry_msgs/PoseStamped');
%subDronePose = rossubscriber('/drone1/mavros/vision_pose/pose','geometry_msgs/PoseStamped');
subDronePose = rossubscriber('/drone1/mavros/local_position/pose','geometry_msgs/PoseStamped');
%/drone1/mavros/local_position/pose
%update this with local_position

%publishers
global pose_sp_pub vel_sp_pub
pose_sp_pub = rospublisher(targetPose_topic,'geometry_msgs/PoseStamped');
vel_sp_pub = rospublisher(targetTwist_topic,'geometry_msgs/TwistStamped');
%%
% Room Details
map.x_lim =[ -8 8];
map.y_lim = [-4 4];
map.z_lim = [0 3];
%%

% ROS Messages
global pose_sp 
pose_sp = rosmessage(pose_sp_pub);
global vel_sp
 vel_sp = rosmessage(vel_sp_pub);
 
 %%
% Counters and Time Initialization
global    k_main
k_main = 0;
global    k_traj
k_traj = 1;
global    k_loop
k_loop = 0;
global replanTimer
replanDeltaT = .2;
replanTimer = timer;%(replanDeltaT);
  %%  
% Get Latest input here
% get the start Pose Here
disp("Waiting for Drone Pose")
dronePose =receive( subDronePose );

start=zeros(3,1);
start(1,1) = dronePose.Pose.Position.X;
start(2,1) = dronePose.Pose.Position.Y;
start(3,1) = dronePose.Pose.Position.Z;


% take in the goal position from the EKF
disp("Waiting for Target Pose")
targetPose =receive( subTarget );
goal=zeros(3,1);
goal(1) = targetPose.Pose.Position.X;
goal(2) = targetPose.Pose.Position.Y;
goal(3) = targetPose.Pose.Position.Z + offset;

% update this with orientation later too once we get there.
% Also desired Velocity

%% Trajectory Generation and Plot
start  = [-4,0,1]'; %[XYZ POSE]
goal   = [-1,0,1.2]';
K_end = 1.5;
endPos = K_end * (goal - start) + goal;

%endPos = [2,0,1.0]';
%endPos=goal;
%compute trajectory
tic
[traj,log] = trajGenVel(start,goal,endPos,0);
toc
%Load the trajectory in the proper format


pos_out = squeeze(traj.f_out(:,1,:));
vel_out = squeeze(traj.f_out(:,2,:));

%t_pos_out = [log.t_fmu ; pos_out];
%t_vel_out = [log.t_fmu ; vel_out];
t_pv_out  = [log.t_fmu(1:end-1) ; pos_out ; vel_out];
%t_pv_out = [log.t_fmu ; traj.x]; %[t pos vel quat omega]
global st_traj
st_traj = t_pv_out;

global  t_end
t_end= rostime(t_pv_out(1,end));


%Check Trajectory Plot
plot3(t_pv_out(2,:),t_pv_out(3,:),t_pv_out(4,:))

axis equal
axis([map.x_lim,map.y_lim,map.z_lim])
plotfixer
hold on
scatter3(start(1),start(2),start(3),'filled')
scatter3(goal(1),goal(2),goal(3),'filled')
legend('trajectory','start','goal')
disp("Z Max=")
disp(num2str(max(pos_out(3,:))));
disp("Z Min=")
disp(num2str(min(pos_out(3,:))));
disp("X Max=")
disp(num2str(max(pos_out(1,:))));
disp("X Min=")
disp(num2str(min(pos_out(1,:))));

disp("Press Key if It looks Good")
pause
%close all

%% Package for ROS 
%Get length of the trajectory; 
global n_fr
n_fr = length(t_pv_out);


%Transfer the file to the VM if possible
%initiatilize the trajectory



%% launch the trajectory 

r = rosrate(200);
disp('about to start publishing')
reset(r)

global t_start 
t_start = rostime('now');

while(1)
    update_setpoint();
    waitfor(r);
    
end


function update_setpoint()
global t_start k_traj n_fr t_final st_traj k_main k_loop pose_sp_pub vel_sp_pub ...
    t_end pose_sp vel_sp
t_stamp = rostime("now");
%t_now = rostime('now').seconds - t_start.seconds;
t_now = t_stamp.seconds - t_start.seconds;

%t_wp = rostime(st_traj(1,k_traj));
t_wp=st_traj(1,k_traj);

%pose_sp = rosmessage(pose_sp_pub);
%vel_sp = rosmessage(vel_sp_pub);


if t_now < t_final.seconds 
    
    t_loop = t_now- k_loop*t_end.seconds; %not sure about this line 
    %make sure you can advance to the next WP
%k_traj
    if  ((k_traj < n_fr) && (t_loop > t_wp))
            pose_sp.Pose.Position.X = st_traj(2,k_traj);
            pose_sp.Pose.Position.Y = st_traj(3,k_traj);
            pose_sp.Pose.Position.Z = st_traj(4,k_traj);
            
             roll = 0.0;
             pitch = 0.0;
             yaw =  st_traj(5,k_traj); %not sure about the coupling here
 
             cy = cos(yaw * 0.5);
             sy = sin(yaw * 0.5);
             cp = cos(pitch * 0.5);
             sp = sin(pitch * 0.5);
             cr = cos(roll * 0.5);
             sr = sin(roll * 0.5);
 
             pose_sp.Pose.Orientation.W = cr * cp * cy + sr * sp * sy; 
             pose_sp.Pose.Orientation.X = sr * cp * cy - cr * sp * sy;
             pose_sp.Pose.Orientation.Y = cr * sp * cy + sr * cp * sy;
             pose_sp.Pose.Orientation.Z = cr * cp * sy - sr * sp * cy;
            
             vel_sp.Twist.Linear.X  = st_traj(6,k_traj);
             vel_sp.Twist.Linear.Y  = st_traj(7,k_traj);
             vel_sp.Twist.Linear.Z  = st_traj(8,k_traj);
            
            %pose_sp.pose.orientation.w = st_traj(8,k_traj);
            %pose_sp.pose.orientation.x = st_traj(9,k_traj);
            %pose_sp.pose.orientation.y = st_traj(10,k_traj);
            %pose_sp.pose.orientation.z = st_traj(11,k_traj);
          
           % vel_sp.twist.angular.x = st_traj(12,k_traj);
            %vel_sp.twist.angular.y = st_traj(13,k_traj);
            vel_sp.Twist.Angular.Z = st_traj(9,k_traj);
            k_traj = k_traj + 1;
            
    elseif ((k_traj > n_fr))
        return
        k_loop = k_loop + 1;
    
        
    end
        

    
    %publish
    
    %t_stamp = rostime("now");
    
    %publish Pose
    
    pose_sp.Header.Stamp = t_stamp;
    pose_sp.Header.Seq = k_main;
    pose_sp.Header.FrameId = "map";
    pose_sp_pub.send(pose_sp);
    
    
    %Publish Twist
    vel_sp.Header.Stamp = t_stamp;
    vel_sp.Header.Seq = k_main;
    vel_sp.Header.FrameId = "map";
    vel_sp_pub.send(vel_sp)

    k_main = k_main + 1;
    
       
end

if t_now>t_final.seconds
disp("done Publishing")
disp("got to:")
k_traj
rosshutdown
error()
end        
end



