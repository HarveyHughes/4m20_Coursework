%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%          Mobile Robot Simulation                              %%%%%%
%%%%%%          4M20 Robotics, coursework template                   %%%%%%
%%%%%%          University of Cambridge                              %%%%%%
%%%%%%          10.10.2018                                           %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main function                                                         %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Blank slate
clear all
close all
clc

%% Parameters
l_s = 0.1;      %shaft length vehicle
r_w = 0.02;     %radius wheel
dt = 1e-3;      %time increment
N = 5000;

%% Initialisation
%robot_state = [0.9;0.9;pi/2];  %initial robot position and orientation
robot_state = [1.0;1.0;0.0];  %initial robot position and orientation
robot_state_old = robot_state;      %save old state for trajectory


figure(1)
plot([robot_state(1);robot_state_old(1)],[robot_state(2);robot_state_old(2)])
hold on


%initialisation of animation
axis([0 2 0 2])
ll = line(0,0,'color','k','LineWidth',2);   %left side of vehicle
lr = line(0,0,'color','k','LineWidth',2);   %right side of vehicle
lf = line(0,0,'color','k','LineWidth',2);   %front of vehicle
lb = line(0,0,'color','k','LineWidth',2);   %back of vehile
lw1 = line(0,0,'color','k','LineWidth',5);  %wheel left
lw2 = line(0,0,'color','k','LineWidth',5);  %wheel right

idx=1;
r_c1 = robot_state(1:2,idx) + l_s/2*[-sin(robot_state(3,idx));cos(robot_state(3,idx))] - l_s/2*[cos(robot_state(3,idx));sin(robot_state(3,idx))];
r_c2 = r_c1 + l_s*[cos(robot_state(3,idx));sin(robot_state(3,idx))];
r_c3 = r_c2 + l_s*[sin(robot_state(3,idx));-cos(robot_state(3,idx))];
r_c4 = r_c3 + l_s*[-cos(robot_state(3,idx));-sin(robot_state(3,idx))];
r_w1 = [r_c1 + (r_c2-r_c1)/4,r_c1 + (r_c2-r_c1)*3/4];
r_w2 = [r_c3 + (r_c4-r_c3)/4,r_c3 + (r_c4-r_c3)*3/4];

plot([robot_state(1,idx);robot_state_old(1)],[robot_state(2,idx);robot_state_old(2)])
set(ll,'xdata',[r_c1(1) r_c2(1)],'ydata',[r_c1(2) r_c2(2)])
set(lf,'xdata',[r_c2(1) r_c3(1)],'ydata',[r_c2(2) r_c3(2)])
set(lr,'xdata',[r_c3(1) r_c4(1)],'ydata',[r_c3(2) r_c4(2)])
set(lb,'xdata',[r_c4(1) r_c1(1)],'ydata',[r_c4(2) r_c1(2)])
set(lw1,'xdata',[r_w1(1,1) r_w1(1,2)],'ydata',[r_w1(2,1) r_w1(2,2)])
set(lw2,'xdata',[r_w2(1,1) r_w2(1,2)],'ydata',[r_w2(2,1) r_w2(2,2)])
drawnow;
      



%% Simulation
% Define a target location based on user input
pointer_location=ginput(1);
pDes = [pointer_location';1.75*atan((pointer_location(2)-robot_state(2))/(pointer_location(1)-robot_state(1)))]    %get user defined target              
%pDes = [.5;.5;-0.8]    %get user defined target              

% Calculate mobile robot trajectory
t = 0:dt:N;
for i = 1:N
    
    [pDes', robot_state(:,i)', pDes'- robot_state(:,i)'];

     robot_jacobian = [r_w*cos(robot_state(3,i))/2, r_w*cos(robot_state(3,i))/2, 0; ...
                   r_w*sin(robot_state(3,i))/2, r_w*sin(robot_state(3,i))/2, 0; ...
                   r_w/l_s/4, -r_w/l_s/4, 0];
    inv_robot_jacobian = pinv(robot_jacobian);
    
    motor_out = inv_robot_jacobian * (pDes - robot_state(:,i));
    robot_state(:,i+1) = robot_state(:,i) + robot_jacobian*[motor_out(1);motor_out(2);0]*dt;
end


% Visualize the obtained trajectory
robot_state_old = robot_state(:,1);
t_next = 0;   %variable for timing of frame capture
RepSpeed = 1; %replay speed
fps = 30;  %frames per second
tic
while toc < t(end)
        
    % Animation
    if mod(toc,1/fps) > mod(toc,1/fps+dt)

        idx = floor(toc/dt*RepSpeed);
        if idx>N
            break
        end

        r_c1 = robot_state(1:2,idx) + l_s/2*[-sin(robot_state(3,idx));cos(robot_state(3,idx))] - l_s/2*[cos(robot_state(3,idx));sin(robot_state(3,idx))];
        r_c2 = r_c1 + l_s*[cos(robot_state(3,idx));sin(robot_state(3,idx))];
        r_c3 = r_c2 + l_s*[sin(robot_state(3,idx));-cos(robot_state(3,idx))];
        r_c4 = r_c3 + l_s*[-cos(robot_state(3,idx));-sin(robot_state(3,idx))];
        r_w1 = [r_c1 + (r_c2-r_c1)/4,r_c1 + (r_c2-r_c1)*3/4];
        r_w2 = [r_c3 + (r_c4-r_c3)/4,r_c3 + (r_c4-r_c3)*3/4];


        
        plot([robot_state(1,idx);robot_state_old(1)],[robot_state(2,idx);robot_state_old(2)])

        set(ll,'xdata',[r_c1(1) r_c2(1)],'ydata',[r_c1(2) r_c2(2)])
        set(lf,'xdata',[r_c2(1) r_c3(1)],'ydata',[r_c2(2) r_c3(2)])
        set(lr,'xdata',[r_c3(1) r_c4(1)],'ydata',[r_c3(2) r_c4(2)])
        set(lb,'xdata',[r_c4(1) r_c1(1)],'ydata',[r_c4(2) r_c1(2)])
        set(lw1,'xdata',[r_w1(1,1) r_w1(1,2)],'ydata',[r_w1(2,1) r_w1(2,2)])
        set(lw2,'xdata',[r_w2(1,1) r_w2(1,2)],'ydata',[r_w2(2,1) r_w2(2,2)])

        drawnow
        robot_state_old = robot_state(:,idx);
    end
end