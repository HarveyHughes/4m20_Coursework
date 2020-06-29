function test_angle(theta)

s=arduino()
% 
sv1 = servo(s, 'D9', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6)
sv2 = servo(s, 'D10', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6)
sv3 = servo(s, 'D11', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6)

currentpos(1) = readPosition(sv1);
currentpos(2) = readPosition(sv2);
currentpos(3) = readPosition(sv3);


%convert to be lines up with the robot
theta = (theta(:) + [7;7;20])/160;
theta(3)=1-theta(3);

T= 0.05;
m=50;
to_start = zeros(3,m);
for i = 1:3
    to_start(i,:) = linspace(currentpos(i),theta(i),m);
end    
traj = to_start;
% plot(1:size(traj,2), traj);

% 
% actual_traj = zeros (size(traj));
for i = 1:size(traj,2)
    writePosition(sv1, traj(1,i));
    writePosition(sv2, traj(2,i));
    writePosition(sv3, traj(3,i));
%     currentpos(1) = readPosition(sv1);
%     currentpos(2) = readPosition(sv2);
%     currentpos(3) = readPosition(sv3);
%     actual_traj(:,i)=currentpos;
%     
    %fprintf('Current motor position is %d degrees\n', angle);
    pause(T);
end
% 
% figure(2);
% plot(1:size(actual_traj,2), actual_traj);
end


