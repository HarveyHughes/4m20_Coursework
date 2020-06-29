function robot_walk(walker)
% s=arduino()

% sv1 = servo(s, 'D9', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
% sv2 = servo(s, 'D10', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
% sv3 = servo(s, 'D11', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
% 
% currentpos(1) = readPosition(sv1);
% currentpos(2) = readPosition(sv2);
% currentpos(3) = readPosition(sv3);
% 
traj = walker.th;
% % 
% % convert to be lines up with the robot
% traj = (traj + [7;7;20])/160;
% traj(3,:)=1-traj(3,:);
% 
 T= (walker.t(2)-walker.t(1));
% m=20;
% to_start = zeros(3,m);
% for i = 1:3
%     to_start(i,:) = linspace(currentpos(i),traj(i,1),m);
% end    
% traj = cat(2,to_start,traj);

%plot(0:size(traj,2), traj);
plot(linspace(0,size(traj,2)*T,size(traj,2)), traj);
xlabel('Time/s')
ylabel('Angle/deg')
legend('Theta 1','Theta 2','Theta 3','location','southwest' )

yd = diff(traj,1,2);
md = max(abs(yd),[],2)/T

figure(2)
plot(linspace(0,size(yd,2)*T,size(yd,2)), yd/T);
ydd = diff(traj,2,2);
mdd=max(abs(ydd),[],2)/(T^2)
figure(3)
plot(linspace(0,size(ydd,2)*T,size(ydd,2)), ydd/(T*T));
% actual_traj = zeros (size(traj));
% for i = 1:size(traj,2)
%     writePosition(sv1, traj(1,i));
%     writePosition(sv2, traj(2,i));
%     writePosition(sv3, traj(3,i));
%     currentpos(1) = readPosition(sv1);
%     currentpos(2) = readPosition(sv2);
%     currentpos(3) = readPosition(sv3);
%     actual_traj(:,i)=currentpos;
%     
%     fprintf('Current motor position is %d degrees\n', angle);
%     pause(T);
% end
% 
% figure(2);
% plot(1:size(actual_traj,2), actual_traj);
end


