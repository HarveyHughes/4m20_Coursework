clear all;

s=arduino()



% 
sv = servo(s, 'D9', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6)
current = readPosition(sv);
   for angle = current:0.01:1
       writePosition(sv, angle);
       fprintf('Current motor position is %d degrees\n', angle);
       pause(.01);
   end
% 
%     for angle = 0:0.01:0.5
%        writePosition(sv, angle);
%        fprintf('Current motor position is %d degrees\n', angle);
%        pause(.01);
%    end

  disp(readPosition(sv));
 
