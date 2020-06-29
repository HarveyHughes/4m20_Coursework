clear s sv1 sv2 sv3;

%% set up arduino
s=arduino()
sv1 = servo(s, 'D9', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
sv2 = servo(s, 'D10', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
sv3 = servo(s, 'D11', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);

currentpos(1) = readPosition(sv1);
currentpos(2) = readPosition(sv2);
currentpos(3) = readPosition(sv3);


%% Servo tests
% servo 1
disp('servo 1 start')
for angle = linspace(0,0.5)
    writePosition(sv1, angle);
    pause(.05);
end
disp('servo 1 end')
pause(2)

% servo 2
disp('servo 2 start')
for angle = linspace(0,0.5)
    writePosition(sv2, angle);
    pause(.05);
end
disp('servo 2 end')
pause(2)

% servo 3
disp('servo 3 start')
for angle = linspace(1,0.5)
    writePosition(sv3, angle);
    pause(.05);
end
disp('servo 3 end')

clear s sv1 sv2 sv3;