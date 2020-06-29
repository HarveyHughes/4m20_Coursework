
% LED Blinker Script

s=arduino() % setup the port

for i=1:5
writeDigitalPin(s,'D2',1) 
pause(1)
writeDigitalPin(s,'D2',0)
pause(1)
end

clear s
