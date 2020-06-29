function xyrobot = walker_fw_kin(th, xy1)

global L1 L2 L3 L4

th1 = th(1);
th2 = th(2);
th3 = th(3);

xy2 = xy1 + [0; L1];
xy3 = xy2 + L2*[sin(th1); cos(th1)];
xy4 = xy3 + L3*[sin(th1 + th2); cos(th1 + th2)];
xy5 = xy4 + L4*[sin(th1 + th2 + th3); cos(th1 + th2 + th3)];
%xy6 = xy5 + L5*[cos(th1 + th2 + th3); -sin(th1 + th2 + th3)];

%which leg is on the floor?
floorlevel = min(xy1(2), xy5(2));


xyrobot = [xy1, xy2, xy3, xy4, xy5] ; %+ [0; floorlevel]; 

end