%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%          2-Link Manipulator                                   %%%%%%
%%%%%%          M20 Robotics, coursework template                    %%%%%%
%%%%%%          University of Cambridge                              %%%%%%
%%%%%%          10.10.2018                                           %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main function                                                         %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function manipulator
%% Blank slate
clear all
close all
clc

%% Parameters
l1 = 1;                 %length bottom link [m]
l2 = 1;                 %length middle link [m]
l3 = 0.5;               %length upper link [m]
lg = 0.2;               %length gripper [m]
wg = 0.2;               %width gripper [m]

%% Initial conditions
phi1 = 0.0;               %angle bottom link [rad]
phi2 = pi/2;            %angle middle link [rad]

phiVec = [phi1;phi2];
lVec = [l1,l2];

%% Initialise animation
lineObj = animInit;
animation([phiVec,phiVec],lVec,lineObj,[-1,1]);

%% Run main routine
for i = 1:1

    pDes = [ginput(1)';1];    %get user defined target              
    phiVecNew = JacInv(phiVec,lVec,pDes);   %inverse kinematics
    animation([phiVec,phiVecNew],lVec,lineObj,pDes) %animation
    phiVec = phiVecNew;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local functions                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Animation initialisation
function lineObj = animInit()
    
    lWin = 3.0;    %window size
    bOffs = 0.1;            %base offset
    
    figure(1)
    hold on
    axis equal
    axis([-3 3 -0.4 3])
    % Ground plot
    plot([-lWin lWin],[-2*bOffs -2*bOffs],'k','LineWidth',3)
    plot(0,-bOffs,'^k','LineWidth',5)
    for j = 1:30
        k = -lWin-1+j*2*lWin/20;
        plot([k;k+0.2],[0-2*bOffs;-0.2-2*bOffs],'k','LineWidth',3)
    end

    %target line object
    lineObj.m1 = line(0,0,'color','r','LineWidth',5,'Marker','o');
    %link line objects
    lineObj.h1 = line(0,0,'color','k','LineWidth',2);
    lineObj.h2 = line(0,0,'color','k','LineWidth',2);
    lineObj.h3 = line(0,0,'color','k','LineWidth',2);

    %joint line objects
    lineObj.d1 = line(0,0,'color','k','LineWidth',5,'Marker','o');
    lineObj.d2 = line(0,0,'color','k','LineWidth',5,'Marker','o');

    %gripper line objects
    lineObj.g1 = line(0,0,'color','k','LineWidth',2);
    lineObj.g2 = line(0,0,'color','k','LineWidth',2);
    lineObj.g3 = line(0,0,'color','k','LineWidth',2);
    

end

%% Animation
function animation(phiMat,lVec,lineObj,PDes)
    
    PhiVec = phiMat(:,2);
    PhiVecOld = phiMat(:,1);
    [A10,A21] = HomCoord(PhiVec,lVec); %calculate transformation matrices


    
    r11 = A10*[0;l1;1];   %position link 1
    r22 = A10*A21*[0;l2;1];   %position link 2

    rg11 = A10*A21*[wg/2;l2;1];   %position gripper base right
    rg22 = A10*A21*[wg/2;lg+l2;1];    %position gripper right
    rg11m = A10*A21*[-wg/2;l2;1]; %position gripper base left
    rg22m = A10*A21*[-wg/2;lg+l2;1];  %position gripper left
    
    %set target point
    set(lineObj.m1,'xdata',PDes(1),'ydata',PDes(2))
    
    %number of animated frames from start to end position
    n = 30;
 
    if PhiVecOld ~= PhiVec
        
        for k = 1:n

            %avoid multiple rotations
            PhiVec = mod(PhiVec,2*pi);  
            PhiVecOld = mod(PhiVecOld,2*pi);
            dPhi = PhiVec-PhiVecOld;
            dC = abs(dPhi)>pi;
            dPhi = dPhi - dPhi.*dC - sign(dPhi.*dC).*(2*pi*dC-abs(dPhi.*dC));

            %calculate intermediate positions
            phiVecT = PhiVecOld + dPhi*k/n;
            [A10T,A21T] = HomCoord(phiVecT,lVec);
            r1T = A10T*[0;l1;1];
            r2T = A10T*A21T*[0;l2;1];
   
            rg1T = A10T*A21T*[wg/2;l2;1];
            rg2T = A10T*A21T*[wg/2;lg+l2;1];
            rg1Tm = A10T*A21T*[-wg/2;l2;1];
            rg2Tm = A10T*A21T*[-wg/2;lg+l2;1];

            %set intermediate positions and pause
            set(lineObj.h1,'xdata',[0 r1T(1)],'ydata',[0 r1T(2)])
            set(lineObj.h2,'xdata',[r1T(1) r2T(1)],'ydata',[r1T(2) r2T(2)])

            set(lineObj.d1,'xdata',r1T(1),'ydata',r1T(2))
%            set(lineObj.d2,'xdata',r2T(1),'ydata',r2T(2))

            set(lineObj.g1,'xdata',[rg1Tm(1) rg1T(1)],'ydata',[rg1Tm(2) rg1T(2)])
            set(lineObj.g2,'xdata',[rg1T(1) rg2T(1)],'ydata',[rg1T(2) rg2T(2)])
            set(lineObj.g3,'xdata',[rg1Tm(1) rg2Tm(1)],'ydata',[rg1Tm(2) rg2Tm(2)])
            pause(1/n)
        end
    end
    
    %set target position and pause
    set(lineObj.h1,'xdata',[0 r11(1)],'ydata',[0 r11(2)])
     set(lineObj.h2,'xdata',[r11(1) r22(1)],'ydata',[r11(2) r22(2)])

     set(lineObj.d1,'xdata',r11(1),'ydata',r11(2))
%     set(lineObj.d2,'xdata',r22(1),'ydata',r22(2))

     set(lineObj.g1,'xdata',[rg11m(1) rg11(1)],'ydata',[rg11m(2) rg11(2)])
     set(lineObj.g2,'xdata',[rg11(1) rg22(1)],'ydata',[rg11(2) rg22(2)])
     set(lineObj.g3,'xdata',[rg11m(1) rg22m(1)],'ydata',[rg11m(2) rg22m(2)])

    drawnow
end

%% Homogeneous coordinate transform
function [A10,A21] = HomCoord(phiVec,lVec)
    
    Phi1 = phiVec(1);
    Phi2 = phiVec(2);

    L1 = lVec(1);
    L2 = lVec(2);

    A10 = [cos(Phi1) , -sin(Phi1) , 0; ...
           sin(Phi1) ,  cos(Phi1) , 0; ...
           0          , 0         , 1 ];

    A21 = [cos(Phi2) , -sin(Phi2) , 0; ...
           sin(Phi2) ,  cos(Phi2) , L1; ...
           0          , 0         , 1];

end

%% Jacobian inverse method (inverse kinematics)
function PhiVec = JacInv(PhiVec,LVec,PDes)
    
L2 = LVec(2)+0.1;   %offset target
j = 0;
[A10,A21] = HomCoord(PhiVec,LVec);
rCurr = A10*A21*[0;L2;1] %current end effector position


pErr = norm(PDes-rCurr)    %current position error
errThresh = 0.001;  %error threshold
 [PDes(1:2) rCurr(1:2)]
%  PhiVec 
% LVec
    while pErr > errThresh
        
        [PDes(1:2) rCurr(1:2) PhiVec]
        
        J = Jac(PhiVec,LVec);      

        dx =  pinv(J)*(PDes(1:2)-rCurr(1:2)); %pinv Moore-Penrose pseudoinverse
        PhiVec = PhiVec + dx;

        [A10,A21] = HomCoord(PhiVec,LVec);

        rCurr = A10*A21*[0;L2;1];
        pErr = norm(PDes-rCurr);

        j = j+1

        if j>1000 %interrupt if position not below threshold after 1000 iterations

            %run optimisation routine to minimise position error
 %           lambda = fminsearch(@(x) JacErr(x,PhiVec,PDes,LVec),[0;0;0]);
 %           PhiVec = PhiVec + lambda;
            break
        end
    end
end

%% Calculate Jacobian
function J = Jac(phiVec,lVec)
    
phi1 = phiVec(1);
phi2 = phiVec(2);

l1 = lVec(1);
l2 = lVec(2);


% Jacobian
 J = [  -l1*cos(phi1) - l2*cos(phi1+phi2),  -l2*cos(phi1+phi2); ...
       -l1*sin(phi1)-l2*sin(phi1+phi2), -l2*sin(phi1+phi2)];   
end

%% Objective function error minimisation
function pErr = JacErr(lambda,phiVec,pDes,lVec)

l2 = lVec(2);
phi = phiVec + lambda;
[A10,A21] = HomCoord(phi,lVec);
rCurr = A10*A21*[0;l2;1];
pErr = norm(pDes-rCurr); 
end

end

