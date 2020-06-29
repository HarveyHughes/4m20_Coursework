function walker = cw2_main()

global L0 L1 L2 L3 L4 L5 thub thlb

%% Initialisation and Limits
%sim_type = 'real'
sim_type = 'CW';

switch sim_type
    case 'real'
        L0 = 0.155;
        L1 = 0.118;
        L2 = 0.153;
        L3 = 0.153;
        L4 = 0.118;
        L5 = 0.199;
        L5 = 0.125;
    case 'CW'
        L0 = 0.2;
        L1 = 0.1;
        L2 = 0.1;
        L3 = 0.1;
        L4 = 0.1;
        L5 = 0.2;
end

% joint actuation limits
thub = [151 157 145]';
thlb = [-29 -23  -35]';
th0 = [60; 60; 60];

%% start position
xy1start = [0; 0];
xy5start = [0.13; 0];
groundpoint = 1;
thstart = walker_inv_kin(th0, xy1start, xy5start, 1);
xyrobotstart  = walker_fw_kin(thstart, xy1start, 1);

tstart = 0;
tstep = 3;
tend = tstart + tstep;

%% Step Length
% max step length
maxstep_th = walker_inv_kin(th0, xy1start, (xy1start + [1;0]), 1);
maxstep_xyrobot  = walker_fw_kin(maxstep_th, xy1start, 1);
max_steplength = maxstep_xyrobot(1,5) - xy5start(1);

% specified step length
steplength =0.05; %0.105;

if steplength > max_steplength
    disp('step size too large! Using max step size instead')
    steplength = max_steplength;
end

%% Initialisation
walker.t = [];
walker.th = [];
walker.gnd = [];
walker.xy1 = [];
walker.xy5 = [];

%% approach first step
first_steps = 1;

for i = 1:first_steps
    [xy1end, xy5end, stages]...
        = step_stages(thstart, xy1start, xy5start, 0.05); %0.11 with a start of 0.18 works with our weird start spasm, with 170 degrees in the target orientation
    
    % joint trajectories
    [th, gnd, xy1, xy5, t] = joint_trajectories(stages, tstart, tend);
    
    % save data
    walker.th = [walker.th, th];
    walker.gnd = [walker.gnd, gnd];
    walker.xy1 = [walker.xy1, xy1];
    walker.xy5 = [walker.xy5, xy5];
    walker.t = [walker.t, t];
    tstart = tend;
    
    % iterate
    tend = tstart + tstep;
    xy1start = xy1end;
    xy5start = xy5end;
    thstart = th(:,end);
end

%% horizontal and vertical steps
steplengths = [0.10, 0.10, 0.10];
stepheights = [0.05, 0.07, 0.09];

no_steps = 0;
%while tend < 10
for i = 1:no_steps
    %% generate vertical step
    [xy1end, xy5end, stages]...
        = vstep_stages(thstart, xy1start, xy5start, steplengths(i), stepheights(i));
    
    % joint trajectories
    [th, gnd, xy1, xy5, t] = joint_trajectories(stages, tstart, tend);
    
    % save data
    walker.th = [walker.th, th];
    walker.gnd = [walker.gnd, gnd];
    walker.xy1 = [walker.xy1, xy1];
    walker.xy5 = [walker.xy5, xy5];
    walker.t = [walker.t, t];
    tstart = tend;
    
    % iterate
    tend = tstart + tstep;
    xy1start = xy1end;
    xy5start = xy5end;
    thstart = th(:,end);
    
    %% generate horizontal step
    [xy1end, xy5end, stages]...
        = step_stages(thstart, xy1start, xy5start, 0.05);
    
    % joint trajectories
    [th, gnd, xy1, xy5, t] = joint_trajectories(stages, tstart, tend);
    
    % save data
    walker.th = [walker.th, th];
    walker.gnd = [walker.gnd, gnd];
    walker.xy1 = [walker.xy1, xy1];
    walker.xy5 = [walker.xy5, xy5];
    walker.t = [walker.t, t];
    tstart = tend;
    
    % iterate
    tend = tstart + tstep;
    xy1start = xy1end;
    xy5start = xy5end;
    thstart = th(:,end);
end

end


function xyrobot = walker_fw_kin(th, xyground, groundpoint)
%% Forward kinematics of walker
global L1 L2 L3 L4

th1 = th(1);
th2 = th(2);
th3 = th(3);

%% which leg is on the ground?
if groundpoint == 1
    % xy1 on ground
    xy1 = xyground;
    xy2 = xy1 + [0; L1];
    xy3 = xy2 + L2*[sind(th1); cosd(th1)];
    xy4 = xy3 + L3*[sind(th1 + th2); cosd(th1 + th2)];
    xy5 = xy4 + L4*[sind(th1 + th2 + th3); cosd(th1 + th2 + th3)];
else
    % xy5 on ground
    xy5 = xyground;
    xy4 = xy5 + [0; L4];
    xy3 = xy4 + L3*[-sind(th3); cosd(th3)];
    xy2 = xy3 + L2*[-sind(th2 + th3); cosd(th2 + th3)];
    xy1 = xy2 + L1*[-sind(th3 + th2 + th1); cosd(th3 + th2 + th1)];
    xyrobot = [xy1, xy2, xy3, xy4, xy5];
    xy1;
end
xyrobot = [xy1, xy2, xy3, xy4, xy5];

end

function th = walker_inv_kin(thstart, xy1, xy5, groundpoint)
%% find inverse kinematics, usindg an fsolve non-linear solving algorithm
global thlb thub

fun = @(th_try)err_walker_inv_kin(xy1,xy5,th_try, groundpoint);

options = optimset('Display','off', 'TolFun', 1e-7);
th = lsqnonlin(fun, thstart, thlb, thub, options);

end

function err = err_walker_inv_kin(xy1, xy5, th, groundpoint)
%% solver function for inverse kinematics

% error between desired xy and current tried xy
if groundpoint == 1
    % find xy5
    xyfor = walker_fw_kin(th, xy1, groundpoint);
    err = xy5 - xyfor(:,5);
else
    % find xy1
    xyfor = walker_fw_kin(th, xy5, groundpoint);
    %     figure(2)
    %     clf
    %     plot(xyfor(1,:), xyfor(2,:), '-.o')
    err = xy1 - xyfor(:,1);
end
err2 = 180 - (th(1) + th(2) + th(3));

err = [err; err2];

end

function xy_mass = cent_mass(xyrobot)
%% find centre of mass of robot

m1 = 1;
m2 = 1;
m3 = 1;
m4 = 1;

m_total = m1 + m2 + m3 + m4;

xy_mass = 0.5*(...
    m1*(xyrobot(:,1) + xyrobot(:,2)) + ...
    m2*(xyrobot(:,2) + xyrobot(:,3)) + ...
    m3*(xyrobot(:,3) + xyrobot(:,4)) + ...
    m4*(xyrobot(:,4) + xyrobot(:,5)) ...
    )/m_total;

end

function is_topple = check_topple(xyrobot, xymass)
%% check whether robot is toppling
if xymass(1) > xyrobot(1,1) + 0.2 || xymass(1) < xyrobot(1,1)
    is_topple = 1;
else
    is_topple = 0;
end
end

function [xy1end, xy5end, stages] = step_stages(thstart, xy1start, xy5start, steplength)
%% Generate movement stages for taking a step of a certain size

%% Calculate Step Stages
%% A
xy1A = xy1start;
xy5A = xy5start;
thA = thstart;
gndA = 1;

%% B
xy1B = xy1A;
xy5B = xy5A + [0.25*steplength; 0.1];
thB = walker_inv_kin(thA, xy1B, xy5B,1);
gndB = 1;

%% C
xy1C = xy1A;
xy5C = xy5A + [steplength; 0];
thC = walker_inv_kin(thB, xy1C, xy5C,1);
%thC = thC-[0;0;10];
gndC = 1;

%% D
xy5D = xy5C;
xy1D = xy1C + [0.6*steplength; 0.1];
thD = walker_inv_kin(flip(thB), xy1D, xy5D,5);
gndD = 5;

%% E
xy5E = xy5D;
xy1E = xy1C + [steplength; 0];
thE = walker_inv_kin(thA, xy1E, xy5E,5);
gndE = 5;

xy1end = xy1E;
xy5end = xy5E;
% output as struct
stages.th = [thA, thB, thC, thD, thE];
stages.gnd = [gndA, gndB, gndC, gndD, gndE];
stages.xy1 = [xy1A, xy1B, xy1C, xy1D, xy1E];
stages.xy5 = [xy5A, xy5B, xy5C, xy5D, xy5E];
stages.timefraction = [0.25, 0.25, 0.25, 0.25];
end

function [xy1end, xy5end, stages] = vstep_stages(thstart, xy1start, xy5start, steplength, stepheight)
%% Generate movement stages for taking a step of a certain size

%% Calculate Step Stages
%% A
xy1A = xy1start;
xy5A = xy5start;
thA = thstart;
gndA = 1;

%% B
xy1B = xy1A;
xy5B = xy5A + [0; 0.150];
thB = walker_inv_kin(thA, xy1B, xy5B,1);
gndB = 1;

%% BC
xy1BC = xy1A;
xy5BC = xy5B + [0.9*steplength; 0];
thBC = walker_inv_kin(thB, xy1BC, xy5BC,1);
gndBC = 1;

%% C
xy1C = xy1A;
xy5C = xy5A + [steplength; stepheight];
thC = walker_inv_kin(thB, xy1C, xy5C,1);
gndC = 1;


%% D
xy5D = xy5C;
xy1D = xy1C + [0; stepheight + 0.05];
thD = walker_inv_kin(flip(thB), xy1D, xy5D,5);
gndD = 5;

%% E
xy5E = xy5D;
xy1E = xy1C + [steplength; stepheight];
thE = walker_inv_kin(thA, xy1E, xy5E,5);
gndE = 5;

xy1end = xy1E;
xy5end = xy5E;
% output as struct
stages.th = [thA, thB,thBC, thC, thD, thE];
stages.gnd = [gndA, gndB,gndBC, gndC, gndD, gndE];
stages.xy1 = [xy1A, xy1B, xy1BC, xy1C, xy1D, xy1E];
stages.xy5 = [xy5A, xy5B, xy5BC, xy5C, xy5D, xy5E];
stages.timefraction = [0.3, 0.1, 0.1, 0.3, 0.3];
end


function [th_t,gnd_t, xy1_t, xy5_t, t] = joint_trajectories(stages, tstart, tend)
%% fit polynomial trajectory to points specified by stages struc
%% initialise
t = [];
x = linspace(0,tend-tstart,size(stages.th,2));
th_t = [];
gnd_t = [];
xygnd_t = [];

% number of points
total_npoints = 100;

%% fit sequential third-order polynomials
for i = 1:length(x)-1
    thA = stages.th(:,i);
    thB = stages.th(:,i+1);
    xA = x(i);
    xB = x(i+1);
    gndB = stages.gnd(i+1);
    npoints = round(total_npoints*stages.timefraction(i));
    
    % define polynomial coeffs
    a0 = thA;
    a1 = [0;0;0];
    a2 = 3*(thB - thA)/((xB-xA)^2);
    a3 = -2*(thB - thA)/((xB-xA)^3);
    
    % define ground point leg and position
    xAB = linspace(xA, xB, npoints) - xA;
    % remove first point to avoid duplicate timesteps
    if i == 1
        xAB(1) = [];
    end
    gndAB = gndB*ones(1, size(xAB, 2));
    if gndAB(1) == 1
        xygndAB = stages.xy1(:,i+1).*ones(2, size(xAB,2));
    else
        xygndAB = stages.xy5(:,i+1).*ones(2, size(xAB,2));
    end
    
    % calculate polynomial values
    th_t = [th_t, (a3*(xAB.^3) + a2*(xAB.^2) + a1*(xAB) + a0)];
    gnd_t = [gnd_t, gndAB];
    xygnd_t = [xygnd_t, xygndAB];
    t = [t, (xAB + xA)];
end

% fit spline
% endslope = [0;0;0];
% pp = csape(x, [endslope,th_stages,endslope],[1 1]);
% th_t = fnval(pp, t);

% calculate fw kinematics
xy1_t = [];
xy5_t = [];
for i = 1:length(t)
    xyrobot = walker_fw_kin(th_t(:,i), xygnd_t(:,i), gnd_t(i));
    xy1_t = [xy1_t, xyrobot(:,1)];
    xy5_t = [xy5_t, xyrobot(:,5)];
end

t = t + tstart;
end