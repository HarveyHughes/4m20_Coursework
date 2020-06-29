function draw_postions(stages)
%traj = [x1,y1,theta0, theta1, theta2, theta3]  Nx5 array
% the theta0 is the angle L1 makes with vertical, will be required as this
% could change especially when pivoting around xy5
%L = [L0 L1 L2 L3 L4 L5]
%T = time step
    global L0 L1 L2 L3 L4 L5
    
    
    L = [L0 , L1, L2, L3, L4, L5];

%     start=0.15+0.18; % position of the steps
%     draw_steps(start);
    line([-10 10],[0 0],'color','k','LineWidth',2);
    hold on;
    
    cols = ['k-','r-','b-','g-','o-']
%     T=walker.t(2)-walker.t(1);
    no = size(stages.gnd,2)
    for i = 1:no
        
%     stages.xy1(i) = stages.xy1(i) + i*0.2 ;   
%     stages.xy5(i) = stages.xy5(i) + i*0.2 ;
    [traj,foot] = walker_to_traj(stages,i);
    
    pos=fwd_kin(traj,L,foot);    
    xs= pos(1,:);
    ys=pos(2,:);
   
    plot(xs,ys, cols(i))
    hold on
    end
    axis equal;
    xlim([-0.1 1.5])
    ylim([-0.05 0.4])    
    
%     xlabel('x/m')
%     ylabel('y/m')
%      
    hold on;
    h=0.09;
    le=0.12;
    s= [0.555, 0.855, 1.155,1.355]
    for i = 1:length(s)
    line([s(i) s(i)],[0 h],'color','k','LineWidth',1);
    line([s(i) s(i)+le],[h h],'color','k','LineWidth',1);
    line([s(i)+le s(i)+le],[h 0],'color','k','LineWidth',1);
    end
end

function [traj,foot]= walker_to_traj(walker,i)
    foot = walker.gnd(i);
    if foot == 1
        xy = walker.xy1(:,i);
    else
        xy=walker.xy5(:,i);
    end
     x=xy(1);
     y=xy(2);
     ts = walker.th(:,i);
     t1=ts(1);
     t2=ts(2);
     t3=ts(3);
     traj = [x,y,t1,t2,t3];
     
end


function xyrobot = fwd_kin(p,L,foot)
%p is [ x1,y1 , t1 ,t2 ,t3]

if foot ~=1
    xy1= [p(1);p(2)] + [0; L(5)]+L(4)*[-sind(p(5)); cosd(p(5))]+ L(3)*[-sind(sum(p(4:5))); cosd(sum(p(4:5)))] + L(2)*[-sind(sum(p(3:5))); cosd(sum(p(3:5)))];
else
    xy1=[p(1);p(2)];  % position of left corner
end
xy2=xy1 + L(2)*[0; 1];  
xy3 = xy2 + L(3)*[sind(sum(p(2:3))); cosd(sum(p(2:3)))];
xy4 = xy3 + L(4)*[sind(sum(p(2:4))); cosd(sum(p(2:4)))];
xy5 = xy4 + L(5)*[sind(sum(p(2:5))); cosd(sum(p(2:5)))];
xy6 = xy5 + L(6)*[sind(sum(p(2:5))+90); cosd(sum(p(2:5))+90)];
xy0= xy1 + L(1)*[1; 0];

xyrobot = [xy0, xy1, xy2, xy3, xy4, xy5, xy6];
end



function draw_steps(s)

    h=[0.05,0.07,0.09];
    le=0.15;
    

    line([s s],[0 h(1)],'color','k','LineWidth',2);
    line([s s+le],[h(1) h(1)],'color','k','LineWidth',2);
    
    line([s+le s+le],[h(1) h(1)+h(2)],'color','k','LineWidth',2);
    line([s+le s+2*le],[h(1)+h(2) h(1)+h(2)],'color','k','LineWidth',2);
    
    line([s+2*le s+2*le],[h(1)+h(2) h(1)+h(2)+h(3)],'color','k','LineWidth',2);
    line([s+2*le s+3*le],[h(1)+h(2)+h(3) h(1)+h(2)+h(3)],'color','k','LineWidth',2);
    
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