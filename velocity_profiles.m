startp = [30 120 80];
endp = [-25.65,5.85,8.256];
endp=[88.4947,50.8256,119.1692];
diff = endp - startp;
T=0.289;
T=1.92;
%step(T,diff(3));
%trapezoid_optimised(T,diff(3))
%trapezoid(T,diff(3),0.075)

line([0 0],[0 0],'color','r','LineWidth',1);
line([0 0],[0 0],'color','g','LineWidth',1);
line([0 0],[0 0],'color','b','LineWidth',1);
a_t= trapezoid_optimise_sum(T,diff(3),'b')
a_t= trapezoid_optimise_sum(T,diff(2),'g')
a_t= trapezoid_optimise_sum(T,diff(1),'r')



vstep_max=diff*2/T
astep_min=diff*2/T^2

% for dim = 1:3
%     a_t= trapezoid_optimise_sum(T,diff(dim))
%     data
%     evaluate_vp(a_t,t)
% end


function step(T,distance)
    max=distance/T
    c='r';
    line([0 0],[0 max],'color',c,'LineWidth',1);
    line([0 T],[max max],'color',c,'LineWidth',1);
    line([T T],[max 0],[0 0],'color',c,'LineWidth',1);
end


%time is effectivly shorted to T- v/a , at speed v
function trapezoid_optimised(T,distance) %this just makes triangles , it reduces the acceleration
    c='b';
    v_max=2*distance/T;
    a=2*v_max/T;
    acc_time=v_max/a; 
    
    ed = T-acc_time;
    
    %vT-v^2/a = distance; %we want to minimise the max velocity as this also minimises the 
    
    line([0 acc_time],[0 v_max],'color',c,'LineWidth',1);
    line([acc_time ed],[v_max v_max],'color',c,'LineWidth',1);
    line([ed T],[v_max 0],'color',c,'LineWidth',1);

end

function trapezoid(T,distance,acc_time) %this just makes triangles , it reduces the acceleration
    c='g';
    v_max=distance/(T-acc_time);
    ed = T-acc_time;
    
    %vT-v^2/a = distance; %we want to minimise the max velocity as this also minimises the 
    
    line([0 acc_time],[0 v_max],'color',c,'LineWidth',1);
    line([acc_time ed],[v_max v_max],'color',c,'LineWidth',1);
    line([ed T],[v_max 0],'color',c,'LineWidth',1);

end



function a_t = trapezoid_optimise_sum(T,distance,c) %this just makes triangles , it reduces the acceleration
    %c='k';
    sum = 10000000000000;
    for acc_time = 0.001:T/1000:T/2
        v_max=distance/(T-acc_time);
        a=v_max/acc_time; 
        if abs(a*T/2 + v_max) < sum
            sum=abs(a*T/2+v_max);
            a_t = acc_time;
        end
    end
    
    
    a_t
    v_max=distance/(T-a_t);
    ed = T-a_t;
    
    %vT-v^2/a = distance; %we want to minimise the max velocity as this also minimises the 
    
    line([0 a_t],[0 v_max],'color',c,'LineWidth',1);
    line([a_t ed],[v_max v_max],'color',c,'LineWidth',1);
    line([ed T],[v_max 0],'color',c,'LineWidth',1);
    
end
