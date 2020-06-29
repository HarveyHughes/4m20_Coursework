

l1=0.8;
l2=0.8;
l3=0.3;

plot_region(l1,l2,l3);
alphas=[60,78]%,60,78];
trajectory(alphas);
% draw_position([2.377,102.5,137.9],l1,l2,l3,'k');
% draw_position([43.49,127.9,37.63],l1,l2,l3,'r');
% draw_position([42.58,148.3,-40.99],l1,l2,l3,'m');

% draw_position([-25.65,5.85,8.256],l1,l2,l3,'k');
draw_position([30,120,80],l1,l2,l3,'r');
draw_position([88.4947,50.8256,119.1692],l1,l2,l3,'k');

title('');
xlabel('x/m');
ylabel('y/m');
hold on
scatter3(-0.9919,0.6986,0)
function plot_region(l1,l2,l3)
    syms t1 t2
    for t3 = -160:10:160
        x=-l1*sind(t1)+l2*sind(-t1+t2)+l3*sind(-t1+t2+t3);
        y=l1*cosd(t1)+l2*cosd(-t1+t2)+l3*cosd(-t1+t2+t3);
        z=x*0;
        h=ezsurf(x,y,z,[-90,90,-160,160]);
        h.EdgeColor = 'none';
        hold on
    end
end


function trajectory(alphas)
    g= 9.81;
    u=10;
    x0=3;
    y0=0;
    colours=['m','g','r','k']
    syms t
    for i =1:length(alphas)
        hold on
        alpha=alphas(i)
        x=-u*cosd(alpha)*t+x0;
        y=u*sind(alpha)*t+0.5*-g*t^2+y0;
        h=ezplot(x,y,[0,2.037*sind(alpha)]);
        h.Color = colours(i)
    end
    title('Feasible Region & Trajectories');
    xlabel('x/m');
    ylabel('y/m');
    %legend('alpha=45','alpha=60','alpha=78');
end



function draw_position(t,l1,l2,l3,c)
  
    x1=-l1*sind(t(1));
    y1=l1*cosd(t(1));
    
    x2=-l1*sind(t(1))+l2*sind(-t(1)+t(2));
    y2=l1*cosd(t(1))+l2*cosd(-t(1)+t(2));
    
    x3=-l1*sind(t(1))+l2*sind(-t(1)+t(2))+l3*sind(-t(1)+t(2)+t(3));
    y3=l1*cosd(t(1))+l2*cosd(-t(1)+t(2))+l3*cosd(-t(1)+t(2)+t(3));

    line([0 x1],[0 y1],[0 0],'color',c,'LineWidth',2);
    line([x1 x2],[y1 y2],[0 0],'color',c,'LineWidth',2);
    line([x2 x3],[y2 y3],[0 0],'color',c,'LineWidth',2);
end



