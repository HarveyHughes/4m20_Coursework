l1=0.8;
l2=0.8;
l3=0.3;
draw_position([2.377,102.5,137.9],l1,l2,l3);
%draw_position([43.49,127.9,37.63],l1,l2,l3);


function draw_position(t,l1,l2,l3)
    t
    l1
    l2
    l3
    x1=-l1*sind(t(1))
    y1=l1*cosd(t(1))
    
    x2=-l1*sind(t(1))+l2*sind(-t(1)+t(2));
    y2=l1*cosd(t(1))+l2*cosd(-t(1)+t(2));
    
    x3=-l1*sind(t(1))+l2*sind(-t(1)+t(2))+l3*sind(-t(1)+t(2)+t(3));
    y3=l1*cosd(t(1))+l2*cosd(-t(1)+t(2))+l3*cosd(-t(1)+t(2)+t(3));

    line([0 x1],[0 y1],[0 0],'color','k','LineWidth',2);
    line([x1 x2],[y1 y2],[0 0],'color','k','LineWidth',2);
    line([x2 x3],[y2 y3],[0 0],'color','k','LineWidth',2);
end