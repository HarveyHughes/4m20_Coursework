function draw_robot(t,l1,l2,l3,l4,col)
    figure(2)
    x1=l1*sind(t(1));
    y1=l1*cosd(t(1));
    
    x2=x1+l2*sind(t(1)+t(2)); % x2
    y2=y1+l2*cosd(t(1)+t(2));
    
    x3=x2 + l3*sind(t(1)+t(2)+t(3)); % x3 the middle
    y3=y2+l3*cosd(t(1)+t(2)+t(3));
    
    x4=x3+l4*sind(t(1)+t(2)+t(3)+t(4)); % x3 the middle
    y4=y3+l4*cosd(t(1)+t(2)+t(3)+t(4));

    line([0 x1],[0 y1],[0 0],'color',col,'LineWidth',1);
    line([x1 x2],[y1 y2],[0 0],'color',col,'LineWidth',1);
    line([x2 x3],[y2 y3],[0 0],'color',col,'LineWidth',1);
    line([x3 x4],[y3 y4],[0 0],'color',col,'LineWidth',1);
end