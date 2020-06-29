x0=-0.9919;
y0=0.6986;
l1=0.8;
l2=0.8;
l3=0.3;

xs(1)=0;
ys(1)=0;
zs(1)=0;

count=1;
for phi = 0:0.25:360

    syms x y z
    Eq1 = -l1*sind(x)+l2*sind(y-x)+l3*sind(y+z-x) == x0 ;
    Eq2 = l1*cosd(x)+l2*cosd(y-x)+l3*cosd(y+z-x) == y0 ;
    Eq3 = y + z - x == phi;

    result = vpasolve(Eq1,Eq2,Eq3);
    s = size(result.x);
    for sol = 1:s(1)
        tempx =result.x(sol);
        tempy =result.y(sol);
        tempz =result.z(sol);
        if abs(tempx)<= 90 & abs(tempy)<=160 & abs(tempz)<=160
            xs(count)=tempx;
            ys(count)=tempy;
            zs(count)=tempz;
            count=count+1
        end
        
    end  
end

scatter3(xs,ys,zs);


