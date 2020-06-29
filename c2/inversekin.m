x0=0.15;
y0=0.05;
l1=0.118;
l2=0.122;
l3=l2;
l4=l1;

l1=0.1;
l2=l1;
l3=l1;
l4=l1;
xs(1)=0;
ys(1)=0;
zs(1)=0;
tt=0;

maxtheta=180;

count=1;
for phi = 140:0.25:320

    syms x y z
    Eq1 = l1*sind(tt) + l2*sind(x+tt)+l3*sind(x+y+tt)+l4*sind(y+z+x+tt) == x0 ;
    Eq2 = l1*cosd(tt) + l2*cosd(x+tt)+l3*cosd(x+y+tt)+l4*cosd(y+z+x+tt) == y0 ;
    Eq3 = y + z + x +tt== phi;

    result = vpasolve(Eq1,Eq2,Eq3);
    s = size(result.x);
    for sol = 1:s(1)
        tempx =result.x(sol);
        tempy =result.y(sol);
        tempz =result.z(sol);
        if abs(tempx)<= maxtheta & abs(tempy)<=maxtheta & abs(tempz)<=maxtheta
            xs(count)=tempx;
            ys(count)=tempy;
            zs(count)=tempz;
            count=count+1
        end
        
    end  
end

scatter3(xs,ys,zs);


