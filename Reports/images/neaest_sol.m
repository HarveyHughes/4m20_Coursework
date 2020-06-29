ts= 1.8;
te=2;
alpha=78;
norm=999999999999;
s=[0.4521 0.3974];

 g= 9.81;
    u=10;
    x0=3;
    y0=0;

   
for t=ts:0.01:te
    x=-u*cosd(alpha)*t+x0;
    y=u*sind(alpha)*t+0.5*-g*t^2+y0;
    
    if (y-s(2))^2+(x-s(1))^2 < norm
        norm = (y-s(2))^2+(x-s(1))^2 ;
        pos=[x,y];
    
    end
end
    