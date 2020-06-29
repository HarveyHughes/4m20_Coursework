 x= [0.4:.001:0.6];
 y = normpdf(x,0.5,0.01);
y2 = normpdf(x,0.52,0.005);
y3 = normpdf(x,0.51,0.0075);
plot(x,y)
hold on
plot(x,y2)
hold on
plot(x,y3,'g')
legend('Model prediction','State measurment','Posteria update')
xlabel('Position/m')