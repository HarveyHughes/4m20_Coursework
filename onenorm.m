count = 90;
start = [30,120,180];
l1_min=1000

for i = 1:19
    l1_sum=abs(start(1)-xs(i)) +abs(start(2)-ys(i)) + abs(start(3)-zs(i))
    if l1_sum < l1_min
        l1_min=l1_sum;
        index = i;
        
    end
    
end
index
xs(index)
ys(index)
zs(index)