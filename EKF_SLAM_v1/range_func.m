function out = range_func(x1,y1,x2,y2,std)
m = numel(x1);
for i = 1:1:m
    ran(:,i)=(sqrt((x1(i)-x2).^2 + (y1(i)-y2).^2)+std*randn(1,1);
    end
    out = ran;
end