function posV = polar2Cart(dist)
%UNTITLED2 convert polar coordinate to cartesian coordinate
n = length(dist);
j = 1;
for i = (2:n)
    r = dist(1,i)/1000; % mm to m
    if (r ~= 0)
        ang = (i-1)*pi/180; % angle in radian
        x = r*cos(ang);
        y = r*sin(ang);
        posV(j, 1:2) = [x y];
        j = j+1;
    end 
end

