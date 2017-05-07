function distance = distCalc(point1, point2)
%DISTCALC Summary of this function goes here
%   Detailed explanation goes here
difference = point2 - point1;
distance = 0;

for i = 1:length(difference)
    distance = distance + difference(i)^2;
end

distance = sqrt(distance);

end

