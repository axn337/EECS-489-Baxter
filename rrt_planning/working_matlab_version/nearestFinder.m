function nearest = nearestFinder(wholeSet, point)
%NEARESTFINDER Summary of this function goes here
%   Detailed explanation goes here
nearest = 0;
distance = 9999;
for i = 1:length(wholeSet)
    distanceTemp = distCalc(wholeSet{i},point);
    if distanceTemp < distance
        nearest = i;
        distance = distanceTemp;
    end
end

end

