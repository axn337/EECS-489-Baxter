function newPoint = newPointGen(newPointTemp, nearestPoint, dist)
%NEWPOINTGEN Summary of this function goes here
%   Detailed explanation goes here
distance = distCalc(newPointTemp, nearestPoint);
unitDir = (newPointTemp - nearestPoint)/distance;
newPoint = unitDir*dist + nearestPoint;


end

