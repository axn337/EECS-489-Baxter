function collideCheck = edge_check(from_angles, to_angles)
%EDGE_CHECK Summary of this function goes here
%   Detailed explanation goes here
collideCheck = false;
increment = 10;
distance = distCalc(to_angles, from_angles);
unitDir = (to_angles-from_angles)/distance;
for i = 1:10
    testPoint = from_angles + unitDir*distance/increment*i;
    collideCheck = colCheck(testPoint);
    if collideCheck == true
        return;
    end
end

end
