clear all

%Plotted in reverse:
startLoc = [0,0,0,0,0,0,0];
goalLoc = [1,1,1,1,1,1,1];

% startLoc = [-pi/2,-2.147,0,0,0,0,0];
% goalLoc = [0,0,0,0,0,0,0];

range_from=[-1.7016,-2.147,-3.0541,-0.05, -3.059,-1.5707,-3.059];
range_to=  [ 1.7016, 1.047, 3.0541, 2.618, 3.059, 2.094,  3.059];

max_dist = 0.01;
max_points = 20000;

graphPoints{1} = startLoc;
nearest_index(1) = 0;

if colCheck(startLoc) == true
    error('start location collided')
end

if colCheck(goalLoc) == true
    error('goal location collided')
end

current_points = length(graphPoints);

reachedGoal = false;
shortestDist = 99999;

while current_points <= max_points
    genRand = rand();
    if genRand < 0.90
        randomNum = rand(1,7);
        for i = 1:7 
            new_pointTemp(i) = randomNum(i)*abs(range_to(i)-range_from(i)) + range_from(i);
        end
    else
        new_pointTemp = goalLoc;
    end
    
    nearestIndex = nearestFinder(graphPoints,new_pointTemp);
    nearest_point = graphPoints{nearestIndex};
    if (distCalc(nearest_point, new_pointTemp) < max_dist)
        new_point = new_pointTemp;
    else
        new_point = newPointGen(new_pointTemp, graphPoints{nearestIndex},max_dist);
    end
    if colCheck(new_point) == true
        disp('New point collided');
    end
    if colCheck(new_point) == false
        edgeCollide = edge_check(nearest_point, new_point);
        if edgeCollide == true
            disp('Path collided');
        end
        if edgeCollide == false
            graphPoints{current_points+1} = new_point;
            nearest_index(current_points+1) = nearestIndex;
            dispString = strcat('Added point: ', num2str(new_point),' Nearest: ', num2str(nearestIndex));
            disp(dispString)
            disp(num2str(current_points))
            current_points = current_points + 1;
        end
        
        distanceToGoal = distCalc(new_point, goalLoc);
        if distanceToGoal < shortestDist
            shortestDist = distanceToGoal;
        end
        
        if distanceToGoal < max_dist
            disp('Close to goal')
            edgeCollide = edge_check(new_point, goalLoc);
            if edgeCollide == false
                graphPoints{current_points+1} = goalLoc;
                nearest_index(current_points+1) = current_points;
                reachedGoal = true;
                break;
            end
        end
    end
end

if reachedGoal == false
    disp('Failed')
else
    disp('Reached goal')
    
    Eo(:,1) = [2;0.05;0.05];
    object{1} = [1 0 0 0;...
                 0 1 0 0.5;...
                 0 0 1 0;...
                 0 0 0 1];
    
    disp(graphPoints{length(graphPoints)})
    indexCheck = nearest_index(length(graphPoints));
    while indexCheck ~= 0
        kinematicsPlotter(graphPoints{indexCheck})
        disp(graphPoints{indexCheck})
        indexCheck = nearest_index(indexCheck);
        pause(0.1);
    end
end