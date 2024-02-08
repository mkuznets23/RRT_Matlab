function [swept, thetas_sweep, collisionFlag] = sweepArm(L,thetas1,thetas2,obstacles,sweepCount)
%swept output is points swept by end effector of form [X;Y] where X and Y
%are arrays
%thetas_sweep output ruturns the set of 3 thetas (as a column) that each swept point is at

    theta1_sweep = linspace(thetas1(1),thetas2(1),sweepCount);
    theta2_sweep = linspace(thetas1(2),thetas2(2),sweepCount);
    theta3_sweep = linspace(thetas1(3),thetas2(3),sweepCount);
    thetas_sweep = [theta1_sweep;theta2_sweep;theta3_sweep];
    
    X = zeros(1,sweepCount);
    Y = zeros(1,sweepCount);

    collisionFlag = 0;
    for i = 1:sweepCount
        endPos = endEffectorPos(L,thetas_sweep(:,i));
        X(i) = endPos(1);
        Y(i) = endPos(2);

        for j = 1:length(obstacles)
            obstacleCenter = obstacles{j}.center;
            obstacleRadius = obstacles{j}.radius;
            if checkArmCollision(L,thetas_sweep(:,i),obstacleCenter,obstacleRadius)
                collisionFlag = 1;
%                 plotArm(L,thetas_sweep(:,i),'green')
            end
        end     
    end
    swept = [X;Y];
end

