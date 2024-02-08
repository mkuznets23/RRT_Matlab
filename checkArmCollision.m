%Check arm collision
function result = checkArmCollision(L,thetas,obstacleCenter,obstacleRadius)
    L1start = [0,0];
    L1end = [L(1)*cosd(thetas(1)), L(1)*sind(thetas(1))];
    L2start = L1end;
    L2end = [L(1)*cosd(thetas(1))+L(2)*cosd(thetas(1)+thetas(2)), L(1)*sind(thetas(1))+L(2)*sind(thetas(1)+thetas(2))];
    L3start = L2end;
    L3end = endEffectorPos(L,thetas);
    L4start = [L(4)/2*cosd(90-sum(thetas))+L3end(1) -L(4)/2*sind(90-sum(thetas))+L3end(2)];
    L4end = [-L(4)/2*cosd(90-sum(thetas))+L3end(1) L(4)/2*sind(90-sum(thetas))+L3end(2)];
    result = checkLinkCollision(L1start,L1end, obstacleCenter, obstacleRadius)...
        || checkLinkCollision(L2start,L2end, obstacleCenter, obstacleRadius)...
        || checkLinkCollision(L3start,L3end, obstacleCenter, obstacleRadius)...
        || checkLinkCollision(L4start,L4end, obstacleCenter, obstacleRadius);
    
end