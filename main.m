close all
clear
% rng("default")

%% SETUP
sweepCount = 30;
maxNodeTimeout = 5000;

%robot arm (mm and degrees)
L = [500,300,100,50];
thetas = [20,5,5];

goal_thetas = [160,5,5];
goal_pos = endEffectorPos(L,goal_thetas);

%obstacle
obstacle.radius = 42;
obstacle.center = [0,700];
obstacle2.radius = 35;
obstacle2.center = [575, 575];
obstacle3.radius = 55;
obstacle3.center = [-600, 600];
obstacle4.radius = 8;
obstacle4.center = [800, 350];
obstacles = {obstacle,obstacle2,obstacle3,obstacle4};

%store all the parents and children
parents = TreeNode.empty();
children = TreeNode.empty(); %one edge per child, so this is effectively a way to organize edges
sweep_points = {};
sweep_joints = {};

% Create nodes
NodeCounter = 0;
start_pos = endEffectorPos(L,thetas);
root = TreeNode(start_pos);
root.Joints = thetas;
root.Number = NodeCounter;

goalFound = 0; %flag for if goal is located by tree
collidedNodes = 0; %track how many randomly placed nodes of nodeCount collide with object

prevThetas = thetas;
prevNewNode_val = start_pos;
prevNode = root;

%% LOOP
for nodeCount = 1:maxNodeTimeout
    coin = 100*rand();
    
    if coin <50
        thetas = [180*rand(),180*rand()-90,180*rand()-90];
        newNode_val = endEffectorPos(L,thetas);

        if isempty(children) %if there are no children (just root)
            %case when no edges yet (first node after root added)

            %here we're checking if the node collides with obstacle
            %and move it if it does it gets moved toward the nearest point on tree
            %until there is no longer any collision
            newNode_val = preventCollision(obstacles,start_pos,newNode_val);

            [swept,joints,collisionFlag] = sweepArm(L,root.Joints,thetas,obstacles,sweepCount);
            if collisionFlag == 0
                sweep_points{end+1} = swept;
                sweep_joints{end+1} = joints;
                newNode = TreeNode(newNode_val);
                newNode.Joints = thetas;
                root.addChild(newNode);
                parents(end+1) = root;
                children(end+1) = newNode; 
            else
                collidedNodes = collidedNodes+1;

                continue; %to next random node
            end
        else
            %find index of nearest edge and the corresponding point where it
            %connects
            [smallestDistanceIndex,nearestPoint] = findNearestEdge(children,parents,newNode_val);

            newNode_val = preventCollision(obstacles,nearestPoint, newNode_val);

            newNode = TreeNode(newNode_val);
            NodeCounter = NodeCounter + 1;
            newNode.Joints = thetas;
            newNode.Number = NodeCounter;

            if (nearestPoint == children(smallestDistanceIndex).Value)

                %sweep from child node of edge to new node
                [swept,joints,collisionFlag] = sweepArm(L,children(smallestDistanceIndex).Joints,thetas,obstacles,sweepCount);
                if collisionFlag==0
                    sweep_points{end+1} = swept;
                    sweep_joints{end+1} = joints;
                    %simply add as a new node with selected child as parent
                    children(smallestDistanceIndex).addChild(newNode);
                    parents(end+1) = children(smallestDistanceIndex);
                    children(end+1) = newNode;
                else
                    collidedNodes = collidedNodes+1;
                    continue; %to next random node
                end

            elseif (nearestPoint == parents(smallestDistanceIndex).Value)

                %sweep from parent node of edge to new node
                [swept,joints,collisionFlag] = sweepArm(L,parents(smallestDistanceIndex).Joints,thetas,obstacles,sweepCount);
                if collisionFlag==0
                    sweep_points{end+1} = swept;
                    sweep_joints{end+1} = joints;
                    %simply add as a new node with selected parent as parent
                    parents(smallestDistanceIndex).addChild(newNode);
                    parents(end+1) = parents(smallestDistanceIndex);
                    children(end+1) = newNode;

                else
                    collidedNodes = collidedNodes+1;
                    continue; %to next random node
                end
            else
                %this is when the new node projects a new node in the middle of an existing edge
                %Now find the closest swept point of previous edge to the new
                %node
                minSweepDistance = 10000;
                minSweepIndex = 0;
                previous_sweep = sweep_points{smallestDistanceIndex};
                for i = 1:sweepCount
                    sweep_point = [previous_sweep(1,i),previous_sweep(2,i)];
                    %check distance from the new node to the sweep of edge node is
                    %attached to
                    dist = sqrt((sweep_point(1)-newNode_val(1))^2 + (sweep_point(2)-newNode_val(2))^2);
                    if dist < minSweepDistance
                        minSweepIndex = i; 
                        minSweepDistance=dist;
                    end    
                end


                nearestPreviousSweepAngles = sweep_joints{smallestDistanceIndex}(:,minSweepIndex);
                [swept,joints,collisionFlag] = sweepArm(L,nearestPreviousSweepAngles,thetas,obstacles,sweepCount);

                if collisionFlag==0
                    %create a new point in edge and reassign parents and children to
                    %accomodate it
                    newNode2 = TreeNode(nearestPoint);
                    NodeCounter = NodeCounter + 1;
                    newNode2.Number = NodeCounter;
                    parent = parents(smallestDistanceIndex);
                    child = children(smallestDistanceIndex);
                    parent.removeChild(child);
                    parent.addChild(newNode2);
                    newNode2.addChild(child);
                    newNode2.addChild(newNode);
                    newNode2.Joints = nearestPreviousSweepAngles;

                    %add new edges (ORDER MATTERS)

                    [swept1,joints1,~] = sweepArm(L,parent.Joints,nearestPreviousSweepAngles,obstacles,sweepCount);
                    parents(end+1) = parent;
                    children(end+1) = newNode2;
                    sweep_points{end+1} = swept1;
                    sweep_joints{end+1} = joints1;

                    [swept2,joints2,~] = sweepArm(L,nearestPreviousSweepAngles,child.Joints,obstacles,sweepCount);
                    parents(end+1) = newNode2;
                    children(end+1) = child;
                    sweep_points{end+1} = swept2;
                    sweep_joints{end+1} = joints2;

                    parents(end+1) = newNode2;
                    children(end+1) = newNode;
                    sweep_points{end+1} = swept;
                    sweep_joints{end+1} = joints;

                    %remove old edge
                    removalArray = logical((children == child).*(parents == parent));
                    parents(removalArray)=[];
                    sweep_points(removalArray) = [];
                    sweep_joints(removalArray) = [];
                    children(removalArray)=[];
                else
                    collidedNodes = collidedNodes+1;
                    continue; %to next random node
                end

            end
            prevThetas = thetas;
            prevNewNode_val = newNode_val;
            prevNode = newNode;
        end

    else
        %try to plot a linear path to goal from new node
        inLineCollision = 0;
        for i = 1:length(obstacles)
                obstacleCenter = obstacles{i}.center;
                obstacleRadius = obstacles{i}.radius;

                distanceToObstacleCenter = pointToLineSegmentDistance(obstacleCenter, goal_pos, prevNewNode_val);
                if distanceToObstacleCenter < obstacleRadius %collision occurs along edge
                    inLineCollision = 1;
                end
        end

        %sweep arm through its configurations from new node to goal and see if
        %any obstacle collision happens
        if inLineCollision == 0
            [swept, joints, collisionFlag] = sweepArm(L,prevThetas,goal_thetas,obstacles,sweepCount);

            if collisionFlag == 0
                goalNode = TreeNode(goal_pos);
                prevNode.addChild(goalNode);
                parents(end+1) = prevNode;
                children(end+1) = goalNode;
                sweep_points{end+1} = swept;
                sweep_joints{end+1} = joints;
                goalFound = 1;
                break;
            else
                continue;
            end

        end
    end
end

%% OBTAIN BRANCH PATH
if goalFound
    pathIndices = []; %placed on the parents of each edge
    currentNode = children(end); %start at second to last node (right before goal)
    while currentNode ~= root
        nextNodeIndex = find(children == currentNode);
        pathIndices = [pathIndices nextNodeIndex];
        %now need to find the parent of this new node
        %by searching for it in the children array
        currentNode = parents(nextNodeIndex);
    end
end


%% PLOTTING
figure
hold on

if goalFound
    for edges = pathIndices
        for i = 1:sweepCount
            plotArm(L,sweep_joints{edges}(:,i),'green',0.1)
%             endPos = endEffectorPos(L,sweep_joints{edges}(:,i));
%             plot(endPos(1),endPos(2),'b.')
        end
    end
    for i = pathIndices
        plot(children(i).Value(1),children(i).Value(2),'*b')
    end
end

for i = 1:length(obstacles)
    obstacleCenter = obstacles{i}.center;
    obstacleRadius = obstacles{i}.radius;
    rectangle('Position',[obstacleCenter(1)-obstacleRadius ...
    obstacleCenter(2)-obstacleRadius obstacleRadius*2 obstacleRadius*2]...
    ,'Curvature',[1,1],'FaceColor','red');
end
daspect([1,1,1])
plotTree(root, start_pos(1), start_pos(2));

%robot arm
%reset angles to original for plotting
thetas = [20,5,5];
plotArm(L,thetas,'black',1)
plot(goal_pos(1),goal_pos(2), '*r')
text(goal_pos(1),goal_pos(2), 'Goal')
text(start_pos(1),start_pos(2), 'Start')
hold off


figure
hold on
if goalFound
    for edges = pathIndices
        for i = 1:sweepCount
%             plotArm(L,sweep_joints{edges}(:,i),'green',0.1)
            endPos = endEffectorPos(L,sweep_joints{edges}(:,i));
            plot(endPos(1),endPos(2),'b.')
        end
    end
end

for i = 1:length(obstacles)
    obstacleCenter = obstacles{i}.center;
    obstacleRadius = obstacles{i}.radius;
    rectangle('Position',[obstacleCenter(1)-obstacleRadius ...
    obstacleCenter(2)-obstacleRadius obstacleRadius*2 obstacleRadius*2]...
    ,'Curvature',[1,1],'FaceColor','red');
end
daspect([1,1,1])
%robot arm
%reset angles to original for plotting
thetas = [20,5,5];
plotArm(L,thetas,'black',1)
plot(goal_pos(1),goal_pos(2), '*r')
text(goal_pos(1),goal_pos(2), 'Goal')
text(start_pos(1),start_pos(2), 'Start')
hold off

figure
hold on
if goalFound
    for edges = pathIndices
        for i = 1:sweepCount
            plotArm(L,sweep_joints{edges}(:,i),'green',0.1)
        end
    end
end

for i = 1:length(obstacles)
    obstacleCenter = obstacles{i}.center;
    obstacleRadius = obstacles{i}.radius;
    rectangle('Position',[obstacleCenter(1)-obstacleRadius ...
    obstacleCenter(2)-obstacleRadius obstacleRadius*2 obstacleRadius*2]...
    ,'Curvature',[1,1],'FaceColor','red');
end
daspect([1,1,1])
%robot arm
%reset angles to original for plotting
thetas = [20,5,5];
plotArm(L,thetas,'black',1)
plot(goal_pos(1),goal_pos(2), '*r')
text(goal_pos(1),goal_pos(2), 'Goal')
text(start_pos(1),start_pos(2), 'Start')
hold off


figure
hold on
for i = 1:length(obstacles)
    obstacleCenter = obstacles{i}.center;
    obstacleRadius = obstacles{i}.radius;
    rectangle('Position',[obstacleCenter(1)-obstacleRadius ...
    obstacleCenter(2)-obstacleRadius obstacleRadius*2 obstacleRadius*2]...
    ,'Curvature',[1,1],'FaceColor','red');
end
daspect([1,1,1])
plotTree(root, start_pos(1), start_pos(2));

%robot arm
%reset angles to original for plotting
thetas = [20,5,5];
plotArm(L,thetas,'blue',1)
plot(goal_pos(1),goal_pos(2), '*r')
plot(start_pos(1),start_pos(2), '*r')
text(goal_pos(1),goal_pos(2), 'Goal')
text(start_pos(1),start_pos(2), 'Start')
hold off





