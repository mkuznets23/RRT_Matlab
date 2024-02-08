function plotArm(L,thetas,color,width)
    line([0 L(1)*cosd(thetas(1))],[0 L(1)*sind(thetas(1))], 'Color', color,'LineWidth',width) %L1hold on
    line([L(1)*cosd(thetas(1)) L(1)*cosd(thetas(1))+L(2)*cosd(thetas(1)+thetas(2))],...
        [L(1)*sind(thetas(1)) L(1)*sind(thetas(1))+L(2)*sind(thetas(1)+thetas(2))], 'Color', color,'LineWidth',width) %L2
    endPos = endEffectorPos(L,thetas);
    line([L(1)*cosd(thetas(1))+L(2)*cosd(thetas(1)+thetas(2)) endPos(1)],...
        [L(1)*sind(thetas(1))+L(2)*sind(thetas(1)+thetas(2)) endPos(2)],...
        'Color', color,'LineWidth',width) %L3
    L4_right = [L(4)/2*cosd(90-sum(thetas))+endPos(1) -L(4)/2*sind(90-sum(thetas))+endPos(2)];
    L4_left = [-L(4)/2*cosd(90-sum(thetas))+endPos(1) L(4)/2*sind(90-sum(thetas))+endPos(2)];
    line([L4_right(1) L4_left(1)],[L4_right(2) L4_left(2)],'Color', color,'LineWidth',width) %L4
end

