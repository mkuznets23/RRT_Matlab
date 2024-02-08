function pos = endEffectorPos(L,thetas)
    x = L(1)*cosd(thetas(1))+L(2)*cosd(thetas(1)+thetas(2))+L(3)*cosd(thetas(1)+thetas(2)+thetas(3));
    y = L(1)*sind(thetas(1))+L(2)*sind(thetas(1)+thetas(2))+L(3)*sind(thetas(1)+thetas(2)+thetas(3));
    pos = [x,y];
end

