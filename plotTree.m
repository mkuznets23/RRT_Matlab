function plotTree(node, x, y)
    % Plot current node
    plot(x, y, 'b');
    
    % Plot branches
    if ~isempty(node.Children)
        numChildren = length(node.Children);
        
        for i = 1:numChildren
            childX = node.Children(i).Value(1);
            childY = node.Children(i).Value(2);
           
            % Draw a line connecting parent and child nodes
            plot([x, childX], [y, childY], 'k-','LineWidth',0.01);
            
            % Recursively plot child nodes
            plotTree(node.Children(i), childX, childY);
        end
    end
end