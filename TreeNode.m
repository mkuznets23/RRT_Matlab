classdef TreeNode < handle
    properties
        Value
        Joints
        Number
        Children = TreeNode.empty(); % Children nodes, initialized as empty array
    end
    
    methods
        % Constructor
        function obj = TreeNode(value)
            obj.Value = value;
        end
        
        % Add child node
        function addChild(obj, childNode)
            obj.Children(end + 1) = childNode;
        end
        
        function removeChild(obj,childNode)
            obj.Children(obj.Children==childNode) = [];
        end
    end
end
