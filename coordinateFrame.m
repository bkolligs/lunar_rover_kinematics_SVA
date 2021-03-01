classdef coordinateFrame
    % coordinateFrame coordinate frame nodes for the pose tree
    %   Each instance of this class is a coordinate frame in the pose graph
    %   to be assemled into the kinematic tree
    
    properties
        % if root, parent is -1
        Parent_;
        % lists of nodes
        Children_;
        % the actual transformation
        Data_;
        
    end
    
    methods
        function obj = coordinateFrame(parent, children, data)
            %coordinateFrame Construct an instance of this class
            %   describe the node
            obj.Parent_ = parent;
            obj.Children_ = children;
            obj.Data_ = data;
        end
    end
end

