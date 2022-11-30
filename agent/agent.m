classdef agent < handle
    properties
        Vertex = [];
        R = 0.3;
        Positon_center = zeros(2, 1);
        Yaw = 0;
        V = [];
        W = [];
        u1 = [];
        u2 = [];
    end
    methods
        
        function obj = agent(varargin)
            obj = parse_args(obj,varargin{:});
            
        end
        
%         function obj = 
    end
end
