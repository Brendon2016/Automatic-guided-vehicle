classdef LPF < matlab.System
    % Untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        sampleFrequency = 4000;         % sample frequency 
        bandWidth = 100;                % band width
    end

    properties(DiscreteState)
        x
    end

    % Pre-computed constants
    properties(Access = private)
        a = 0;
        b = 1;
    end

    methods
        function obj = LPF(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            obj.x = 0;
            w = 2 * pi * obj.bandWidth;
            obj.a = exp(-w / obj.sampleFrequency);
            obj.b = 1 - obj.a; 
            % Perform one-time calculations, such as computing constants
        end

        function y = stepImpl(obj,u)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            obj.x = obj.a * obj.x + obj.b * u;              % 状态更新
            y = obj.x;                      % 更新输出
        end

        function resetImpl(obj)
            obj.x = 0;
            % Initialize / reset discrete-state properties
        end

        function s = infoImpl(obj)
             s = struct('sampleFrenqucy',obj.sampleFrequency, ...
                 'bandWidth' , obj.bandWidth, ...
                 'a' , obj.a , ...
                 'b' , obj.b);
       end
    end
end
