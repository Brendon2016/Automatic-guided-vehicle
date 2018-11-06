classdef forklift < matlab.System
    % Untitled Add summary here
    % This is a kinematic model of forklift based on bicycle model
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    
    % state = [x ; y phi ; vd ; gamma];
    % control input u = [vd ; gamma];
    properties
        wheelBase = 0.25;    % Wheelbase [m]
        dt = 0.001;          % Sample time [s]
        dev_vd = 0.01;       % deviation of vd [m]
        dev_gamma = 0.01;    % deviation of gamma [rad]
        accVdMax = 10;       % max acceleration of forward velocity
        accGammaMax = pi*5;  % max accelaration of steer velocity 
    end

    properties(DiscreteState)
        x;                  % [pose vd gamma]
    end

    % Pre-computed constants
    properties(Access = private)
        initState;
    end

    methods
        function obj = forklift(wheelBase , dev , dt , initState)
            obj.wheelBase = wheelBase;
            obj.dt = dt;
            obj.dev_vd = dev(1);
            obj.dev_gamma = dev(2);
            obj.initState = initState;
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.x = zeros(5 , 1);
        end

        function y = stepImpl(obj ,u)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            y = [0 0 0]';
            vd = u(1) + randn(1) * obj.dev_vd;
            gamma = u(2) + randn(1) * obj.dev_gamma;
            if (vd - obj.x(4)) > obj.accVdMax * obj.dt
                vd = obj.x(4) + obj.accVdMax * obj.dt;
            elseif (vd - obj.x(4)) < -obj.accVdMax * obj.dt
                vd = obj.x(4) - obj.accVdMax * obj.dt;
            end
            if (gamma - obj.x(5)) > obj.accGammaMax * obj.dt
                gamma = obj.x(5) + obj.accGammaMax * obj.dt;
            elseif (gamma - obj.x(5)) < -obj.accGammaMax * obj.dt
                gamma = obj.x(5) - obj.accGammaMax * obj.dt;
            end
            
            if(abs(gamma) <= 0.000001)
                y(3) = obj.x(3);
                dl = vd * obj.dt;
                y(1) = obj.x(1) + dl*cos(obj.x(3));
                y(2) = obj.x(2) + dl*sin(obj.x(3));
                r = 0;
                w = 0;
            else
                r = obj.wheelBase /tan(gamma);
                w = vd * sin(gamma) / obj.wheelBase;

                y(3) = obj.x(3) + w * obj.dt;
                cx = obj.x(1) - r*sin(obj.x(3));
                cy = obj.x(2) + r*cos(obj.x(3));
                y(1) = cx + r*sin(y(3));
                y(2) = cy - r*cos(y(3));
            end
            y = [y ; vd ; gamma];
            obj.x = y;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.x = obj.initState;
        end

        function s = infoImpl(obj)
            % Return structure of information about object obj
            s = struct('state' , obj.x , ...
                        'deviation' , [obj.dev_vd obj.dev_gamma]' , ...
                        'wheelbase' , obj.wheelBase , ...
                        'sampletime' , obj.dt);
        end
    end
end
