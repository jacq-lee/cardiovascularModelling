classdef Circulation
    
    %     Model of systemic circulation from Ferreira et al. (2005), A Nonlinear State-Space Model
    %     of a Combined Cardiovascular System and a Rotary Pump, IEEE Conference on Decision and Control.
    
    properties
        HR {mustBeNumeric}
        Emax  {mustBeNumeric}
        Emin  {mustBeNumeric}
        
        non_slack_blood_volume  {mustBeNumeric}
        R1  {mustBeNumeric}
        R2  {mustBeNumeric}
        R3  {mustBeNumeric}
        R4  {mustBeNumeric}
        C2  {mustBeNumeric}
        C3  {mustBeNumeric}
        L  {mustBeNumeric}
        
        tc {mustBeNumeric}
        Tmax {mustBeNumeric}
    end
    
    methods
        
        function obj = Circulation(HR_, Emax_, Emin_)
            if nargin == 3
                obj.HR = HR_;
                obj.tc = 60/HR_;
                obj.Tmax = 0.2+0.15*obj.tc; % # contraction time
                
                obj.Emax = Emax_;
                obj.Emin = Emin_;
                
                obj.non_slack_blood_volume = 250; %# ml
                obj.R1 = 1.0; %# between 0.5 and 2
                obj.R2 = 0.005;
                obj.R3 = 0.001;
                obj.R4 = 0.0398;
                
                obj.C2 = 4.4;
                obj.C3 = 1.33;
                
                obj.L = 0.0005;
            end
        end
        
        %%% TASK 2
        function [state_derivatives] = get_derivative(obj, t, x)
            % This function implements the state equation x_dot = A * x
            % and it should work for all phases of the cardiac cycle

            % Inputs
            % t: time
            % x: state variables [ventricular pressure; atrial pressure; arterial pressure; aortic flow]
            
            % Output
            % time derivatives of state variables: A*x
            
            % WRITE YOUR CODE HERE for Task 2
            %  Implement this by deciding whether the model is in a filling, ejecting, or isovolumic phase and using
            %  the corresponding dynamic matrix.

            if (x(2) > x(1)) % Complete for filling
                A = obj.filling_phase_dynamic_matrix(t);
                
            elseif (x(1) > x(3)) %|| something! % Complete for ejection
                A = obj.ejection_phase_dynamic_matrix(t);
            else % isovolumetric
                A = obj.isovolumic_phase_dynamic_matrix(t);
            end
            
            state_derivatives = transpose(sum(A));

            
            %%% End of Write Code for Task 2
        end
        
        function [A] = isovolumic_phase_dynamic_matrix(obj, t)
            % Inputs
            % t: time (s; needed because elastance is a function of time)
            
            % Output
            % A matrix for isovolumic phase
            
            el = obj.elastance(t);
            del_dt = obj.elastance_finite_difference(t);
            A = [del_dt/el, 0, 0, 0
                0, -1/(obj.R1*obj.C2), 1/(obj.R1*obj.C2), 0
                0, 1/(obj.R1*obj.C3), -1/(obj.R1*obj.C3), 0
                0, 0, 0, 0];
        end
        
        function [A] = ejection_phase_dynamic_matrix(obj, t)
            % Inputs
            % t: time (s)
            
            % Output
            % A matrix for filling phase
            
            el = obj.elastance(t);
            del_dt = obj.elastance_finite_difference(t);
            A = [del_dt/el, 0, 0, -el
                0, -1/(obj.R1*obj.C2), 1/(obj.R1*obj.C2), 0
                0, 1/(obj.R1*obj.C3), -1/(obj.R1*obj.C3), 1/obj.C3
                1/obj.L, 0, -1/obj.L, -(obj.R3+obj.R4)/obj.L];
        end
        
        %%% TASK 1
        function [A] = filling_phase_dynamic_matrix(obj, t)
            % Inputs
            % t: time (s)
            
            % Output
            % A matrix for filling phase
            
            % WRITE YOUR CODE HERE
            el = obj.elastance(t);
            del_dt = obj.elastance_finite_difference(t);
            A = [(del_dt/el) - el/obj.R2, el/obj.R2, 0, 0
                1/(obj.R2*obj.C2), -(obj.R1+obj.R2)/(obj.C2*obj.R1*obj.R2), 1/(obj.R1*obj.C2), 0
                0, 1/(obj.R1*obj.C3), -1/(obj.R1*obj.R3), 0
                0, 0, 0, 0];

        end
        
        function [time_varying_elastance] = elastance(obj, t)
            % Inputs
            % t: time (needed because elastance is a function of time)
            
            % Output
            % time-varying elastance
            
            tn = obj.get_normalized_time(t);
            En = 1.55 * power(tn/0.7, 1.9) / (1 + power(tn/0.7, 1.9)) / (1 + power(tn/1.17, 21.9));
            time_varying_elastance = (obj.Emax-obj.Emin)*En + obj.Emin;
        end
        
        function [derivative_approximation] = elastance_finite_difference(obj, t)
            %         Calculates finite-difference approximation of elastance derivative. In class I showed another method
            %         that calculated the derivative analytically, but I've removed it to keep things simple.
            
            % Inputs
            % t: time (needed because elastance is a function of time)
            
            % Output
            % finite-difference approximation of time derivative of time-varying elastance
            
            dt = 0.0001;
            forward_time = t + dt;
            backward_time = max(0, t - dt); % small negative times are wrapped to end of cycle
            forward = obj.elastance(forward_time);
            backward = obj.elastance(backward_time);
            derivative_approximation = (forward - backward) / (2*dt);
        end
        
        %%% TASK 3
        function [time, y] = simulate(obj, total_time)
            % Inputs
            % total_time: seconds to simulate
            
            % Output
            % time, state (times at which the state is estimated, state vector at each time)
            
            % WRITE  YOUR CODE HERE
            % Put all the blood in the atria as an initial condition.
            f = @(t, x) get_derivative(obj, total_time, x);
            
            tspan = [0 total_time];
            initial_conditions = [0, obj.non_slack_blood_volume/obj.C2, 0, 0];
            
            [time, y] = ode45(f, tspan, initial_conditions);          
            
        end
        
        function [normalized_time] = get_normalized_time(obj, t)
            % Inputs
            % t: time
            
            % Output
            % time normalized to obj.Tmax (duration of ventricular contraction)
            
            normalized_time = rem(t, obj.tc) / obj.Tmax;
        end
        
    end
    
end