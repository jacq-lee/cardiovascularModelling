function [E] = ObjectiveFunction(w, ventricularPressure_input, time_input)
% This is the objective function which returns the quantitative measure of
% the goodness of a solution (in this example, a solution is: w = (HR, E_max,
% E_min).

%%% INPUTS
% w: a vector which contains HR, E_max, E_min which are parameters we are looking for
% ventricularPressure_input: the measured (true/reference) ventricular pressure
% time_input: time associated with the measured (true/reference) ventricular pressure

%%% OUTPUTS
% E: a measure of error between measured and estimated (using "w") ventricular pressure

% Step 1: 
circulation_model = Circulation(round(w(1)), w(2), w(3));

% Step 2: 
[time, state] = circulation_model.simulate(2);

% Step 3: Making sure that the both measured and estimated ventricular pressures
% have a same length
ventricularPressure_est =  state(:,1);
if length(ventricularPressure_est) < length(ventricularPressure_input)
    ventricularPressure_est = interp1(time , ventricularPressure_est, time_input, 'makima');
else
    ventricularPressure_input = interp1(time_input , ventricularPressure_input, time, 'makima');
end

% Step 4: 
E = (1/2)*rms(ventricularPressure_est - ventricularPressure_input);

end