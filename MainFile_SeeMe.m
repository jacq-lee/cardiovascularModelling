%% Section 1: Modelling
clear
clc

circulation_model = Circulation(75, 2, 0.06);

T = 5;
[time, state] = circulation_model.simulate(T);

%%% TASK 4
% Do the necessary calculation (if required, this may be based on Q (4th state)) and then generate the desired plots.



%%% Plotting
figure()
LineWidth = 1.5;
FontSize = 12;
% Your plotting code should be here
%plot(....
%plot(....
%plot(....
%plot(....

legend('ventricular', 'atrial', 'arterial', 'aortic') % note the order here
xlabel('Time (seconds)')
ylabel('Pressure (mmHg)')
set(gca, 'FontSize', FontSize)

%% Optimization (you dont need to change any code for this section)
clear
clc

% Loading the true Ventricular Pressure which we 
% need it to calculate the objective function for optimization
load VentricularPressure

% create random initial solution
w_int =  [randi([60 80],1,1); rand(2,1)]; % initial solution
options = optimoptions('simulannealbnd','PlotFcns',...
    {@saplotbestf,@saplotf}, ... 
    'FunctionTolerance', 5e-7, ...
    'MaxIterations', 50, ... % here you can change the maximum iteration
    'ReannealInterval', 25);
lb = [60 0.3 0.001]; % lower bound for solutions
ub = [80 4 0.1]; % upper bound for solutions
tic
[w, fval] = simulannealbnd(@(w) ObjectiveFunction(w, ventricularPressure, time), ...
    w_int, lb, ub, options);
toc

disp(['Objective Function Value = ' num2str(fval)])
disp(['Estimated HR = ' num2str(round(w(1)))])
disp(['Estimated E_{max} = ' num2str(w(2))])
disp(['Estimated E_{min} = ' num2str(w(3))])




