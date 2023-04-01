%% SENSITIVITY ANALYSIS 1

% Circulation model has an increased HR, check if results are impacted
increased_HR_model = Circulation(100, 2, 0.06);

T = 5;
[time, state] = increased_HR_model.simulate(T);

ventricular_pressure = state(:,1);
atrial_pressure = state(:,2);
arterial_pressure = state(:,3);
aortic_pressure = zeros(size(state(:,4)));

% Calculating aortic pressure just outside the aortic valve
for i = 1:length(time)
    x_dot = increased_HR_model.get_derivative(time(i), transpose(state(i,:)));
    aortic_pressure(i) = state(i,3) + increased_HR_model.R4*state(i,4) + increased_HR_model.L.*x_dot(4);
end

%%% Plotting
figure()
LineWidth = 1.5;
FontSize = 12;
% Your plotting code should be here
plot(time, ventricular_pressure, 'r', 'LineWidth', LineWidth), hold on;
plot(time, atrial_pressure, 'g', 'LineWidth', LineWidth);
plot(time, arterial_pressure, 'b', 'LineWidth', LineWidth);
plot(time, aortic_pressure, 'k', 'LineStyle', '--', 'LineWidth', LineWidth), hold off;

legend('ventricular', 'atrial', 'arterial', 'aortic')
title('Increased HR')
xlabel('Time (seconds)')
ylabel('Pressure (mmHg)')
set(gca, 'FontSize', FontSize)

%% SENSITIVITY ANALYSIS 2

% Changed initial conditions in Circulation.simulate
% original = [0, obj.non_slack_blood_volume/obj.C2, 0, 0]
% new = [0, 200/obj.C2, 50/obj.C3, 0]

initial_conditions_model = Circulation(75, 2, 0.06);

T = 5;
[time, state] = initial_conditions_model.simulate(T);

ventricular_pressure = state(:,1);
atrial_pressure = state(:,2);
arterial_pressure = state(:,3);
aortic_pressure = zeros(size(state(:,4)));

% Calculating aortic pressure just outside the aortic valve
for i = 1:length(time)
    x_dot = initial_conditions_model.get_derivative(time(i), transpose(state(i,:)));
    aortic_pressure(i) = state(i,3) + initial_conditions_model.R4*state(i,4) + initial_conditions_model.L.*x_dot(4);
end

%%% Plotting
figure()
LineWidth = 1.5;
FontSize = 12;
% Your plotting code should be here
plot(time, ventricular_pressure, 'r', 'LineWidth', LineWidth), hold on;
plot(time, atrial_pressure, 'g', 'LineWidth', LineWidth);
plot(time, arterial_pressure, 'b', 'LineWidth', LineWidth);
plot(time, aortic_pressure, 'k', 'LineStyle', '--', 'LineWidth', LineWidth), hold off;

legend('ventricular', 'atrial', 'arterial', 'aortic')
title('Different Initial Conditions')
xlabel('Time (seconds)')
ylabel('Pressure (mmHg)')
set(gca, 'FontSize', FontSize)

%% LITERATURE COMPARISON

% Graphing aortic flow

initial_conditions_model = Circulation(75, 2, 0.06);

T = 5;
[time, state] = initial_conditions_model.simulate(T);

aortia_inflow = state(:,4);

%%% Plotting
figure()
LineWidth = 1.5;
FontSize = 12;
% Your plotting code should be here
plot(time, aortia_inflow, 'r', 'LineWidth', LineWidth);

title('Aortic Flow over Time')
xlabel('Time (seconds)')
ylabel('Aortic Flow (mL/sec)')
set(gca, 'FontSize', FontSize)