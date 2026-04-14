function stepSignal = funmakeStep(s1, stepVal, tStep, Tfinal, dt)
% generate a step signal for Simulink From Workspace (Array 2-D)


% Time vector
t = (0:dt:Tfinal)';

% initialize signal
u = s1 * ones(size(t));

% Apply step
u(t >= tStep) = stepVal;

% Combine
stepSignal = [t u];

end
