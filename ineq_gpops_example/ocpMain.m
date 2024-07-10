%-------------------------------------------------------------------%
% Inequality Constraint: GPOPS-II                                   %
% Reference: Maurer's Lecture - Part 3                              %
% Main Script                                                       %
%-------------------------------------------------------------------%
clc;clear
tic

%-------------------------------------------------------------------%
%---------------------------- Constant -----------------------------%
%-------------------------------------------------------------------%

% Constant
t0 = 0; tf = 4;                                         % Time
x0 = [1, 1]; xf = [0, 0];                               % State
a = -0.4;

% Bounds - State
xmin = -2;
xmax = 2;

% Bounds - Control
umin = -10;
umax = 10;

% Bounds - Path
path_min = a;
path_max = 1;

%-------------------------------------------------------------------%
%------------------------ Bounds and Guess -------------------------%
%-------------------------------------------------------------------%

%----------------------------- Bounds ------------------------------%
% Time
bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tf;
bounds.phase.finaltime.upper = tf;

% State
bounds.phase.initialstate.lower = x0;
bounds.phase.initialstate.upper = x0;
bounds.phase.finalstate.lower = xf;
bounds.phase.finalstate.upper = xf;
bounds.phase.state.lower = xmin * ones(1, 2);
bounds.phase.state.upper = xmax * ones(1, 2);

% Control
bounds.phase.control.lower = umin;
bounds.phase.control.upper = umax;

% Path Constraint
bounds.phase.path.lower = path_min;
bounds.phase.path.upper = path_max;

% Integral
bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 50;


%------------------------------ Guess ------------------------------%
% Use spherical coordinate to guess position
N = 1000;

% Time
guess.phase.time = linspace(t0, tf, N)';

% State
guess.phase.state(:, 1) = ones(size(guess.phase.time));
guess.phase.state(:, 2) = ones(size(guess.phase.time));

% Control
guess.phase.control = linspace(umax, umin, N)';

% Integral
guess.phase.integral = 0;


%-------------------------------------------------------------------%
%-------------------------------- Mesh -----------------------------%
%-------------------------------------------------------------------%
mesh.method = 'hp1';
mesh.tolerance = 1e-8; 
mesh.maxiteration = 1000;
mesh.colpointmin = 4;
mesh.colpointmax = 50;
mesh.phase.colpoints = 10*ones(1,20);
mesh.phase.fraction = 0.05*ones(1,20);



%-------------------------------------------------------------------%
%--------------------------- Problem Setup -------------------------%
%-------------------------------------------------------------------%
setup.name = 'OCP Example - Inequality Path Constraint';
setup.functions.continuous = @ocpContinuous;
setup.functions.endpoint = @ocpEndpoint;
setup.bounds = bounds;
setup.guess = guess;

setup.nlp.solver = 'snopt';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.scales.method = 'automatic-bounds';
setup.mesh = mesh;
setup.method = 'RPMintegration';


%-------------------------------------------------------------------%
%------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------%
output = gpops2(setup);
solution = output.result.solution;
J = solution.phase.integral;

x1 = solution.phase.state(:, 1);
x2 = solution.phase.state(:, 2);
t = solution.phase.time;


plot(t, x2, 'LineWidth', 1.5);