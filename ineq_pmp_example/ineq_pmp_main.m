%-------------------------------------------------------------------%
% Inequality Constraint: PMP                                        %
% Reference: Maurer's Lecture - Part 3                              %
%-------------------------------------------------------------------%
clc;clear
%-------------------------------------------------------------------%
%------------- Guess: switch point and initial costate -------------%
%-------------------------------------------------------------------%
lambda0 = ones(2, 1);
lambda1 = ones(2, 1);
lambda2 = ones(2, 1);
dt1 = 0.887; 
dt2 = 1.733;
x1t1 = 0.8;
x1t2 = 0.2;
X0 = [dt1; dt2; x1t1; x1t2; lambda0; lambda1; lambda2];


%-------------------------------------------------------------------%
%------------------------------ Solve ------------------------------%
%-------------------------------------------------------------------%
t0 = 0; tf = 4;
x0 = [1; 1];
tol = -1e-3;
A = [-eye(2), zeros(2, 8);
     ones(1, 2), zeros(1, 8)];
b = [tol*ones(2, 1); tf-t0+tol];
options = optimoptions("fmincon", ...
                       "ConstraintTolerance", 1e-8, ...
                       "FunctionTolerance", 1e-8, ...
                       "MaxIterations", 1e3, ...
                       "UseParallel",true, ...
                       "MaxFunctionEvaluations", 1e6, ...
                       "Algorithm", "sqp", ...
                       "OptimalityTolerance", 1e-8, ...
                       "StepTolerance", 1e-15, ...
                       "Display", "iter");
[X, J] = fmincon(@obj_func, X0, A, b, [],[],[],[], @nonlcon, options);

[~,res] = nonlcon(X);

%-------------------------------------------------------------------%
%------------------------------ Plot -------------------------------%
%-------------------------------------------------------------------%
a = -0.4;
dt1 = X(1); dt2 = X(2);
t1 = t0 + dt1; t2 = t1 + dt2;
x1t1 = X(3); x1t2 = X(4);
lambda0 = zeros(2, 1); lambda0 = X(5:6);
lambda1p = zeros(2, 1); lambda1p = X(7:8);
lambda2p = zeros(2, 1); lambda2p = X(9:10);

%-------------------------- Initial Guess --------------------------%
y0_guess = [x0; lambda0];
y1_guess = [x1t1; a; lambda1p];
y2_guess = [x1t2; a; lambda2p];

%---------------------------- Solving ------------------------------%
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-10, ...
                 'NormControl','on');
[t01, y01] = ode45(@odefun_off, [t0, t1], y0_guess, options);
[t12, y12] = ode45(@odefun_on, [t1, t2], y1_guess, options);
[t2f, y2f] = ode45(@odefun_off, [t2, tf], y2_guess, options);

x1 = [y01(:, 1); y12(:, 1); y2f(:, 1)];
x2 = [y01(:, 2); y12(:, 2); y2f(:, 2)];
u = -[y01(:, 4); y12(:, 4); y2f(:, 4)]./2;
t = [t01; t12; t2f];

figure
plot(t, x2, 'LineWidth', 1.5);

figure
plot(t, u, 'LineWidth', 1.5);