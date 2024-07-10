%-------------------------------------------------------------------%
% Inequality Constraint: PMP                                        %
% Reference: Maurer's Lecture - Part 3                              %
%-------------------------------------------------------------------%
clc;clear
%-------------------------------------------------------------------%
%------------- Guess: switch point and initial costate -------------%
%-------------------------------------------------------------------%
lambda0 = [28.6193227741150; 10.4422400568376];
lambda1 = [31.0886616237822; -2.03566438569542];
lambda2 = [24.3163148662157; -0.704213576776701];
dt1 = 0.411538306898497; 
dt2 = 3.77274066575983 - 0.411538306898497;
x1t1 = 1.05398863289202;
x1t2 = 0.0374836455215355;
X0 = [dt1; dt2; x1t1; x1t2; lambda0; lambda1; lambda2];


%-------------------------------------------------------------------%
%------------------------------ Solve ------------------------------%
%-------------------------------------------------------------------%
options = optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt');
X = fsolve(@nonlcon, X0, options);

%-------------------------------------------------------------------%
%------------------------------ Plot -------------------------------%
%-------------------------------------------------------------------%
a = -0.3;
t0 = 0; tf = 4;
x0 = [1; 1];
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
dJ = x1.^2 + x2.^2 + u.^2;
J = trapz(t, dJ);

figure
plot(t, x1, 'LineWidth', 1.5);hold on
plot(t, x2, 'LineWidth', 1.5);
legend('x1', 'x2')
title('State Trajectories')

figure
plot(t, u, 'LineWidth', 1.5);
title('Control')