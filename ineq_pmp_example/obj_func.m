%-------------------------------------------------------------------%
%----------------------- Objective Function ------------------------%
%-------------------------------------------------------------------%
function J = obj_func(X)
a = -0.4;
t0 = 0; 
dt1 = X(1); dt2 = X(2);
t1 = t0 + dt1; t2 = t1 + dt2;
x1t1 = X(3); x1t2 = X(4);
lambda0 = zeros(2, 1); lambda0 = X(5:6);
lambda1p = zeros(2, 1); lambda1p = X(7:8);
lambda2p = zeros(2, 1); lambda2p = X(9:10);

t0 = 0; tf = 4;
x0 = [1; 1];

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
end