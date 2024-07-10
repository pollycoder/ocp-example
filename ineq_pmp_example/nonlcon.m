%-------------------------------------------------------------------%
%---------------------- Nonlinear Constraint -----------------------%
%-------------------------------------------------------------------%
function [ceq] = nonlcon(X)
a = -0.3;
t0 = 0; tf = 4;
x0 = [1; 1]; xf = [0; 0];
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
[~, y01] = ode45(@odefun_off, [t0, t1], y0_guess, options);
lambda1m = y01(end, 3:4)';
x2t1m = y01(end, 2);

[~, y12] = ode45(@odefun_on, [t1, t2], y1_guess, options);
lambda2m = y12(end, 3:4)';
x2t2m = y12(end, 2);

[~, y2f] = ode45(@odefun_off, [t2, tf], y2_guess, options);
xf_solve = y2f(end, 1:2)';

res = [xf_solve - xf; x2t1m - a; x2t2m - a; 
       lambda1m - lambda1p; lambda2m - lambda2p];
x1 = [y01(:, 1); y12(:, 1); y2f(:, 1)];
x2 = [y01(:, 2); y12(:, 2); y2f(:, 2)];
lambda1 = [y01(:, 3); y12(:, 3); y2f(:, 3)];
lambda2 = [y01(:, 4); y12(:, 4); y2f(:, 4)];
u = -lambda2./2;
H = u.^2 + x1.^2 + x2.^2 + lambda1.*x2 + lambda2.*(-x1 + x2.*(ones(size(x1)) - x1.^2) + u);
dH = diff(H);
ceq = [res; norm(dH)]; c = [];
end