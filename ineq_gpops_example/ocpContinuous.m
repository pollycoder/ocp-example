%-------------------------------------------------------------------%
% Inequality Constraint: GPOPS-II                                   %
% Reference: Maurer's Lecture - Part 3                              %
% Continuous Function                                               %
%-------------------------------------------------------------------%
function phaseout = ocpContinuous(input)
t = input.phase.time;
x1 = input.phase.state(:, 1);
x2 = input.phase.state(:, 2);
u = input.phase.control;

dx(:, 1) = x2;
dx(:, 2) = -x1 + x2 .* (ones(size(x2)) - x1.^2) + u;

phaseout.dynamics = dx;
phaseout.integrand = u.^2 + x1.^2 + x2.^2;
phaseout.path = x2;
end