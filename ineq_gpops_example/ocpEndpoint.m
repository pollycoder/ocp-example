%-------------------------------------------------------------------%
% Inequality Constraint: GPOPS-II                                   %
% Reference: Maurer's Lecture - Part 3                              %
% Endpoints Function                                                %
%-------------------------------------------------------------------%
function output = ocpEndpoint(input)
q = input.phase.integral;
output.objective = q;
end