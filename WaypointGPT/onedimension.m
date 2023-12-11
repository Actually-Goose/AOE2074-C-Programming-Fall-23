%% Create cubic spline in one dimension
function [x,xdot,xddot, t0, t1] = onedimension(a,b,t0,t1)
    %find a cubic polynomial such that 
    % f(t0) = a, f(t1) = b, f'(t0) = 0, f'(t1) = 0
    % x(t) = c1 + c2t + c3 t^2 + c4 t^3
    bvec = [a; b; 0; 0; 0; 0];
    A = [1 t0 t0^2 t0^3   t0^4    t0^5; %f(t0) = a
         1 t1 t1^2 t1^3   t1^4    t1^5; %f(t1) = b
         0 1  2*t0 3*t0^2 4*t0^3  5*t0^4; %f'(t0) = 0
         0 1  2*t1 3*t1^2 4*t1^3  5*t1^4; %f'(t1) = 0
         0 0  2    6*t0   12*t0^2 20*t0^3; %f''(t0) = 0
         0 0  2    6*t1   12*t1^2 20*t1^3;]; %f''(t1) = 0
    cvec = A\bvec; % solve system

    x = @(tau) cvec(1) + cvec(2)*tau + cvec(3)*tau.^2 + cvec(4)*tau.^3 + cvec(5)*tau.^4 + cvec(6)*tau.^5; %construct first and second derivatives
    xdot = @(tau) cvec(2) + 2*cvec(3)*tau + 3*cvec(4)*tau.^2 + 4*cvec(5)*tau.^3 + 5*cvec(6)*tau.^4;
    xddot = @(tau) 2*cvec(3) + 6*cvec(4)*tau + 12*cvec(5)*tau.^2 + 20*cvec(6)*tau.^3;

end




