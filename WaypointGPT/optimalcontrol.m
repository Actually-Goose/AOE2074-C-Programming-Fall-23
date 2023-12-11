%% Define Problem

%Find v, u
v = [x y z xdot ydot zdot phi theta psi wx wy wz];
    %State vector v contains: x, y, z, roll (phi), pitch (theta), yaw (psi)
    %Values and all their first derivates (velocities)
u = [taux tauy tauz thrust];
    %Control function u contains accelerations in all 6 degrees of freedom

% Subject to constraints
t0 = 0; %Initial time, seconds
v0 = [0 0 0 0 0 0 0 0 0 0 0 0]; %Initial state
vf = [1 1 1 0 0 0 0 0 pi/2 0 0 0]; %Final state
%tf is free
%{
    find u, v to minimize
        where u is variable-length matrix containing one 
            [taux tauy tauz Thrust] for each time step
            [
    J = h(tf) + integral(g(x), t0, tf);
        
    
    where 

%}


%For minimum

function rcost = g(u, t)

    persistent v;

    dt = 0.01; %time step
    c = 100; %Parameter of Optimization - importance of travel time

    [v, ~] = dynamics(v, u, t, dt);

    transform = [sin(v(8))*cos(v(7)) ...
                 sin(v(7))*cos(v(8)) ... 
                 cos(v(8))*cos(v(7))];
    rcost = dot(u(1:3),v(10:12)) + ...
           u(4) * dot(transform, v(4:6)) +  c;
end





