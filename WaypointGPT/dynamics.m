function [v, vdot] = dynamics(v0, u, t, dt) 
    %%Physical Parameters
    mass = 1.062; %kg
    g = 9.81; %m/s^2
    Ix = 1.07E-2; %kg m^2
    Iy = 1.11E-2; %kg m^2
    Iz = 2.29E-2; %kg m^2

    % v0 = [x  y  z  xd  yd  zd  phi  theta  psi  wx    wy      wz]
    % vd = [xd yd zd xdd ydd zdd phid thetad psid phidd thetadd psidd]
    % u =  [taux tauy tauz thrust]

    c = cos(v0(7:9)); %cos/sin/tan phi theta psi
    s = sin(v0(7:9));
    t = tan(v0(7:9));

    %%Run Dynamics

    %Update [xdd ydd zdd]
    vdot(4:6) = [0 0 g] - [c(3)*s(2)*c(1)+s(3)*s(1) ...
                           s(3)*s(2)*c(1)-c(3)*s(1) ...
                           c(2)*c(1)]*(u(4)/mass);
    %Calculates xdd ydd zdd based on thrust and orientation

    %Update [phidd thetadd psidd]
    vdot(10:12) = u(1:3).*[Ix Iy Iz] + [(Iy-Iz)/Ix (Iz-Ix)/Iy (Ix-Iy)/Iz].*[v0(11)*v0(12) v0(10)*v0(12) v0(10)*v0(11)];
    % d/dt w = tau / I + rotation
    vdot(7:9) = [1  t(2)*s(1) t(2)*c(1);
                 0  c(1)      -s(1);
                 0  s(1)/c(2) c(1)/c(2)] * v0(10:12)';
    
    %Update [xd yd zd]

    vdot(1:3) = v0(4:6);
    
    %%Euler's method time step
    v = v0 + vdot.*dt;

end
