function path = threedimensionpath(a, b, t)
    
    % Do the path in all three dimensions
    [x,xdot,xddot, ~, ~] = onedimension(a(1),b(1),t(1),t(2));
    [y,ydot,yddot, ~, ~] = onedimension(a(2),b(2),t(1),t(2));
    [z,zdot,zddot, ~, ~] = onedimension(a(3),b(3),t(1),t(2));

    path = {t(1), 0, t(2); %  t0    ~    t1
            x, xdot, xddot; % x(t) x'(t) x''(t)
            y, ydot, yddot; %  "     "     "  
            z, zdot, zddot};%  "     "     "  

end

