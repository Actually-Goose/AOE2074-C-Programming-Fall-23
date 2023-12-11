function [] = fullsim() 
    % Choose a set of waypoints
    % Ensure that these are the same as the real data you're looking at
    %a = [3.01 1 1 -2]; % [time xpos ypos zpos]
    %b = [6 0 0 -1];
    dt = 0.01; %discrete time step

    % path = threedimensionpath(a(2:4),b(2:4),[a(1) b(1)]);
    % Calculate the functions that represent position, velocity,
    % acceleration in each axis
    % Write the data to a file

    path1 = threedimensionpath([0 0 0], [0 0 -1], [0 3]);
    path2 = threedimensionpath([0 0 -1], [1 1 -2], [0 3]);
    path3 = threedimensionpath([1 1 -2], [-1 -1 -2], [3.01 6]);
    path4 = threedimensionpath([-1 -1 -2], [0 0 -1], [6.01 9]);
    path5 = threedimensionpath([0 0 -1], [0 0 0], [0 3]);

    %% Pull in real data

    n = height(testdata);
    rt = testdata(2:n,2).u_Time ./ 1000000;
    rt = rt - rt(1);
    rx = testdata(2:n,3).x;
    ry = testdata(2:n,4).y;
    rz = testdata(2:n,5).z;
    rqxyz = testdata(2:n,6:8);
    rqw = testdata(2:n, 9);
    rq = table2array([rqw rqxyz]);

    [ryaw,rpitch,rroll] = quat2angle(rq);

    rroll = rroll';
    rpitch = rpitch';
    ryaw = ryaw';

    states = testdata(2:n,11).state;

    states = states';

    rt = rt';
    rx = rx';
    ry = ry';
    rz = rz';

    %Find where Takeoff, Land, Guided was pressed

    startTime = rt(find(states == 1,1)); %Locate the time when T was pressed, or the period of interest begins
    rt = rt - startTime; %subtract that from all the times to make it start at zero

    takeoffStartTime = 0; %this what we just found and set to zero
    takeoffEndTime = takeoffStartTime + 3; %takeoff takes 3 seconds
    guidedStartTime = rt(find(states == 2,1)); %find when the state changes to 2
    guidedEndTime = guidedStartTime + 9; %guided takes 9 seconds
    landingStartTime = rt(find(states == 3, 1)); %find when the state changes to 3
    landingEndTime = landingStartTime + 3; %landing takes 3 seconds

    %% New Plot Data

    f = figure;
    hold on;
    xline(takeoffStartTime,'Color','black');
    xline(takeoffEndTime,'Color','black');
    xline(guidedStartTime,'Color','black');
    xline(guidedEndTime,'Color','black');
    xline(landingStartTime,'Color','black');
    xline(landingEndTime,'Color','black');
    xlim([takeoffStartTime-0.5 landingEndTime+0.5]);
    plot(rt, -rpitch, 'Color', 'r', 'DisplayName','x');
    plot([0:dt:3], 0.05*path1{2,3}([0:dt:3]),"Color",'blue');
    plot(guidedStartTime + [0:dt:3], 0.05*path2{2,3}([0:dt:3]),"Color",'blue');
    plot(guidedStartTime + [3.01:dt:6], 0.05*path3{2,3}([3.01:dt:6]),"Color",'blue');
    plot(guidedStartTime + [6.01:dt:9], 0.05*path4{2,3}([6.01:dt:9]),"Color",'blue');
    plot(landingStartTime + [0:dt:3], 0.05*path5{2,1}([0:dt:3]),"Color",'blue');

    yRange = ylim;
    textPositionY = yRange(1) + (yRange(2) - yRange(1)) * 0.9; 

    midTakeoff = (takeoffStartTime + takeoffEndTime) / 2;
    text(midTakeoff, textPositionY, 'Takeoff', 'HorizontalAlignment', 'center', 'Color', 'black');
    
    midGuided = (guidedStartTime + guidedEndTime) / 2;
    text(midGuided, textPositionY, 'Guided', 'HorizontalAlignment', 'center', 'Color', 'black');
    
    midLanding = (landingStartTime + landingEndTime) / 2;
    text(midLanding, textPositionY, 'Landing', 'HorizontalAlignment', 'center', 'Color', 'black');

    xlabel("Time (s)");
    ylabel("Angle (rad), Acceleration (m/s^2)");

end

function graphall(path, t0, tf, dt) 
    figure;
    subplot(3, 1, 1); % 3 rows, 1 column, 1st subplot
    graph(path{2,1}, path{2,2}, path{2,3}, [t0:dt:tf], 'x');
    subplot(3, 1, 2); % 3 rows, 1 column, 1st subplot
    graph(path{3,1}, path{3,2}, path{3,3}, [t0:dt:tf], 'y');
    subplot(3, 1, 3); % 3 rows, 1 column, 1st subplot
    graph(path{4,1}, path{4,2}, path{4,3}, [t0:dt:tf], 'z');
end

function graph(x,xdot,xddot,t,dim)
    plot(t, x(t),"Color",'r','DisplayName', ['$' dim '$']);
    hold on;
    plot(t, xdot(t),"Color",'b', 'DisplayName', ['$\dot{' dim '}$']);
    plot(t, xddot(t),"Color",'g', 'DisplayName', ['$\ddot{' dim '}$']);
    xlabel("Time");
    ylabel("Value");
    lgd = legend('Interpreter', 'latex');
    lgd.FontSize = 16;
end

