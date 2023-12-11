function out = writeToFile(path, name, dt)

    tvec = (path{1,1}:dt:path{1,3}); %first column is time
    n = length(tvec); %calculate how many lines

    xvec = path{2,1}(tvec); %evaluate x over tvec
    xdvec = path{2,2}(tvec); %same with x.

    yvec = path{3,1}(tvec); %same for y
    ydvec = path{3,2}(tvec); %same for y'

    zvec = path{4,1}(tvec); %same for z
    zdvec = path{4,2}(tvec); %same for z'

    %these are not important but needed for the file format
    roll = zeros(n,1);
    pitch = zeros(n,1);
    yaw = zeros(n,1);
    rolldot = zeros(n,1);
    pitchdot = zeros(n,1);
    yawdot = zeros(n,1);
    flags = zeros(n,1);

    %put eveyrthing into one matrix transposed for the right shape
    out = [tvec', xvec', yvec', zvec', xdvec', ydvec', zdvec', roll, pitch, yaw, rolldot, pitchdot, yawdot, flags];

    filename = [name '.cfg']; %make and open file
    fp = fopen(filename, 'w');

    for k = 1:n %print out line by line
        fprintf(fp, '%f ', out(k, 1:end-1));
        fprintf(fp, '%d', out(k, end));
        fprintf(fp, '\n');
    end

    fclose(fp); %save file
end