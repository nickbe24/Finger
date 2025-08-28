
%% Square Demos
clear
points_per_edge = 2; 
points_between = points_per_edge - 1; 
path_pts = [];
%corners = [0 60 60; 0 70 60; 0 70 50; 0 60 50]; % YZ plane (no abduction)
%corners = [-10 60 60; -10 60 40; 10 60 40; 10 60 60]; % ZX plane (abductino)
%corners =  [-10 60 20; -10 80 20; 10 80 20; 10 60 20]; %XY plane
%corners = [0 50 -20; 0 50 -30; 0 60 -30; 0 60 -20]; % YZ lower workspace
corners = [0 60 60; 0 60.5 60; 0 60.5 59.5; 0 60 59.5;0 60 60];

for i = 1:4
    p1 = corners(i, :);
    p2 = corners(mod(i,4)+1, :);

    for j = 0:(points_between - 1)  % exclude last point
        t = j / points_between;
        pt = (1 - t) * p1 + t * p2;
        path_pts(end+1, :) = pt;
    end
end

% Append the first corner to close the loop
path_pts(end+1, :) = corners(1, :);

%for i = 1:size(path_pts, 1)
%    fprintf('%.2f %.2f %.2f\n', path_pts(i, :));
%end


%% Circle demo YZ No abduction
clear
r = 10;
center = [0, 70, 21.5];
N = 20;  % number of points

% Angles evenly spaced around circle
theta = linspace(0, 2*pi, N + 1);

% Compute points
x = zeros(1, N+1);
y = r * cos(theta) + center(2);
z = r * sin(theta) + center(3);

path_pts = [x', y', z'];

%% Circle demo XZ plane with Y locked at 65
clear
r = 10;
center = [0, 60, 40];
N = 20;  % number of points

% Angles evenly spaced around circle
theta = linspace(0, 2*pi, N + 1);

% Compute points
x = r * cos(theta) + center(1);
y = ones(1, N+1) * 65;
z = r * sin(theta) + center(3);

path_pts = [x', y', z'];

%% step path 5mm
clear
path_pts = [0 50 20; 0 55 20; 0 55 25; 0 60 25; 0 60 30; 0 65 30; 0 65 35; 0 70 35; 0 70 40; 0 50 20];

%% step path 3mm
clear
path_pts = [0 50 20; 0 53 20; 0 53 23; 0 56 23; 0 56 26; 0 59 26; 0 59 29; 0 62 29; 0 62 32; 0 50 20];

%% step path 3mm
clear
path_pts = [0 60 31.5; 0 63 31.5; 0 63 34.5; 0 66 34.5; 0 66 37.5; 0 69 37.5; 0 69 40.5];



%% SEND TO ARDUINO 
port = "COM6";      
baud = 115200;
sp = serialport(port, baud);
pause(1);          
for i = 1:size(path_pts, 1)
    line = sprintf('%.2f %.2f %.2f\n', path_pts(i, 1), path_pts(i, 2), path_pts(i, 3));
    fprintf("Sending: %s", line);  
    write(sp, line, "string");       
end

writeline(sp, "END");

