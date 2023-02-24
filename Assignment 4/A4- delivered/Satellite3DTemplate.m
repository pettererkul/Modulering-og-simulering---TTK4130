clear all
%close all
clc

% Eart params
earth_radius = 6356e+3;
orbit_height = 36000e+3;
azi = pi/4; %azimuth
dec = pi/4; %declination

% Setting inintial conditions for the states
position = (earth_radius+orbit_height)*[sin(dec)*cos(azi);sin(dec)*sin(azi);cos(dec)];
omega = [deg2rad(60);deg2rad(80);deg2rad(100)];
velocity = [0;0;0];
% Rotation matrix describing the satellite orientation
r = random('norm',0,1,[3,1]); % Random axis of rotation / angle
orientation = expm([   0,      -r(3),  +r(2);
             +r(3),     0,   -r(1);
             -r(2), +r(1),    0]);      

% Define your initial state, e.g. as:
 state = [position;
          reshape(orientation,9,1);
          velocity;
          omega];

% "parameters" allows you to pass some parameters to the "SatelliteDynamics" function
mass = 1;
length = 50e-3;
parameters = 1/6*mass*length^2;

time_final = 6; %Final time

% Simulate satellite dynamics
[time,statetraj] = ode45(@(t,x)SatelliteDynamics(t, x, parameters),[0,time_final],state);

% Here below is a template for a real-time animation
ScaleFrame = 5;   % Scaling factor for adjusting the frame size (cosmetic)
FS         = 15;  % Fontsize for text
SW         = 0.035; % Arrows size
tic; % resets Matlab clock
time_display = 0; % time displayed
while time_display < time(end)
    time_animate = toc; % get the current clock time
    % Interpolate the simulation at the current clock time
    state_animate = interp1(time,statetraj,time_animate)';

    pos_cm = state_animate(1:3)*1e-7;        %scaled position of the satellite
    R = reshape(state_animate(4:12),3,3); % orientation
    
    omega = state_animate(16:18);          % omega

    

    figure(1);clf;hold on
    % Use the example from "Satellite3DExample.m" to display your satellite
    MakeFrame(zeros(3,1),eye(3),ScaleFrame,FS,SW,'a', 'color', 'k')
    MakeFrame(pos_cm, R, ScaleFrame,FS,SW,'b', 'color', 'r')
    MakeArrow(pos_cm,R*omega,FS,SW,'$$\omega$$', 'color', [0,0.5,0])
    DrawRectangle(pos_cm,R,'color',[0.5,0.5,0.5]);
    FormatPicture([0;0;2],0.5*[73.8380   21.0967   30.1493])

    if time_display == 0
        display('Hit a key to start animation')
        pause
        tic
    end
    time_display = toc; % get the current clock time
end