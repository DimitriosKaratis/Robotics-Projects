%% Robotics
% Assignment A - May 2025
% KARATIS DIMITRIOS 10775

clc; clear; close all;

% Parameters
T = 5;                  % total duration of motion [s]
dt = 0.01;              % time step for simulation [s]
t_phase1 = 0:dt:T/4;    % time vector for phase 1
t_phase2 = 0:dt:T/2;    % time vector for phase 2
t_phase3 = 0:dt:T/4;    % time vector for phase 3

N1 = length(t_phase1);  % number of steps in phase 1
N2 = length(t_phase2);  % number of steps in phase 2
N3 = length(t_phase3);  % number of steps in phase 3

% Geometry parameters
y = 2;                  
l = 1;
l0 = 0.1;
h = 0.7;

%% Define transformation matrices for points a and c 
% g_od_a: pose of door frame D relative to origin O at initial config A
g_od_a = [1 0 0 0;
          0 1 0 2;
          0 0 1 0;
          0 0 0 1];

% g_dh_a: pose of handle frame H relative to door frame D at config A
g_dh_a = [0 1 0 l-l0;
          -1 0 0 0;
          0 0 1 h;
          0 0 0 1]; 

% g_oh_a: pose of handle H relative to origin O at config A
g_oh_a = g_od_a * g_dh_a;

% Door frame pose at config B same as A initially
g_od_b = g_od_a;

% Rotate handle frame around X-axis by -45° from config A to get config B
R_dh_b = t2r(g_dh_a) * t2r(trotx(-pi/4));
p_dh_b = [l-l0; 0; h];
g_dh_b = [R_dh_b, p_dh_b; 0 0 0 1]; 

% Calculate handle pose at config B relative to origin
g_oh_b = g_od_b * g_dh_b;

% Door frame pose at config C is rotated by -30° about Z-axis from config A
R_od_c = t2r(g_od_a) * t2r(trotz(-pi/6));
p_od_c = [0; 2; 0];
g_od_c = [R_od_c, p_od_c; 0 0 0 1];

% Handle pose at config C (handle relative to door remains same as config A)
g_dh_c = g_dh_a;
g_oh_c = g_od_c * g_dh_c;


%% Plot initial coordinate frames for visualization 
figure('Name', 'Initial Coordinate Frames');
axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Initial Coordinate Frames');
hold on;

% Plot world frame (origin)
trplot(eye(4), 'frame', '0', 'color', 'k', 'length', 0.2);

% Plot door frame D and handle frame H at initial config A
trplot(g_od_a, 'frame', 'D', 'color', 'b', 'length', 0.2);
trplot(g_oh_a, 'frame', 'H', 'color', 'r', 'length', 0.2);

% Plot door and handle frames at final config C for comparison
trplot(g_od_c, 'frame', 'D_{end}', 'color', 'g', 'length', 0.2);
trplot(g_oh_c, 'frame', 'H_{end}', 'color', 'g', 'length', 0.2);


%% Interpolate transformations through all phases
% Initialize arrays to store trajectory matrices and positions/quaternions
g_oh_traj = zeros(4,4,N1+N2+N3);
g_od_traj = zeros(4,4,N1+N2+N3);
p_oh_traj = zeros(N1+N2+N3,3);
q_oh_traj = zeros(N1+N2+N3,4);

% Phase 1 interpolation (A -> B) using cubic time scaling
for i = 1:N1
    s = time_scaling_cubic(t_phase1(i)/(T/4));      % scaled time parameter s(t)
    g_dh_i = trinterp(g_dh_a, g_dh_b, s);           % interpolate handle wrt door
    g_od_i = g_od_a;                                % door frame fixed in phase 1
    g_oh_i = g_od_i * g_dh_i;                       % compute handle pose in world
    g_od_traj(:,:,i) = g_od_i;
    g_oh_traj(:,:,i) = g_oh_i;
    p_oh_traj(i,:) = transl(g_oh_i)';               % extract position vector
    q_oh_traj(i,:) = UnitQuaternion(g_oh_i).double; % extract orientation quaternion
end

% Phase 2 interpolation (B -> B')
for i = 1:N2
    s = time_scaling_cubic(t_phase2(i)/(T/2));      % scaled time parameter
    g_od_i = trinterp(g_od_b, g_od_c, s);           % interpolate door frame pose
    g_dh_i = g_dh_b;                                % handle fixed relative to door
    g_oh_i = g_od_i * g_dh_i;                       % compute handle pose
    g_od_traj(:,:,N1+i) = g_od_i;
    g_oh_traj(:,:,N1+i) = g_oh_i;
    p_oh_traj(N1+i,:) = transl(g_oh_i)';
    q_oh_traj(N1+i,:) = UnitQuaternion(g_oh_i).double;
end

% Phase 3 interpolation (B' -> C)
for i = 1:N3
    s = time_scaling_cubic(t_phase3(i)/(T/4));
    g_dh_i = trinterp(g_dh_b, g_dh_a, s);           % interpolate handle relative to door
    g_od_i = g_od_c;                                % door frame fixed in phase 3
    g_oh_i = g_od_i * g_dh_i;
    g_od_traj(:,:,N1+N2+i) = g_od_i;
    g_oh_traj(:,:,N1+N2+i) = g_oh_i;
    p_oh_traj(N1+N2+i,:) = transl(g_oh_i)';
    q_oh_traj(N1+N2+i,:) = UnitQuaternion(g_oh_i).double;
end

%% Animate the door and handle motion 
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
rotate3d on; 
title('Door and Handle Motion');
hold on;

% Plot fixed world frame
trplot(eye(4), 'frame', '0', 'color', 'k', 'length', 0.2);

% Plot initial door and handle frames
h_door = trplot(g_od_a, 'frame', 'D', 'color', 'b', 'length', 0.2);
h_handle = trplot(g_oh_a, 'frame', 'H', 'color', 'r', 'length', 0.2);

% Plot handle trajectory path as dashed line
plot3(p_oh_traj(:,1), p_oh_traj(:,2), p_oh_traj(:,3), 'k--');

% Total number of steps across all phases (Phase 1 + Phase 2 + Phase 3)
total_steps = N1 + N2 + N3;

% Total duration of the motion (in seconds)
total_time = T;

% Time allocated per frame 
frame_time = total_time / total_steps;

% Display every 4th frame to reduce computational load and speed up animation
step_interval = 4;
num_frames = floor(total_steps / step_interval);

% Begin animation loop
for j = 1:num_frames
    % Compute the actual index in the trajectory arrays
    i = (j - 1) * step_interval + 1;

    % Get current transformation matrices for door and handle
    g_od_i = g_od_traj(:,:,i);
    g_oh_i = g_oh_traj(:,:,i);

    % Delete previous door frame from plot
    delete(h_door);
    
    % Plot current door frame
    h_door = trplot(g_od_i, 'frame', 'D', 'color', 'b', 'length', 0.2);

    % Delete previous handle frame from plot
    delete(h_handle);
    
    % Plot current handle frame
    h_handle = trplot(g_oh_i, 'frame', 'H', 'color', 'r', 'length', 0.2);

    %% Optional: plot a visual door object
    % Uncomment this block if you want to render the door as a 3D patch
    % if exist('door','var'), delete(door); end
    % door = plotDoor(g_od_i, 1, 2.0);
    %%

    % Refresh the figure with updated frames
    drawnow;
end


%% Plot handle position and orientation trajectory
% Construct full timeline by concatenating phase time vectors with offsets
time_full = [t_phase1, t_phase2 + T/2, t_phase3 + (3*T/4)];

% Plot 3D position trajectory of handle H
figure('Name','Handle Position Trajectory');
plot3(p_oh_traj(:,1), p_oh_traj(:,2), p_oh_traj(:,3), 'LineWidth', 2);
hold on;
plot3(p_oh_traj(1,1), p_oh_traj(1,2), p_oh_traj(1,3), 'go', 'MarkerSize', 10, 'DisplayName', 'Start'); % start marker
plot3(p_oh_traj(end,1), p_oh_traj(end,2), p_oh_traj(end,3), 'ro', 'MarkerSize', 10, 'DisplayName', 'End'); % end marker
grid on; axis equal; xlabel('X'); ylabel('Y'); zlabel('Z');
title('Handle {H} Position Trajectory');
legend show;

% Plot orientation quaternion components vs time
figure('Name','Handle Orientation Quaternion Trajectory');
plot(time_full, q_oh_traj(:,1), 'LineWidth', 2, 'DisplayName','q_0');
hold on;
plot(time_full, q_oh_traj(:,2), 'LineWidth', 2, 'DisplayName','q_1');
plot(time_full, q_oh_traj(:,3), 'LineWidth', 2, 'DisplayName','q_2');
plot(time_full, q_oh_traj(:,4), 'LineWidth', 2, 'DisplayName','q_3');
grid on;
xlabel('Time [s]');
ylabel('Quaternion components');
title('Handle {H} Orientation Trajectory (Unit Quaternion)');
legend show;

% Save the trajectories for part B
save('partA_oh_traj.mat', 'g_oh_traj');
save('partA_od_traj.mat', 'g_od_traj');




%%%%%%%%%%%%%%%%%%%%%%%%%%%% HELPER FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% === Time scaling cubic function ===
% Smooth time scaling function for s(t) in [0,1]
function s = time_scaling_cubic(tau)
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
end

% === Door plotting helper function ===
% Plot a rectangular door as a flat black patch given its transform
function h = plotDoor(g_od, width, height)
    % Define door corners in door frame coordinates
    corners = [0 width width 0;   % X coords
               0 0 0 0;           % Y coords (zero thickness)
               0 0 height height; % Z coords (height)
               1 1 1 1];          % Homogeneous coords

    % Transform corners to world frame
    transformed = g_od * corners;
    
    % Extract X, Y, Z coordinates for patch function
    X = transformed(1,:);
    Y = transformed(2,:);
    Z = transformed(3,:);

    % Draw filled polygon representing door
    h = patch(X, Y, Z, [0 0 0], 'FaceAlpha', 1);  % black color
end







