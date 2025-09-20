%% Robotics
% Assignment B - May 2025
% KARATIS DIMITRIOS 10775

clc; clear; close all;

%% Initialization
ur10 = ur10robot();     % Create the UR10 robot object
T = 5;                  % Total time [seconds]
dt = 0.01;              % Time step
time = 0:dt:T;          % Time vector
N = length(time);       % Number of time steps

% Initial joint configuration (in radians) 
q0 = [-1.7752 -1.1823 0.9674 0.2149 1.3664 1.5708];  
q_traj = zeros(N, 6);      % To store joint positions
qd_traj = zeros(N, 6);     % To store joint velocities

% Load desired trajectories from Part A
load('partA_oh_traj.mat','g_oh_traj'); % Handle pose trajectory
load('partA_od_traj.mat','g_od_traj'); % Door pose trajectory

% Transformation from handle frame to end-effector frame 
R_he = [0 0 -1;
         0 1 0;
         1 0 0];

p_he = [0.1; 0.1; 0];  % Translation vector

g_he = [R_he, p_he;
        0 0 0 1];

g_eh = inv(g_he);  % Inverse transform (if needed)

% Initialize variables
q = q0';
q_traj(1,:) = q0;
g_oh = g_oh_traj(:,:,1);       % Initial handle pose
g_oe = g_oh * g_he;            % Initial end-effector pose

%% Control loop for joint trajectory generation 
for k = 2:N
    % Desired end-effector pose relative to base
    g_oh = g_oh_traj(:,:,k);
    g_oe_des = g_oh * g_he;

    % Current end-effector pose from forward kinematics
    g_oe_curr = ur10.fkine(q);

    % Pose error between current and desired (6x1 vector)
    deltaT = tr2delta(g_oe_curr.T, g_oe_des);

    % End-effector Jacobian (6x6)
    J = ur10.jacobe(q);

    % Calculate joint velocities via pseudo-inverse Jacobian
    q_dot = pinv(J) * (deltaT / dt);

    % Update joint positions using Euler integration
    q = q + q_dot * dt;

    % Store joint positions and velocities
    q_traj(k,:) = q';
    qd_traj(k,:) = q_dot';
end

%% Set up figure for animation 
figure('Name','UR10 Animation');
axis([-1 2 -0.5 3 0 1.5]); 
view(3); grid on;
xlabel('X'); ylabel('Y'); zlabel('Z'); 
hold on;

% Plot coordinate system reference
trplot(eye(4), 'frame', '0', 'length', 0.2, 'color', 'k');
plot3([-1 2], [0 0], [0 0], 'k--'); 
plot3([0 0], [-0.5 3], [0 0], 'k--');  


% Initial robot pose plot to setup figure
ur10.plot(q_traj(1,:), ...
    'workspace', [-1 2 -0.5 3 0 1.5], ...
    'view', [45 30]);

% Plot initial frames for door (d_0), end-effector (e_0), handle (H_0) 
g_e_start = ur10.fkine(q_traj(1,:));
g_h_start = g_oh_traj(:,:,1);
g_d_start = g_od_traj(:,:,1);

trplot(g_d_start, 'frame', 'd_0', 'color', 'b', 'length', 0.2);
trplot(g_e_start.T, 'frame', 'e_0', 'color', 'r', 'length', 0.2);  
trplot(g_h_start, 'frame', 'H_0', 'color', 'r', 'length', 0.2);  

% Animation Parameters 
step_interval = 4;                     % Show every 4th frame
num_frames = floor(N / step_interval); % Number of plotted frames
dt_anim = T / num_frames;              % Time per plotted frame

%% Animate robot and frames over trajectory 
for j = 1:num_frames
    i = (j - 1) * step_interval + 1;   % Index in the full trajectory corresponding to current animation frame

    ur10.animate(q_traj(i,:));        % Animate the UR10 robot at joint configuration q(i)
    
    % Compute the current poses of end-effector, handle, and door
    g_e = ur10.fkine(q_traj(i,:));    % End-effector pose from forward kinematics
    g_h = g_oh_traj(:,:,i);           % Handle pose at time step i
    g_d = g_od_traj(:,:,i);           % Door pose at time step i

    % Remove previous frame's plots (if they exist) to avoid overlap
    if exist('h_e','var'), delete(h_e); end
    if exist('h_h','var'), delete(h_h); end
    if exist('h_d','var'), delete(h_d); end
    if exist('h_handle','var'), delete(h_handle); end
    if exist('h_door','var'), delete(h_door); end

    % Plot the coordinate frames of end-effector, handle, and door
    h_e = trplot(g_e.T, 'frame', 'e', 'color', 'g', 'length', 0.2);
    h_h = trplot(g_h, 'frame', 'H', 'color', 'g', 'length', 0.2);
    h_d = trplot(g_d, 'frame', 'd', 'color', 'g', 'length', 0.2);
    
    %% Visualize the handle and door geometry for better clarity (comment or uncomment accordingly)
    h_handle = plotHandle(g_h, 0.02, 0.15);   % Draw cylindrical handle
    h_door = plotDoor(g_d, 1, 2.2);           % Draw rectangular door

    drawnow;  
end

%% Plot joint positions over time
figure('Name','Joint Positions');
plot(time, q_traj, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Joint angles [rad]');
legend('q1','q2','q3','q4','q5','q6');
grid on; title('Joint Position Trajectories');

% Plot joint velocities over time 
figure('Name','Joint Velocities');
plot(time, qd_traj, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Joint velocities [rad/s]');
legend('q̇1','q̇2','q̇3','q̇4','q̇5','q̇6');
grid on; title('Joint Velocity Trajectories');

%% End-effector pose trajectory in base frame 
p_oe_traj = zeros(N,3);       % Position of end-effector in world
q_oe_traj = zeros(N,4);       % Quaternion orientation

for k = 1:N
    g_oe = ur10.fkine(q_traj(k,:));
    p_oe_traj(k,:) = transl(g_oe)';
    q_oe_traj(k,:) = UnitQuaternion(g_oe).double;
end

% Plot position trajectory of end-effector
figure('Name','End-Effector Position Trajectory');
plot3(p_oe_traj(:,1), p_oe_traj(:,2), p_oe_traj(:,3), 'LineWidth', 2);
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('End-Effector Position Trajectory');
legend('Trajectory');

% Plot orientation (quaternion) over time
figure('Name','End-Effector Orientation (Quaternion)');
plot(time, q_oe_traj, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Quaternion Components');
legend('q_0','q_1','q_2','q_3');
grid on;
title('End-Effector Orientation in Unit Quaternion');

%% Relative Pose of End-Effector w.r.t. Handle Frame
p_eh_traj = zeros(N, 3);     % Position of e w.r.t. H
q_eh_traj = zeros(N, 4);     % Orientation quaternion of e w.r.t. H

for k = 1:N
    g_oh = g_oh_traj(:,:,k);
    g_oe = ur10.fkine(q_traj(k,:));
    g_eh = inv(g_oh) * g_oe.T;

    % Extract position and orientation
    p_eh_traj(k,:) = transl(g_eh)';
    q_eh_traj(k,:) = UnitQuaternion(g_eh).double;
end

% Plot relative position of end-effector to handle
figure('Name','Relative Position: End-Effector w.r.t. Handle');
plot3(p_eh_traj(:,1), p_eh_traj(:,2), p_eh_traj(:,3), 'LineWidth', 2);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Relative Position of End-Effector w.r.t. Handle');

% Plot relative orientation (quaternion) over time
figure('Name','Relative Orientation: End-Effector w.r.t. Handle');
plot(time, q_eh_traj, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Quaternion Components');
legend('q_0','q_1','q_2','q_3');
grid on;
title('Relative Orientation of End-Effector w.r.t. Handle');


%%%%%%%%%%%%%%%%%%%%%%%%%%%% HELPER FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function h = plotHandle(g_oh, radius, length)
    % Plot a cylinder representing the handle
    % g_oh: transform of handle frame to world
    % radius: radius of cylinder (meters)
    % length: length of cylinder (meters)
    
    [X, Y, Z] = cylinder(radius); % Create unit cylinder along Z axis
    Z = Z * length;                % Scale height
    Z = Z - length/2;              % Center cylinder on Z axis

    % Rotation matrix to align cylinder from Z-axis to Y-axis
    R = [1 0 0;
         0 0 1;
         0 -1 0];

    n = numel(X);
    points = [X(:)'; Y(:)'; Z(:)'];  % Flatten points
    points = R * points;             % Rotate points

    % Convert to homogeneous coordinates for transformation
    points = [points; ones(1, n)];

    % Transform points to world frame
    transformed = g_oh * points;
    Xt = reshape(transformed(1,:), size(X));
    Yt = reshape(transformed(2,:), size(Y));
    Zt = reshape(transformed(3,:), size(Z));

    % Plot the handle surface
    h = surf(Xt, Yt, Zt, ...
        'FaceColor', [0.5 0.5 0.5], ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 1);
end

function h = plotDoor(g_od, width, height)
    % Plot a rectangular door as a flat patch
    % g_od: transform of door frame to world
    % width: door width along X-axis (meters)
    % height: door height along Z-axis (meters)
    corners = [0 width width 0;  % X coords
               0 0 0 0;          % Y coords (zero thickness)
               0 0 height height; % Z coords (height)
               1 1 1 1];          % Homogeneous coords

    % Transform corners to world frame
    transformed = g_od * corners;
    
    % Extract X, Y, Z for patch
    X = transformed(1,:);
    Y = transformed(2,:);
    Z = transformed(3,:);

    % Draw as a filled polygon (patch)
    h = patch(X, Y, Z, [0 0 0], 'FaceAlpha', 1);  % black color
end