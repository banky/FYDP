% Copyright Beach Cleaning Automated
%
% Author: Banky Adebajo
% 
% Path planning simulation for obstacle avoidance and garbage detection

close all; clc;

% Initialize Video
% videoobj = VideoWriter('vid_13.mp4','MPEG-4');
% truefps = 1;
% videoobj.FrameRate = 10; %Anything less than 10 fps fails.
% videoobj.Quality = 100;
% open(videoobj);

% Region Bounds
posMinBound = [-2 -2];
posMaxBound = [25 25];

% Discrete time step
update_rate = 10;
gps_correction_rate = 1;
dt = 1/update_rate;
gps_dt = 1/gps_correction_rate;
delta_max = 25*pi/180;

% Robot Properties
L = 0.67;   % Length from back axle to front axle
sensor_range_max = 4;
sensor_theta_max = pi / 4;
sensor_meas_type = 3; % 1 - range, 2 - bearing, 3 - both

% Initial State
x0 = [0 0 0]';

% Waypoints
turn_distance = 5; % Distance between adjacent waypoints
waypoints = [];
for i=1:4:5
    waypoints(i,:) = [posMaxBound(1)-2 (i-1)*turn_distance/2];
    waypoints(i+1,:) = [posMaxBound(1)-2 (i-1)*turn_distance/2 + turn_distance];
    waypoints(i+2,:) = [posMinBound(1)+2 (i-1)*turn_distance/2 + turn_distance];
    waypoints(i+3,:) = [posMinBound(1)+2 (i-1)*turn_distance/2 + 2*turn_distance];
end

% Plastic positions
num_plastic = 5;
plastic_map = rand(num_plastic,2);
plastic_map = [(posMaxBound(1)-2 - posMinBound(1))*plastic_map(:,1) + posMinBound(1) ...
    (posMaxBound(2)-2 - posMinBound(2))*plastic_map(:,2) + posMinBound(2)];

% Prior
mu = [0 0 0]'; % mean (mu) - Prior Belief
S = 1*eye(3);% covariance (Sigma) - Prior Covariance

% Discrete motion model
Ad = eye(3);
Bd = zeros(3,1);
Cd = eye(3);
Dd = 0;

% Motion model covariance. Account for slippage etc. here
R = [0.01^2 0 0; 0 0.01^2 0; 0 0 (deg2rad(0.1)^2)];
[RE, Re] = eig (R);

% Measurement model
Q = [0.1 0; 0 deg2rad(10)];
Q_gps =  [0.5 0 0; 0 0.5 0; 0 0 deg2rad(10)];
[QE, Qe] = eig(Q);
[QE_gps, Qe_gps] = eig(Q_gps);

% Simulation Initializations
Tf = 15;    % Time in seconds for simulation to run
T = 0:dt:Tf;
n = length(Ad(1,:));
x = zeros(n,length(T));
x(:,1) = x0;

m = length(Cd(1,:));
y = zeros(m,length(T));
mup_S = zeros(n,length(T));     % Predicted state belief over time
mu_S = zeros(n,length(T));      % State belief over time
K_S = zeros(n, n, length(T));   % Kalman Gain over time

% Set up environment
startPos = x0(1:2, :)';

% Number of obstacles
numObsts = 5;
% Size bounds on obstacles
minLen.a = 1;
maxLen.a = 2;
minLen.b = 1;
maxLen.b = 2;

% Random environment generation
obstBuffer = 0.5;
max_tries = 10000; % Maximum number of times to try to generate random environment
seedNumber = rand('state');
[aObsts,bObsts,obsPtsStore] = polygonal_world(posMinBound, posMaxBound, minLen, maxLen, ...
numObsts, startPos, [waypoints; plastic_map], obstBuffer, max_tries);

for i=1:numObsts
    obsCentroid(i,:) = (obsPtsStore(1,2*(i-1)+1:2*i)+obsPtsStore(3,2*(i-1)+1:2*i))/2;
end

% Plot random environment
figure(1); clf;
hold on;
plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, plastic_map);
title('True State and Predicted State')
xlabel('X Position (m)');
ylabel('Y Position (m)');

% Grid up the space
dx =.2;
dy = dx;
[X,Y] = meshgrid(posMinBound(1):dx:posMaxBound(1),posMinBound(2):dy:posMaxBound(2));

% Planning Constants
Tmax = 10000; % If it takes us too long to get there, probably stuck
r0 = 0.5; % Radius of Repulsion
rc0 = 4;
K_att = 0.2;%0.2; % Attractive
K_rep = 1000; % Repulsive
Vmax = 50; % Upper bound on potential

% Controls Constants
K_steer = 3;
K_smoothing = 1.5;
velocity = 1;

total_time = 2;  % Total time from beginning of sim. Starts from 2 since initial state is already known

picking_garbage = 0;    % On our way to pick garbage

i = 1;
while i < size(waypoints,1)
    % Calculate potential field at each grid point. For visual only
%     [V, gV] = gen_potential_field(X, Y, waypoints(i,:), numObsts, obsCentroid, obsPtsStore);
    
    t = 1;  % Starting time index for path planning
    path = mu(1:2);
    end_pos = waypoints(i,:);

    gVcur = [1 1]; % Initialize current gradient vector
    while ((norm(gVcur)>0.01) && (t<Tmax))
        % Generate path to get to next waypoint
        t = t+1;
        pos = path(:,t-1)';
        gVcur = K_att*(pos - end_pos);
        for j=1:numObsts
            curobs = obsPtsStore(:,2*(j-1)+1:2*j);
            if (inpolygon(pos(1),pos(2),curobs(:,1),curobs(:,2)))
                gVcur = [NaN NaN];
            else
                curpoly = [curobs curobs([2:end, 1],:)];
                [minD, minPt, d, pt, ind] = min_dist_to_edges(pos, curpoly);
                if (minD < r0)
                    gVcur = gVcur + K_rep*(-1/minD+1/r0)*(pos-minPt)/minD^(3);
                end
                % Add potential of distance to center, to avoid getting
                % stuck on flat walls
                centD = norm(pos - obsCentroid(j,:));
                if (centD < rc0)
                    gVcur = gVcur + K_rep*(-1/centD+1/rc0)*(pos-obsCentroid(j,:))/centD^(3);
                end
            end
        end
        path(:,t) = path(:,t-1) - dx.*gVcur';
    end
    
    % Smooth out the path
    path(1,1:t) = smooth(path(1,1:t), 20);
    path(2,1:t) = smooth(path(2,1:t), 20);
    
%     figure(1); hold on;
%     plot(path(1, 1:t), path(2, 1:t));
    
%     figure(2); clf; hold on;
%     surf(X,Y,V)
%     axis([posMinBound(1) posMaxBound(1) posMinBound(2) posMaxBound(2) 0 Vmax])
    
    reached_feature = 0;    % Whether we have reached the current waypoint
    TOL = 0.7;  % Distance to waypoint to indicate we have reached it

    figure(1); hold on;
    
    while(~reached_feature)
        
        % Get positions of garbage in view
        [features, featureInViewFlag] = get2dpointmeasurement(plastic_map,...
            x(:,total_time-1),sensor_range_max,sensor_theta_max,Q,sensor_meas_type);
        % Find first feature in view
        garbage_index = find(featureInViewFlag==1,1);
        % Determine coordinates of feature used for localization
        selectedFeature = plastic_map(garbage_index,:);
        
        % Not currently on the way to pick some garbage
        % and we see garbage and we are not already on top of garbage
        if (~picking_garbage && ~isempty(selectedFeature) && ...
            sqrt((mu(1)-selectedFeature(1))^2 + (mu(2)-selectedFeature(2))^2) > TOL)
            if (i == 1) % Shift first waypoint over
                waypoints = [selectedFeature; waypoints];
            else
                waypoints = [waypoints(1:i-1,:); selectedFeature; waypoints(i:end,:)];
            end
            
            picking_garbage = 1;
            break;
        end
        
        % If heading to waypoint and not plastic, use a less strict tolerance
        if (~picking_garbage && isempty(selectedFeature))
            TOL = 2;
        end
        
        % Find point on path that is closest to robot
        shortest_dist = -1;
        shortest_dist_idx = 0;
        for j=1:size(path,2)
            curr_dist = sqrt((mu(1) - path(1,j))^2 + (mu(2) - path(2,j))^2);
            if (shortest_dist < 0 || curr_dist < shortest_dist)
                shortest_dist = curr_dist;
                shortest_dist_idx = j;
            end
        end
        
        % Edge case at the beginning of path
        if shortest_dist_idx < 2
            shortest_dist_idx = 2;
        end
        
         end_point = path(:,shortest_dist_idx)';
         start_point = path(:,shortest_dist_idx - 1)';
         
         end_point = [0, 50];
         start_point = [0, 0];
         mu = [-50; 50; 0.3];

        traj_angle = atan2(end_point(2) - start_point(2), end_point(1) - start_point(1));
        [crosstrack_error, next_point] = distanceToLineSegment(start_point,end_point,mu(1:2)');

        % Calculate steering angle
        curr_delta = angleWrap((mu(3) - traj_angle))+ atan2(crosstrack_error,K_smoothing*velocity);
        delta = max(-delta_max,min(delta_max, K_steer*curr_delta));
        w = velocity;

        % Get current state
        % Select a motion disturbance
        e = RE*sqrt(Re)*randn(n,1);

        % Set Bd based on current pose and steering angle
        theta = x(3,total_time-1);
        Bd = [cos(theta); sin(theta); -(tan(delta)/L)];

        % Update State
        x(:,total_time) = Ad*x(:,total_time-1) + dt*Bd*w + e;

        % Take Measurement
        d = QE_gps*sqrt(Qe_gps)*randn(m,1);
        y(:, total_time) = Cd*x(:,total_time) + dt*Dd*w + d;

        % Extended Kalman Filter Estimation
        % Prediction Update
        Gt = [1, 0, -w*sin(mu(3))  % Linearization
            0, 1,  w*cos(mu(3))
            0, 0,          1];
        mup = Ad*mu + dt*Bd*w;
        Sp = Gt*S*Gt' + R;

        % Measurement Update
        Ht = eye(3);    % Linearization
        K = Sp*Ht'*inv(Ht*Sp*Ht'+Q_gps);
        mu = mup + K*(y(:,total_time) - Cd*mup);
        S = (eye(n) - K*Ht)*Sp;

        % Store results
        mup_S(:,total_time) = mup;
        mu_S(:,total_time) = mu;
        K_S(:, :,total_time) = K;
        
        figure(1); hold on;
        p_x = plot(x(1,2:total_time),x(2,2:total_time), 'ro');
        p_mu = plot(mu(1),mu(2), 'bx');
        legend([p_x p_mu], 'True State', 'Predicted State', 'AutoUpdate', 'off');
        
        total_time = total_time + 1;
        
        % Check if we have reached the next feature
        reached_feature = sqrt((mu(1) - end_pos(1))^2 + (mu(2) - end_pos(2))^2) < TOL;
    
%         set(gcf, 'Position', [100, 100, 640, 640]);
%         F = getframe(gcf);
%         writeVideo(videoobj, F);
    end
    
    % If we are on our way to pick garbage
    if (picking_garbage == 1 && ~reached_feature)
        continue;
    elseif (picking_garbage ==1 && reached_feature)
        plastic_map(garbage_index, :) = [];
    end
    
    picking_garbage = 0;
    i = i + 1;
end

% close(videoobj);


