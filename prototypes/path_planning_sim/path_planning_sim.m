% Copyright Beach Cleaning Automated
%
% Author: Banky Adebajo
% 
% Path planning simulation for obstacle avoidance and garbage detection

close all; clc;

% Discrete time step
update_rate = 10;
gps_correction_rate = 1;
dt = 1/update_rate;
gps_dt = 1/gps_correction_rate;
delta_max = 25*pi/180;

% Constants
L = 0.67;   % Length from back axle to front axle
gamma = 0;  % Front wheel rotation angle

% Initial State
x0 = [0 0 0]';

% Waypoints
turn_distance = 7; % Distance between adjacent waypoints
waypoints = [];
for i=1:4:10
    waypoints(i,:) = [20 (i-1)*turn_distance/2];
    waypoints(i+1,:) = [20 (i-1)*turn_distance/2 + turn_distance];
    waypoints(i+2,:) = [0 (i-1)*turn_distance/2 + turn_distance];
    waypoints(i+3,:) = [0 (i-1)*turn_distance/2 + 2*turn_distance];
end

% Plastic positions. TODO: Generate randomly
plastic_pos = [10 0.4];

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
Q = [0.5 0 0; 0 0.5 0; 0 0 deg2rad(10)];
Q_gps =  [0.01 0 0; 0 0.01 0; 0 0 deg2rad(10)];
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

% Region Bounds
posMinBound = [-2 -2];
posMaxBound = [25 20];

% Number of obstacles
numObsts = 1;
% Size bounds on obstacles
minLen.a = 1;
maxLen.a = 2;
minLen.b = 1;
maxLen.b = 2;

disp('Generating Random Environment...')
% Random environment generation
obstBuffer = 0.5;
max_tries = 10000; % Maximum number of times to try to generate random environment
seedNumber = rand('state');
% [aObsts,bObsts,obsPtsStore] = polygonal_world(posMinBound, posMaxBound, minLen, maxLen, ...
% numObsts, startPos, [waypoints; plastic_pos], obstBuffer, max_tries);

for i=1:numObsts
    obsCentroid(i,:) = (obsPtsStore(1,2*(i-1)+1:2*i)+obsPtsStore(3,2*(i-1)+1:2*i))/2;
end
disp('Environment Generation Complete...')

% Plot random environment
figure(1); clf;
hold on;
plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, waypoints);

% Grid up the space
dx =.2;
dy = dx;
[X,Y] = meshgrid(posMinBound(1):dx:posMaxBound(1),posMinBound(2):dy:posMaxBound(2));

% Planning Constants
Tmax = 10000;
r0 = 1; % Radius of Repulsion
rc0 = 4;
K_att = 0.2;%0.2; % Attractive
K_rep = 1000; % Repulsive
Vmax = 50; % Upper bound on potential

% Controls Constants
K_steer = 3;
velocity = 1;

total_time = 2;  % Total time from beginning of sim

for i=1:size(waypoints,1)
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
    
%     figure(2); clf; hold on;
%     surf(X,Y,V)
%     axis([posMinBound(1) posMaxBound(1) posMinBound(2) posMaxBound(2) 0 Vmax])
    
    figure(1); hold on;
    
    while(sqrt((mu(1) - end_pos(1))^2 + (mu(2) - end_pos(2))^2) > 0.5)
        
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
        
        % Edge case at the beginning of run
        if shortest_dist_idx < 2
            shortest_dist_idx = 2;
        end
        
        end_point = path(:,shortest_dist_idx)';
        start_point = path(:,shortest_dist_idx - 1)';

        traj_angle = atan2(end_point(2) - start_point(2), end_point(1) - start_point(1));
        [crosstrack_error, next_point] = distanceToLineSegment(start_point,end_point,mu(1:2)');

        % Calculate steering angle
        curr_delta = angleWrap((mu(3) - traj_angle))+ atan2(crosstrack_error,1.5*velocity);
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
        d = QE*sqrt(Qe)*randn(m,1);
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
        K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
        mu = mup + K*(y(:,total_time) - Cd*mup);
        S = (eye(n) - K*Ht)*Sp;

        % Store results
        mup_S(:,total_time) = mup;
        mu_S(:,total_time) = mu;
        K_S(:, :,total_time) = K;
        
        figure(1); hold on;
        plot(x(1,2:total_time),x(2,2:total_time), 'ro')
        plot(mu(1),mu(2), 'bx')
        
        total_time = total_time + 1;
    end
    


%         plot(x(1,2:t),x(2,2:t), 'ro--')
%         plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
        
%         % Plot results
%         figure(2); clf; hold on;
%         plot(0,0,'mx', 'MarkerSize', 6, 'LineWidth', 2)
%         
%         plot(y(1,2:t),y(2,2:t), 'gx--')
%         plot(x(1,2:t),x(2,2:t), 'ro--')
%         plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
%         
%         mu_pos = [mu(1) mu(2)];
%         S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
%         error_ellipse(S_pos,mu_pos,0.75);
%         error_ellipse(S_pos,mu_pos,0.95);
%         legend({'Origin', 'Measurement', 'True State', 'Predicted State', '75% Confidence', '95% Confidence'}, 'AutoUpdate', 'off');
% 
%         title('True State and Predicted State')
%         xlabel('X Position (m)');
%         ylabel('Y Position (m)');
%         axis square
%         
%         axis_width = 100;
%         axis_height = 100;
%         axis([-axis_width/2 axis_width/2 -axis_height/2 axis_height/2])

end


