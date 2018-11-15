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

% Constants
L = 0.67;   % Length from back axle to front axle
gamma = 0;  % Front wheel rotation angle

% Initial State
x0 = [0 0 0]';

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

% Main loop
for t=2:length(T)
    % Set inputs at timestep
    if t < 50
        w = 1;
        gamma = 0;
    else
        w = 1;
        gamma = pi/8;
    end
    
    % Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    
    % Set Bd based on current pose and steering angle
    theta = x(3,t-1);
    Bd = [cos(theta); sin(theta); (tan(gamma)/L)];
    
    % Update State
    x(:,t) = Ad*x(:,t-1) + Bd*w + e;
    
    % Take Measurement
    d = QE*sqrt(Qe)*randn(m,1);
    y(:, t) = Cd*x(:,t) + Dd*w + d;
    
    % Extended Kalman Filter Estimation
    % Prediction Update
    Gt = [1, 0, -w*sin(mu(3))  % Linearization
          0, 1,  w*cos(mu(3))
          0, 0,          1];
    mup = Ad*mu + Bd*w;
    Sp = Gt*S*Gt' + R;
    
    % Measurement Update
    Ht = eye(3);    % Linearization
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    mu = mup + K*(y(:,t) - Cd*mup);
    S = (eye(n) - K*Ht)*Sp;
    
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    K_S(:, :,t) = K;
    
    % Plot results
    figure(1); clf; hold on;
    plot(0,0,'mx', 'MarkerSize', 6, 'LineWidth', 2)
    
    plot(y(1,2:t),y(2,2:t), 'gx--')
    plot(x(1,2:t),x(2,2:t), 'ro--')
    plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
    
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    legend({'Origin', 'Measurement', 'True State', 'Predicted State', '75% Confidence', '95% Confidence'}, 'AutoUpdate', 'off');

    title('True State and Predicted State')
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    axis square
    
    axis_width = 100;
    axis_height = 100;
    axis([-axis_width/2 axis_width/2 -axis_height/2 axis_height/2])
end


