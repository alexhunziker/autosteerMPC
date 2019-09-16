%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Potentially nonlinear model %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% X = [y, yaw, yaw_derivative, schwimm, speed]
% Y = [steering_angle, acceleration]

% Sample Time
Ts = 0.1;

% Set Speed for testing
Vx = 4.0;

% Vehicule Parameter set up
m = 30;     % Mass
lf = 0.7;   % Distance mass center/front wheel
lr = 0.3;   % Distance mass center/rear wheel
cf = 300;   % Cornering Stiffness front
cr = 300;   % Cornering Stiffness rear
iz = 100;   % Yaw Moment of Innertia?

% TODO: Non-linear model goes here

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trimming                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Linearize SS model          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Continuous-time model
Ac = [0, 0, -Vx, 0, -1, Vx, 0;
       0, 0, Vx, 0, 1, 0, 0;
       0, 0, 0, 1, 0, 0, 0;
       0, 0, 0, -(2*cf*lf^2+2*cr*lr^2)/iz/Vx, -(2*cf*lf-2*cr*lr)/iz/Vx, 0, 0;
       0, 0, 0,  -Vx-(2*cf*lf-2*cr*lr)/m/Vx, -(2*cf+2*cr)/m/Vx, 0, 0;
       0, 0, 0, 0, 0, -.1, 0;
       0, 0, 0, -Vx, 0, 1/Vx, 0];
   
 Bc = [0, 0, 0, 2*cf*lf/iz, 2*cf/m, 0, 0;
        0, 0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, -2, 0]';
 
 % Position, Yaw, Speed
 Cc = [1, 0, 0, 0, 0, 0, 0;
        0, 1, 0, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 0, 1];
    
 Dc = zeros(5, 3);

ss_model = ss(Ac, Bc, Cc, Dc);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Time Discrete Model         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ss_model_discrete = c2d(ss_model, 0.1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create MPC Block            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% create MPC controller object with sample time
bycicle_mpc = mpc(ss_model_discrete, 0.1);
% specify prediction horizon
bycicle_mpc.PredictionHorizon = 6;
% specify control horizon
bycicle_mpc.ControlHorizon = 2;
% specify nominal values for inputs and outputs
bycicle_mpc.Model.Nominal.U = [0;0];
bycicle_mpc.Model.Nominal.Y = [0;0;0;0];
% specify constraints for MV and MV Rate
bycicle_mpc.MV(1).Min = -0.785398163397448;    % Steering Angle
bycicle_mpc.MV(1).Max = 0.785398163397448;
bycicle_mpc.MV(2).Min = -1;                    % Acceleration/Break input
bycicle_mpc.MV(2).Max = 0.5;

bycicle_mpc.OV(4).Min = 0;                     % Speed limits
bycicle_mpc.OV(4).Max = 4;
bycicle_mpc.OV(3).Min = -pi;                   % Yaw cannot exceed 2pi. necesssary?
bycicle_mpc.OV(3).Max = pi;
% specify overall adjustment factor applied to weights
beta = 0.96079;
% specify weights
bycicle_mpc.Weights.MV = [0 0 0]*beta;
bycicle_mpc.Weights.MVRate = [0.2 0.1 0]/beta;
bycicle_mpc.Weights.OV = [0 0 2 0.1 0]*beta;
bycicle_mpc.Weights.ECR = 100000;              % Constraint Softening, default
% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';           % might be useful. TODO: check docs
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';