% Sample Time
Ts = 0.1;

% Set arbitrary speed for generating the object
Vx = 4.0;
m = 30;     % Mass
lf = 0.7;   % Distance mass center/front wheel
lr = 0.3;   % Distance mass center/rear wheel
cf = 300;   % Cornering Stiffness front
cr = 300;   % Cornering Stiffness rear
iz = 100;   % Yaw Moment of Innertia?

% Continuous-time model
Ac = [0, 0, -Vx, 0, -1, 1, 0;     % TODO: Gedanke - Hier kann ich y anders brerechnen wenn y als offset zur mitte definiert wird; abz�glich yaw rate
       0, 0, Vx, 0, 1, 0, 0;
       0, 0, 0, 1, 0, 0, 0;
       0, 0, 0, -(2*cf*lf^2+2*cr*lr^2)/iz/Vx, -(2*cf*lf-2*cr*lr)/iz/Vx, 0, 0;
       0, 0, 0,  -Vx-(2*cf*lf-2*cr*lr)/m/Vx, -(2*cf+2*cr)/m/Vx, 0, 0;
       0, 0, 0, 0, 0, -.1, 0;
       0, 0, 0, -Vx, 0, -1, 0];
   
 Bc = [0, 0, 0, 2*cf*lf/iz, 2*cf/m, 0, 0;
        0, 0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, -2, 0]';
 
 % Position, Yaw, Speed
 Cc = [1, 0, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0, 0;
     0, 0, 0, 1, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 1];
    
 Dc = zeros(7, 3);

ss_model = ss(Ac, Bc, Cc, Dc);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Time Discrete Model         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ss_model_discrete = c2d(ss_model, Ts);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create MPC Block            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% create MPC controller object with sample time
blind_mpc = mpc(ss_model_discrete, Ts);
% specify prediction horizon
blind_mpc.PredictionHorizon = 12;
% specify control horizon
blind_mpc.ControlHorizon = 4;
% specify nominal values for inputs and outputs
blind_mpc.Model.Nominal.U = [0;0;0];
blind_mpc.Model.Nominal.Y = [0;0;0;0;0;0;0];
% specify constraints for MV and MV Rate
blind_mpc.MV(1).Min = -0.785398163397448;    % Steering Angle
blind_mpc.MV(1).Max = 0.785398163397448;
blind_mpc.MV(2).Min = 0;                    % Acceleration input
blind_mpc.MV(2).Max = 1;
blind_mpc.MV(3).Min = 0;                    % Break input
blind_mpc.MV(3).Max = 1;

blind_mpc.OV(6).Min = 0;                     % Speed limits
blind_mpc.OV(6).Max = 4;

% specify overall adjustment factor applied to weights
beta = 0.96079;
% specify weights
blind_mpc.Weights.MV = [0 0.1 0.1]*beta;
blind_mpc.Weights.MVRate = [0.2 0.1 0.1]/beta;
blind_mpc.Weights.OV = [0 0 2 0 0 0.1 0]*beta;
blind_mpc.Weights.ECR = 100000;              % Constraint Softening, default
% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';           % might be useful. TODO: check docs
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';

% blind_mpc.Model.Plant=minreal(blind_mpc.Model.Plant);