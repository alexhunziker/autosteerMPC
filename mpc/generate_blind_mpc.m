% Sample Time
Ts = 0.1;

% Set arbitrary speed for generating the object
Vx = 4.0;

% Continuous-time model
Ac = [0, 0, -Vx, 0, -1, Vx, 0;     % TODO: Gedanke - Hier kann ich y anders brerechnen wenn y als offset zur mitte definiert wird; abzüglich yaw rate
       0, 0, Vx, 0, 1, 0, 0;
       0, 0, 0, 1, 0, 0, 0;
       0, 0, 0, -(2*cf*lf^2+2*cr*lr^2)/iz/Vx, -(2*cf*lf-2*cr*lr)/iz/Vx, 0, 0;
       0, 0, 0,  -Vx-(2*cf*lf-2*cr*lr)/m/Vx, -(2*cf+2*cr)/m/Vx, 0, 0;
       0, 0, 0, 0, 0, -.1, 0;
       0, 0, 0, 0, 0, -1, 0];
   
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
collision_mpc = mpc(ss_model_discrete, Ts);
% specify prediction horizon
collision_mpc.PredictionHorizon = 6;
% specify control horizon
collision_mpc.ControlHorizon = 3;
% specify nominal values for inputs and outputs
collision_mpc.Model.Nominal.U = [0;0;0];
collision_mpc.Model.Nominal.Y = [0;0;0;0;0;0;0];
% specify constraints for MV and MV Rate
collision_mpc.MV(1).Min = -0.785398163397448;    % Steering Angle
collision_mpc.MV(1).Max = 0.785398163397448;
collision_mpc.MV(2).Min = 0;                    % Acceleration input
collision_mpc.MV(2).Max = 1;
collision_mpc.MV(3).Min = 0;                    % Break input
collision_mpc.MV(3).Max = 1;

collision_mpc.OV(4).Min = 0;                     % Speed limits
collision_mpc.OV(4).Max = 4;
collision_mpc.OV(3).Min = -pi;                   % Yaw cannot exceed 2pi. necesssary?
collision_mpc.OV(3).Max = pi;
% specify overall adjustment factor applied to weights
beta = 0.96079;
% specify weights
collision_mpc.Weights.MV = [0 0 0]*beta;
collision_mpc.Weights.MVRate = [0.2 0.1 0.1]/beta;
collision_mpc.Weights.OV = [0 0 2 0 0 0.1 10]*beta;
collision_mpc.Weights.ECR = 100000;              % Constraint Softening, default
% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';           % might be useful. TODO: check docs
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';

% Eliminate unobservable states
collision_mpc.Model.Plant=minreal(collision_mpc.Model.Plant);