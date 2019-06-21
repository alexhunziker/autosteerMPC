% X = PositionX, PositionY, YawAngle, YawAngle_Dot, PosY_Dot, Velocity
% Y = Steering Angle, Velocity

% Vx (longitudional velocity) actually needs to be updated, model is non
% linear, this needs to be done in an adaptive MPC

% Vehicule Parameter set up
m = 50;     % Mass
lf = 1;     % Distance mass center/front wheel
lr = 1;     % Distance mass center/rear wheel
cf = 19000; % Cornering Stiffness front
cr = 33000; % Cornering Stiffness rear
iz = 2800;  % Yaw Moment of Innertia?

% Continuous-time model
A = [0, 0, -Vx, 0, -1, 1;   %  Very crude spaceholder calc for x pos
    0, 0, Vx, 0, 1, 0;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx, -(2*Cf*lf-2*Cr*lr)/Iz/Vx, 0;
    0, 0, 0, -Vx-(2*Cf*lf-2*Cr*lr)/m/Vx, -(2*Cf+2*Cr)/m/Vx, 0;
    0, 0, 0, 0, 0, -.1        % Very crude definition of speed
    ];

B = [0, 0, 0, 2*Cf*lf/Iz, 2*Cf/m, 0;    % Steering angle
    0, 0, 0, 0, 0, 1]';                 % acceleration/break

C = [1, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 0, 1];

D = zeros(4, 2);

ss_model = ss(A, B, C, D);

ss_model_discrete = c2d(ss_model, 0.1);


