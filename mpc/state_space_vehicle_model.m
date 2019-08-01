% X = PositionX, PositionY, YawAngle, YawAngle_Dot, PosY_Dot, Velocity
% Y = Steering Angle, Velocity
x = [0, 0, 0, 0, 0, 0];  % just so it is defined

% Vx (longitudional velocity) actually needs to be updated, model is non
% linear, this needs to be done in an adaptive MPC

% Vehicule Parameter set up
m = 50;     % Mass
lf = 1;     % Distance mass center/front wheel
lr = 1;     % Distance mass center/rear wheel
cf = 19000; % Cornering Stiffness front
cr = 33000; % Cornering Stiffness rear
iz = 2800;  % Yaw Moment of Innertia?

factor_y = sin(x(3)+ x(5));
factor_x = cos(x(3)+ x(5));

% Continuous-time model
A = [0, 0, -Vx, 0, -1, 1;   %  Very crude spaceholder calc for x pos
    %0, 0, 0, 0, 0, factor_x;
    %0, 0, 0, 0, 0, factor_y;
    0, 0, Vx, 0, 1, 0;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, -1/Vx*(cf*lf^2+cr*lr^2)/iz, -(cf*lf-cr*lr)/iz, 0;
    0, 0, 0, -1-1/Vx^2*(cf*lf-cr*lr)/m, -1/Vx*(cf+cr)/m, 0;
    0, 0, 0, 0, 0, -.1        % Very crude definition of speed
    ];

B = [0, 0, 0, 2*cf*lf/iz, 2*cf/m, 0;    % Steering angle
    0, 0, 0, 0, 0, 1]';                 % acceleration/break

C = [1, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 0, 1];

D = zeros(4, 2);

ss_model = ss(A, B, C, D);

ss_model_discrete = c2d(ss_model, 0.1);


