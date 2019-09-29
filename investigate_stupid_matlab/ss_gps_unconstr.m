m = 30;     % Mass
lf = 0.7;   % Distance mass center/front wheel
lr = 0.3;   % Distance mass center/rear wheel
cf = 3000;  % Cornering Stiffness front
cr = 3000;  % Cornering Stiffness rear
iz = 100;   % Yaw Moment of Innertia

% Sample Time
Ts = 0.01;

% Set arbitrary speed for generating the object
Vx = 2.0;

Ac = [0, 1, 0, 0;
       0, -(cr*lr^2+cf*lf^2)/(iz*Vx), -(cr*lr-cf*lf)/iz, 0;
       0, 1-(cr*lr-cf*lf)/(m*Vx^2), -(cf+cr)/(m*Vx), 0;
       0, 0, 0, -1/10];
       
Bc = [0, cf*lf/iz, -cf/(m*Vx), 0;
      0, 0, 0, 1;
      0, 0, 0, -2]';
 
% Position, Yaw, Speed
Cc = eye(4);
    
Dc = zeros(4, 3);

ss_gps_s = c2d(ss(Ac, Bc, Cc, Dc), Ts);