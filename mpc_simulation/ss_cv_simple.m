% Parameters definition
m = 22;     % (kg)   ETROPOLIS Neo 29er 50W 36V 12Ah
lf = 1.10;  % (m) Distance mass center/front wheel
lr = 0.70;  % (m) Distance mass center/rear wheel
cf = 900;   % (N/rad) lateral static stiffness
cr = 1370;  % (N/rad) lateral static stiffness
iz = 1/12 * (m) * (lr+lf)^2;  % (kgm^2) moment of inertia of bicycle --> 1/12 m l^2 = 1/12 * (22 kg) * (1.18 m)^2

% Sample Time
Ts = 0.01;

% Set arbitrary speed for generating the object
Vx = 2.0;

% Create State Space Matrixes
Ac = [1, 0, -Vx, 0, +Vx, 0;
       0, 0, 1, 0, 0, 0;
       0, 0, -(cr*lr^2+cf*lf^2)/(iz*Vx), (cr*lr-cf*lf)/iz, 0, 0;
       0, 0, -1-(cf*lf-cr*lr)/(m*Vx^2), -(cf+cr)/(m*Vx), 0, 0;
       0, 0, 0, 0, -1, 0;
       0, 0, 0, 0, 0, -1/10];
       
Bc = [0, 0, cf*lf/iz, -cf/(m*Vx), 0, 0;
      0, 0, 0, 0, -1, 0;
      0, 0, 0, 0, 0, 1;
      0, 0, 0, 0, 0, -2]';
 
Cc = eye(6);
    
Dc = zeros(6, 4);

ss_model = c2d(ss(Ac, Bc, Cc, Dc), Ts);