m = 22;     %  (kg)   ETROPOLIS Neo 29er 50W 36V 12Ah
lf = 1.10;  % Distance mass center/front wheel
lr = 0.70;  % Distance mass center/rear wheel
cf = 7*10^4; %(N/rad) lateral static stiffness L1 Tire-- http://www.ein.org.pl/sites/default/files/2014-01-11.pdf
cr = 7*10^4; %(N/rad) lateral static stiffness L1 Tire-- http://www.ein.org.pl/sites/default/files/2014-01-11.pdf
iz = 1/12 * (m) * (lr+lf)^2;  % (kgm^2) moment of inertia of bicycle --> 1/12 m l^2 = 1/12 * (22 kg) * (1.18 m)^2

% Sample Time
Ts = 0.01;

% Set arbitrary speed for generating the object
Vx = 2.0;

Ac = [1, 0, -Vx, 0, +Vx, 0;
       0, 0, 1, 0, 0, 0;
       0, 0, -(cr*lr^2+cf*lf^2)/(iz*Vx), -(cr*lr-cf*lf)/iz, 0, 0;
       0, 0, 1-(cr*lr-cf*lf)/(m*Vx^2), -(cf+cr)/(m*Vx), 0, 0;
       0, 0, 0, 0, -.99, 0;
       0, 0, 0, 0, 0, -1/10];
       
Bc = [0, 0, cf*lf/iz, -cf/(m*Vx), 0, 0;
      0, 0, 0, 0, -.99, 0;
      0, 0, 0, 0, 0, 1;
      0, 0, 0, 0, 0, -2]';
 
% Position, Yaw, Speed
Cc = eye(6);
    
Dc = zeros(6, 4);

ss_model = c2d(ss(Ac, Bc, Cc, Dc), Ts);