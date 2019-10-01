m = 30;     % Mass
lf = 0.7;   % Distance mass center/front wheel
lr = 0.3;   % Distance mass center/rear wheel
cf = 3000;   % Cornering Stiffness front
cr = 3000;   % Cornering Stiffness rear
iz = 100;   % Yaw Moment of Innertia?

Ac = [0, 0, -Vx, 0, -1, 1, 0;     % TODO: Gedanke - Hier kann ich y anders brerechnen wenn y als offset zur mitte definiert wird; abzüglich yaw rate
       0, 0, Vx, 0, 1, 0, 0;
       0, 0, 0, 1, 0, 0, 0;
       0, 0, 0, -(cr*lr^2+cf*lf^2)/iz/Vx, -(cr*lr-cf*lf)/iz, 0, 0;
       0, 0, 0, 1-(cr*lr-cf*lf)/m/Vx^2, -(cf+cr)/m/Vx, 0, 0;
       0, 0, 0, 0, 0, -.1, 0;
       0, 0, 0, 0, 0, -1, 0];
   
Bc = [0, 0, 0, cf*lf/iz, -cf/m/Vx, 0, 0;
        0, 0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, -2, 0]';
    
state = [0, 0, 0, 0, 0, 0, 0]';

controls = [0.5, 0.8, 0]';

for i = 1: 100
    state = state + Ac*state + Bc*controls
end