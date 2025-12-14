% Robot parameters
r = 0.05;  
R = 0.15;  
dt = 0.01;  
x0 = 0; y0 = 0; th0 = 0;

% wheel speeds in rad/s
%om1 = 2;  
%om2 = -1;  
%om3 = -1;

% Body velocities
%vx_d = 0.10;  vy_d = 0;  w_d = 0;     % straight line
%vx_d = 0;  vy_d = 0; w_d = 0.2;     % pure spin
vx_d = 0.10; vy_d = 0; w_d = 0.2;   % circle

tau1 = 0.10;  K1 = 1;
tau2 = 0.10;  K2 = 1;
tau3 = 0.10;  K3 = 1;

% Then in each Discrete Transfer Fcn block:
% Wheel 1
Numerator1   =  K1*(1 - exp(-dt/tau1)) ;
Denominator1 = [ 1  -exp(-dt/tau1)     ];

% Wheel 2
Numerator2   =  K2*(1 - exp(-dt/tau2)) ;
Denominator2 = [ 1  -exp(-dt/tau2)     ];

% Wheel 3
Numerator3   =  K3*(1 - exp(-dt/tau3)) ;
Denominator3 = [ 1  -exp(-dt/tau3)     ];

% Controller gains
Kp=5; Ki=50;


