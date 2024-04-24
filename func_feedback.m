
% Compute control action using feedback linearization
%
% Inputs:
%       x: states
%           q1
%           q2
%           q3
%           dq1
%           dq2
%           dq3
%       alpha: Bezier coefficients for q2 and q3
%           alpha2 (1st to 5th coefficients)
%           alpha3 (1st to 5th coefficients)
%       s_params: for gait timing
%           q1_min
%           delq
%
% Outputs:
%       u: control action
%
function u = func_feedback(x,alpha,s_params)
% gains
kp1 = 200;
kp2 = 200;
kd1 = 100;
kd2 = 100;

% Seperating inputs
q = x(1:3);
dq = x(4:6);

% Get model parameters
[r,m,Mh,Mt,l,g] = func_model_params;
params = [r,m,Mh,Mt,l,g];

% Seperate Bezier coefficients
alpha2 = alpha(1:5);
alpha3 = alpha(6:10);

% Seperate s_params
q1_min = s_params(1);
q1_max = s_params(2);
delq = q1_max - q1_min;

% Gait timing variable
% Inputs:
%       q1
%       q1_min
%       delq
s = func_gait_timing(q(1), q1_min, q1_max);

% Building output function
%       y = h(x) = Hq - hd

% Get D,C,G,B matrices
% Inputs:
%       q = [q1, q2, q3]
%       dq = [dq1, dq2, dq3]
%       params = [r,m,Mh,Mt,l,g]
[D,C,G,B] = func_compute_D_C_G_B(q,dq,params);

% Defining fx and gx
fx = [dq; D\(-C*dq - G)];
gx = [zeros(3,2); D\B];


M=4;
% find y
b2=bezier(s,M,alpha2);
b3=bezier(s,M,alpha3);
h = [q(2) - b2; q(3) - b3];         
y = h;

% Calculating y_dot
dh_dx = zeros(2,6);
dh_dx(1,1) = -d_ds_bezier(s,M,alpha2)/delq;
dh_dx(2,1) = -d_ds_bezier(s,M,alpha3)/delq;
dh_dx(1,2) = 1;
dh_dx(2,3) = 1;
Lfh = dh_dx*fx;
dy = Lfh;

%%%% PD controller
Kp = [kp1,0; 0,kp2];
Kd = [kd1,0; 0,kd2];
v = Kp*h+Kd*Lfh;


dh_dq = zeros(2,3);
dh_dq(1,1) = -d_ds_bezier(s,M,alpha2)/delq;
dh_dq(2,1) = -d_ds_bezier(s,M,alpha3)/delq;
dh_dq(1,2) = 1;
dh_dq(2,3) = 1;

%%%% Feedback linerization:
dLfh = func_compute_dLfh([s,delq],dq(1),[alpha2,alpha3]);
L2fh = dLfh*fx;
LgLfh = dh_dq*(inv(D))*B;

% Control action that uses feedback linearization with PD controller
u_star = -inv(LgLfh)*L2fh;
u = u_star-v;

end