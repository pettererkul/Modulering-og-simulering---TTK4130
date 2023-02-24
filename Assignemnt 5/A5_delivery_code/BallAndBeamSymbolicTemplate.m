clear all
clc

% Parameters
syms J M R g To real
% Variables
syms x theta  real
syms dx dtheta real

% Define symbolic variable q for the generalized coordinates
% x and theta
q  = [x; theta];
% Define symbolic variable dq for the derivatives 
% of the generalized coordinates
dq = [dx; dtheta];
% Write the expressions for the position of
% the center of the ball:
p = x*[cos(theta); sin(theta)] + R*[-sin(theta); cos(theta)];
             
% Kinetic energy
T = 1/2*J*dq(2)^2; % kinetic energy of beam

dp = jacobian(p,q)*dq; % linear velocity of ball
T  = T + 1/2*(dp.')*dp; % add linear kinetic energy of ball

I     = 2/5*M*R^2 ; % inertia in rotation of ball
omega = dq(2) + dq(1)/R; % angular velocity of ball

T  = T + 1/2*I*omega^2; % add rotational kinetic energy of ball

T = simplify(T);

% Potential energy
V = M*g*p(2);

% Generalized forces
Q = [0;To];

% Lagrangian
Lag = T - V;

Lag_q = simplify(jacobian(Lag,q)).';
Lag_qdq = simplify(jacobian(Lag_q.',dq));
Lag_dq = simplify(jacobian(Lag,dq)).';
Lag_dqdq = simplify(jacobian(Lag_dq.',dq));

% The equations have the form W*q_dotdot = RHS, with
W = Lag_dqdq;
RHS = Q + simplify(Lag_q - Lag_qdq*dq);

state = [q;dq];
param = [J; M; R; g];

matlabFunction(p, 'file','BallPosition','vars',{state,param});
matlabFunction(W,RHS, 'file','BallAndBeamODEMatrices','vars',{state,To,param});
