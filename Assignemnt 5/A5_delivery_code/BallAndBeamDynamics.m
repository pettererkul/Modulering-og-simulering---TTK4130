function x_dot = BallAndBeamDynamics(t, state, parameters)
% Sorting states and states_dot
% [x; theta]
q = state(1:2);
% [x_dot; theta_dot]
q_dot = state(3:4);

% Torque T = 200(x-theta) + 70(x_dot-theta_dot)
T = 200*(q(1)-q(2)) + 70*(q_dot(1) - q_dot(2));

% This function is called using the state, torque and parameters as input,
% and returns W and RHS
[W,RHS] = BallAndBeamODEMatrices(state,T,parameters);

% Returns x_dot which contains q_dot and the W\RHS (inv(W)*RHS)
x_dot = [q_dot; W\RHS];
end


