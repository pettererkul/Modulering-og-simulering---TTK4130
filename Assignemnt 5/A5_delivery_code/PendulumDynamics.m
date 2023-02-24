function x_dot = PendulumDynamics(t, state, parameters)
% [x; theta1; theta2]
q = state(1:3);
% [x_dot; theta1_dot; theta2_dot]
q_dot = state(4:6);

% F = -10*x - x_dot
F = -10*q(1) - q_dot(1);

[W,RHS] = PendulumODEMatrices(state,F,parameters);

% x_dot = [q_dot; (d^2*L/dq_dot^2)^1*(Q + dL/dq - d^2*L/dq_dot*dq * q_dot)]
%                 ( this is W    ) \ ( this is RHS                       )

x_dot = [q_dot; W\RHS];
end
