function [ state_dot ] = SatelliteDynamics( t, x, parameters )

    % Intertia matrix
    M = parameters;
    % Input vector x distributed
    position = x(1:3);
    % Creates a 3x3 matrix from vector element x4-x12
    R = reshape(x(4:12),3,3);       
    velocity = x(13:15);
    omega = x(16:18);
    % Skew matrix of omega
    omega_skew = [0        -omega(3)  omega(2);
                  omega(3)  0        -omega(1);
                 -omega(2)  omega(1)  0];
    %Return
    state_dot = [velocity; 
                 reshape(R*omega_skew,9,1);      % Calculate R_dot, and reshapes the 3x3 matrix, to a 9x1 vector
                 Gravity_acceleration(position);
                 -inv(M)*omega_skew*M*omega];
    % The code must return in the order you selected, e.g.:
    %    state_dot =  [velocity;
    %                  orientation_dot;
    %                  acceleration (ac);
    %                  angular acceleration (omega dot)];
end

function g = Gravity_acceleration(position)
    G = 6.676e-11;      % Gravitational constant
    M_t = 5,972e+24;      % Earth's mass
    g = -G*M_t*position/norm(position,2)^3;
end



