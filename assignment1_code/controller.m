%% This is my own assignment on 1-D Quadrotor Control with PD Controller
function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

%u = 0;
% PD controller gains
Kp = 25;  % Proportional gain
Kd = 5;   % Derivative gain

    % Extract the current state variables
z = s(1);    % Current height
v_z = s(2);  % Current vertical velocity

    % Extract the desired state variables
z_des = s_des(1);    % Desired height
v_z_des = s_des(2);  % Desired vertical velocity
a_z_des = 0;         % Desired vertical acceleration

    % Calculate the error terms
error_z = z_des - z;          % Position error
error_v_z = v_z_des - v_z;    % Velocity error

    % Calculate the control output (force) using the PD controller formula
u = params.mass*(a_z_des + Kp * error_z + Kd * error_v_z + params.gravity);


% FILL IN YOUR CODE HERE


end

