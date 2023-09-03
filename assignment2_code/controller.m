function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
% PD controller gains for position and velocity control
    %Using Zeiglers mehtod
    Tu_y = 0.6;
    Ku_y = 13;
    Td_y = Tu_y/8;
    
    Tu_z = 0.2857;
    Ku_z = 500;
    Td_z = Tu_z/8;
    
    Tu_p = 0.133;
    Ku_p = 2000;
    Td_p = Tu_p/8;
    
    %Y wieghts
    %Kp_y = 0;
    Kp_y = 0.8*Ku_y*10;    % Proportional gain for y position control
    %Kd_y = 0;
    Kd_y = Kp_y*Td_y*10;    % Derivative gain for y velocity control
    %Z weights
    %Kp_z = 0;
    Kp_z = 0.8*Ku_z*10;    % Proportional gain for z position control
    %Kd_z = 0;
    Kd_z = Kp_z*Td_z*10;    % Derivative gain for z velocity control
    
    %Kp_phi = 500;
    Kp_phi = 0.8*Ku_p*10;    % Proportional gain for z position control
    %Kd_phi = 0;
    Kd_phi = Kp_phi*Td_p*10;    % Derivative gain for z velocity control


    % Extract the current state variables
    y = state.pos(1);        % Current position in y direction
    z = state.pos(2);        % Current position in z direction
    y_dot = state.vel(1);    % Current velocity in y direction
    z_dot = state.vel(2);    % Current velocity in z direction
    phi_norm = state.rot;
    phi_dot = state.omega;
    %phi_c_dot = 0;
    %phi_c_dot_dot = 0;
    
    % Extract the desired state variables
    y_des = des_state.pos(1);      % Desired position in y direction
    z_des = des_state.pos(2);      % Desired position in z direction
    y_dot_des = des_state.vel(1);  % Desired velocity in y direction
    z_dot_des = des_state.vel(2);  % Desired velocity in z direction
    y_dot_dot = des_state.acc(1);  % Desired acceleration in y direction
    z_dot_dot = des_state.acc(2);  % Desired acceleration in z direction

    
    % Calculate the error terms for position and velocity control
    error_y = y_des - y;             % Position error in y direction
    error_z = z_des - z;             % Position error in z direction
    error_y_dot = y_dot_des - y_dot; % Velocity error in y direction
    error_z_dot = z_dot_des - z_dot; % Velocity error in z direction

    % Calculate the desired controls using PD controller for position and velocity control
    phi_c = (-1/params.gravity)*(y_dot_dot + Kd_y*error_y_dot + Kp_y*error_y);
    u1 = params.mass*(params.gravity + z_dot_dot + Kd_z*error_z_dot + Kp_z*error_z);
    u2 = params.Ixx*(Kd_phi*(-phi_dot) + Kp_phi*(phi_c - phi_norm));
% FILL IN YOUR CODE HERE

end

