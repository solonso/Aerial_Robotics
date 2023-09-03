function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
%% My Code
k_p = [1; 1; 50; 2000; 2000; 4000];
k_d = [2; 2; 70; 10; 10; 10];

% Thrust 

r1_t = des_state.acc(1) + k_d(1) * (des_state.vel(1) - state.vel(1)) + k_p(1) * (des_state.pos(1) - state.pos(1)); 

r2_t = des_state.acc(2) + k_d(2) * (des_state.vel(2) - state.vel(2)) + k_p(2) * (des_state.pos(2) - state.pos(2)); 

r3_t = des_state.acc(3) + k_d(3) * (des_state.vel(3) - state.vel(3)) + k_p(3) * (des_state.pos(3) - state.pos(3)); 

 

u1 = params.mass * params.gravity + params.mass * r3_t; 

F = u1; 

% Moment 

%M = zeros(3,1); 

phi_t = (r1_t * sin(des_state.yaw) - r2_t * cos(des_state.yaw)) / params.gravity;

theta_t = (r1_t * cos(des_state.yaw) + r2_t * sin(des_state.yaw)) / params.gravity; 

p_t = 0; 

q_t = 0; 

u2 = [k_p(4) * (phi_t - state.rot(1)) + k_d(4) * (p_t - state.omega(1)); 

      k_p(5) * (theta_t - state.rot(2)) + k_d(5) * (q_t - state.omega(2)); 

      k_p(6) * (des_state.yaw - state.rot(3)) + k_d(6) * (des_state.yawdot - state.omega(3))]; 

M = params.I * u2;

% Thrust
%F = 0;

% Moment
%M = zeros(3,1);

% =================== Your code ends here ===================

end