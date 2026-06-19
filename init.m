clear; clc; clearvars; close all

% data

Ix=0.029;           % [kg*m^2]  -   body moment of inertia about the x axis
Iy=0.024;           % [kg*m^2]  -   body moment of inertia about the y axis
Iz=0.005;           % [kg*m^2]  -   body moment of inertia about the z axis

Re=6378;            % [km]  -   radius of earth
h=700;              % [km]      -   heigh of the circular orbit
mu=3.986e5;         % [km^3/s^2]    -   gravitational parameter
Ro=Re+h;            % [km]  -   orbital radius
w0=sqrt(mu/(Ro^3)); % [rad/s]  -   circular orbit's angular velocity
mo = 7.94e15;       % [Tm^3]   -   Earth's magnetic dipole moment
Bo = mo / (Ro*1000)^3;  % [T] average magnetic field
inc = 30*pi/180;    % [rad] orbital inclination
d = [0 0 0.005]';   % [Am^2] residual dipole moment, its direction in the body
                    % frame can be chosen arbitrarily

w_oi=[0 -w0 0]';    % angular velocity vector of o wrt i in orbital
                    % coordinates

q_ic=[0 0 0 1]';    % initial quaternion

w_bi_ic=w_oi;       % definition of the initial angular velocity vector of 
                    % b wrt i in orbital coordinates considering a
                    % nadir-pointing satellite

alpha_phi = 1/(w0^2*(Iz-Iy));
alpha_psi = 1/(w0^2*(Iy-Ix));

G_phi = tf(alpha_phi,[alpha_phi*Ix 0 -1]);
G_theta = tf(1,[Iy 0 0]);
G_psi = tf(alpha_psi,[alpha_psi*Iz 0 1]);

% parameters

K_phi = 4.1144e-6;
T1_phi = 300;
T2_phi = 12;

K_theta = 3.0000e-6;
T1_theta = 300;
T2_theta = 7;

K_psi = 2.5564e-6;
T1_psi = 300;
T2_psi = 7;

% transfer functions of the controller

C_phi = tf([K_phi*T1_phi K_phi],[T2_phi 1]);
C_theta = tf([K_theta*T1_theta K_theta],[T2_theta 1]);
C_psi = tf([K_psi*T1_psi K_psi],[T2_psi 1]);

% open-loop transfer functions

Gtilde_phi = C_phi * G_phi;
Gtilde_theta = C_theta * G_theta;
Gtilde_psi = C_psi * G_psi;

% write the closed-loop transfer functions

W_phi = Gtilde_phi / (1 + Gtilde_phi);
W_theta = Gtilde_theta / (1 + Gtilde_theta);
W_psi = Gtilde_psi / (1 + Gtilde_psi);

% trace the Bode diagrams of the open-loop transfer function

figure;
bode(Gtilde_phi);
title('Bode phi');
grid on;

figure;
bode(Gtilde_theta);
title('Bode theta');
grid on;

figure;
bode(Gtilde_psi);
title('Bode psi');
grid on;

% Verify the step response

figure;
step(W_phi);
title('phi');
grid on;

figure;
step(W_theta);
title('theta');
grid on;

figure;
step(W_psi);
title('psi');
grid on;


