%% State-Space catcher thing
clear; close all

%% define vars
% motor parameters
    Kt = 1;         % torque constant = 2NBlr
    Kv = 1;         % back-emf constant (incase have this instead)
    L = 1;          % H motor inductance
    r = 1000;       % ohms  motor resistance
% physical parameters    
    m1 = 10;        % kg mass of platform
    m2 = 3;         % kg mass of puck
    k = 1000;       % N/m mechanical spring element
    b1 = 1000;      % Ns/m
    J = 1;          % kgm^2 moment of inertial
    b2 = 1000;      % Ns/m bearing friction
    p1 = 5;         % cm  pulley 1
    p2 = 5;         % cm pulley 2
    pRatio = p2/p1;
% transformer
    TFbelt = pRatio;     % transformer translation to rotation
    TFmotor = Kt;    % transformer rotation to electrical

% stuff
Vs = 12;        % volts voltage supply
TFelement = (b1*TFbelt^2*TFmotor)/(J*TFmotor*m2*TFbelt^2*TFmotor); % calculating elements in A
t = 0:.001:1;  % time array

%% define matricies
x0 = [...
    0;...
    0;...
    0;...
    0;];
A = [...
    -b1/m1, b1/m1, -1/m1,0;...
    TFelement, -TFelement, TFelement, TFelement/(TFbelt*TFmotor);...
    k, -k,0,0;...
    0,0,0,-r/L;];
    
B = [...
    0;...
    0;...
    0;...
    1/L];
C = [1,0,0,0];
D = 0;

%% create state-space
sys = ss(A,B,C,D);
%sys.InputName = 'Vs';
%sys.OutputName = {'Vm1';'Vm2'};
F1 = impulse(sys,t);

%% plot things
figure
plot(t,F1);
title('Impulse Response');
xlabel('time (s)')
ylabel('velocity of m1 (m/s)')

figure(2)
rlocus(sys);


