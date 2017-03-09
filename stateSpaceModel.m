%% State-Space catcher thing
clear; close all

%% define vars
% motor parameters
    Kt = 6.66;       % Nm/A torque constant based off BSM-50N-275 ABB servo
    %Kv = 1;         % back-emf constant (incase have this instead)
    L = .0332;       % H motor inductance
    r = 16;          % ohms  motor resistance
% physical parameters    
    m1 = 3;          % kg mass of puck
    m2 = 5;          % kg mass of platform
    k = 18500;       % N/m mechanical spring element
    b1 = 500;       % Ns/m
    J = 11240;       % kgm^2 moment of inertial
    b2 = 60*.02;     % Ns/m bearing friction
    rWheel = .025;   % m  drive wheel radius
% transformer
    TFrp = rWheel;     % transformer translation to rotation
    TFmotor = Kt;      % transformer rotation to electrical

% stuff
TFelement = (b1*TFrp^2*TFmotor)/(J*TFmotor*m2*TFrp^2*TFmotor); % calculating elements in A
dt=.001;
t = 0:dt:.3;  % time array

%% define matricies
x0 = [...
    2.445;... % Vm1
    0;... % Vm2
    0;... % Fk
    0;];  %iL
A = [...
    -b1/m1, b1/m1, -1/m1,0;...
    TFelement, -TFelement, TFelement, TFelement/(TFrp*TFmotor);...
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
u = NaN*ones(1,length(t));
for j = 1:length(t)
    u(j) = 0;
end
y = lsim(sys,u,t,x0);
a = cat(1,NaN, diff(y)/dt);

%% plot things
figure
plot(t,y);
%impulse(sys,t);
grid on
title('Velocity Response');
xlabel('time (s)')
ylabel('velocity of puck (m/s)')

figure
plot(t,a);
grid on
title('Acceleration Response');
xlabel('time (s)')
ylabel('acceleration of puck (m/s^2)')
%figure(2)
%rlocus(plant);


