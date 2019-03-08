% ------------------------------------------------------------------------%
% title    : Lab 6                                                        %
% subtitle : Speed and Position Control of Qnet DC Motor                               %
% date     : Week of November 20, 2017                                    %
% ------------------------------------------------------------------------%

%% Initialization (can be called only once)
% Clear all input and output from the Command Window
clc,

% Change the default figure window style to docked
set(0,'DefaultFigureWindowStyle', 'docked')

% We start by adding `Interface` sub-directory to Matlab's search path.
addpath('Interface');

% Create an object handle to interface with Qnet DC Motor
Motor = QnetDCMotor();

s = tf('s')

%% PI and PD Controllers Design in MATLAB



% Define the transfer functions here
gain = 28.5;
tau = 0.16;

H = gain/(tau*s + 1);
G = (1/s)*(gain)/(tau*s+1);
%Define the time here
T = 5;       % Simulation duration
dt = 0.01;    % Simulation step time
time = 0:dt:T;

%Complete the reference signals here
w_ref = 75*(time<1) + 25*(time>=1 & time<2) + 75*(time>=2 & time<3) + 25*(time>=3 & time<4) + 75*(time>=4 & time<5)
p_ref = 180*(time<1) + -180*(time>=1 & time<2) + 180*(time>=2 & time<3) + -180*(time>=3 & time<4) + 180*(time>=4 & time<5)

% Plot results here
figure(1); clf;
subplot(2,1,1)
hold on
plot(time, w_ref, '--k')
ylim([0 1.25*max(w_ref)])
xlim([0 T])
legend('Reference \omega (rad/s)')
grid on
subplot(2,1,2)
hold on
plot(time, p_ref, '--r')
ylim([1.25*min(p_ref) 1.25*max(p_ref)])
xlim([0 T])
legend('Reference \theta (degree)')
grid on

% Design PI and PD controllers for speed and position control here
C_PI = pidtune(H, 'pi');
C_PD = pidtune(G, 'PD');

% Change the values of the gains in controllers here
%C_PI = pid(.,.);
%C_PD = pid(.,.);

% Find the closed loop controlled transfer function here
Ho = series(C_PI,H);
Hcl = feedback(Ho,1);

Go = series(C_PD,G);
Gcl = feedback(Go,1);

%Find the output here
w_out = lsim(Hcl, w_ref, time);
p_out = lsim(Gcl, p_ref, time);

% % Plot results
figure(3); clf;
subplot(2,1,1)
hold on
plot(time, w_out, '--k')
hold on
plot(time,w_ref,'b')
ylim([0 1.25*max(w_out)])
xlim([0 T])
legend('Output \omega (rad/s)')
grid on
subplot(2,1,2)
hold on
plot(time, p_out, '--r')
hold on
plot(time,p_ref,'b')
ylim([1.25*min(p_out) 1.25*max(p_out)])
xlim([0 T])
legend('Output \theta (degree)')
grid on

%% PI Controller for Qnet DC Motor Speed Control

Kp = 0.06;
Ki = 0.7;

% Drive the Qnet DC Motor

w_error = zeros(size(time)); %Initialize an error array.

Motor.reset();  % reset the motor internal variable

for n = 1:length(time)
 t_ = time(n);
 w_ = Motor.velocity(t_);
 w_error(n) = w_ref(n)-w_;
 u_ = Kp* w_error(n) + Ki*sum(w_error)*dt;
 Motor.drive(u_, t_, dt);
end
Motor.off();

% Get results of driving Qnet DC Motor

w_motor = Motor.velocity(0, T);
u = Motor.voltage(0, T);

% % Plot results
figure(3); clf;
subplot(2,1,1)
hold on
plot(time, w_motor, '--k')
hold on
plot(time,w_ref,'b')
ylim([0 1.25*max(w_motor)])
xlim([0 T])
legend('Motor output \omega (rad/s)')
grid on
subplot(2,1,2)
hold on
plot(time, u, '--r')
ylim([1.25*min(u) 1.25*max(u)])
xlim([0 T])
legend('Motor input voltage (V)')
grid on

%% PD Controller for Qnet DC Motor Position control

Kp = 0.9;
Kd = 0.0366;

% % Drive the Qnet DC Motor
p_error = zeros(size(time)); %Initialize an array for error.

p_ref = p_ref*pi/180; %change the reference signal from degree to radian

Motor.reset();  % reset the motor internal variable
for n = 1:length(time)  % drive the control signal as well as the output of the system
 t_ = time(n);
 p_ = Motor.angle(t_);
 p_error(n) = p_ref(n)-p_;
 if(n==1)
     u_ = Kp * p_error(1) + Kd/dt * (p_error(1)-pi);
 else
     u_ = Kp * p_error(n) + Kd/dt * (p_error(n)-p_error(n-1));
 end
 
 Motor.drive(u_, t_, dt);
end
Motor.off();

% % Get results of driving Qnet DC Motor

p_motor = Motor.angle(0, T);
u = Motor.voltage(0, T);

p_motor = p_motor*180/pi; % change the units again from radian to degree.
p_ref = p_ref*180/pi;


% Plot results
figure(4); clf;
subplot(2,1,1)
hold on
plot(time, p_motor, '--k')
hold on
plot(time,p_ref,'b')
ylim([1.25*min(p_motor) 1.25*max(p_motor)])
xlim([0 T])
legend('Motor position (degree)')
grid on
subplot(2,1,2)
hold on
plot(time, u, '--r')
ylim([1.25*min(u) 1.25*max(u)])
xlim([0 T])
legend('Motor input voltage (V)')
grid on

%% PD Position Control- Question 4
a = 10*2*pi*(0.4/(pi*0.05));
b = 0;
% 
p_ref = a*(time<1) + b*(time>=1 & time<2)+a*(time>=2 & time<3)...
   +b*(time>=3 & time<4)+a*(time>=4 & time<5)+b*(time>=5);
% 
Kp = 0.9;
Kd = 0.05;

% Drive the Qnet DC Motor
p_error = zeros(size(time)); %Define an array for error

Motor.reset();  % reset the motor internal variable
for n = 1:length(time)
    t_ = time(n);
    p_ = Motor.angle(t_);
    p_error(n) = p_ref(n)-p_;
    if(n==1)
        u_ = Kp * p_error(1) + Kd/dt * (p_error(1)-pi);
    else
        u_ = Kp * p_error(n) + Kd/dt * (p_error(n)-p_error(n-1));
    end
    Motor.drive(u_, t_, dt);
end
Motor.off();

% % Get results of driving Qnet DC Motor
p_motor = Motor.angle(0, T);
u = Motor.voltage(0, T);

% Plot results
figure(5); clf;
subplot(2,1,1)
hold on
plot(time, p_motor, '--k')
hold on
plot(time,p_ref,'b')
ylim([1.25*min(p_motor) 1.25*max(p_motor)])
xlim([0 T])
legend('Motor position (degree)')
grid on
subplot(2,1,2)
hold on
plot(time, u, '--r')
ylim([1.25*min(u) 1.25*max(u)])
xlim([0 T])
legend('Motor input voltage (V)')
grid on