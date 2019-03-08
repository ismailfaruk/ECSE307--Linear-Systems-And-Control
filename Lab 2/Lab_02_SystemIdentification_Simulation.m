% ------------------------------------------------------------------------%
% title    : Lab 1                                                        %
% subtitle : System Identification of a Dynamical System                  %
% date     : Week of 25 September, 2017                                   %
% ------------------------------------------------------------------------%

%% Question 1
k = 1
w = pi
s = tf('s')

for zeta = [0,0.5,1,1.5] 
    H = (k*w^2)/(s^2 + 2*zeta*w*s + w^2)
    step(H, 10)
    title('Step Response of ' + zeta)
    grid on
    stepinfo(H, 'SettlingTimeThreshold', 0.05)
    figure
end



addpath('Interface');

Parameters.Resistance = 1.5;
Parameters.Inertia = 5e-4;

Motor = SimDCMotor(Parameters);


%% Initialization
% Clear all input and output from the Command Window
clc

% Many helper functions are provided in the `Interface` sub-directory.
% We start by adding that directory to Matlab's search path.
addpath('Interface');

% Set parameters for DC motor
Parameters.Resistance = 1.5; % Terminal resistance (Ohm)
Parameters.Inertia    = 5e-4; % Rotor inertia (kg m2)

% Create an object handle for SimDCMotor with specified parameters
Motor = SimDCMotor(Parameters);
%% System Identification : Step Response

% The first method for system identification is to observe the step
% response of the system. To do so, we create a delayed step signal, input
% it as the voltage to the motor, and observe its angular velocity, and
% plot the output signal (in time).

Motor.reset();  % reset the motor internal variable (including time)

voltage = 1;
delay = 1;
duration = 5;

% Drive Motor with a voltage for a duration starting from delay
Motor.drive(voltage, delay, duration);

%=== Plot example ===%
t = Motor.time;
y = Motor.velocity;
u = Motor.voltage;
figure(1)
clf;
yyaxis left
plot(t, y)
ylim([0 1.3*y(end)])
yyaxis right
plot(t, u)
ylim([0 1.3*u(end)])
stepinfo(y, t, 'SettlingTimeThreshold', 0.05)
legend(['Speed (' Motor.Units '/s)'], 'Voltage (V)')
grid on
%% System Identification : Frequency Analysis

% The second method for system identification is to construct the bode plot
% of the system (magnitude and phase of the system's transfer function for
% each frequency). To do so, we send a cosine wave input (a voltage (V)) to
% the motor and observe the output (angular velocity (rad/s)). Since the
% result of sending a cosine wave input is a cosine wave with the same
% frequency  but with different phase and gain, we only need to measure the
% gain and  the phase between the 2 cosine waves. We then repeat this
% operation for  different frequencies. We suggest the following
% frequencies:
% [0.01, 0.1, 0.2, 0.4, 0.5, 0.7, 1.0, 2.5, 5.0]

Motor.reset();

dt = 0.01; % Choose an appropriate sampling time (in seconds)
T  = 6;    % Choose an appropriate total duration of simulation (in seconds)
time  = 0:dt:T;  % A vector containing all time samples

% Simulate driving DC motor
% To simulate a continuous time signal in discrete time, we need to choose
% a sampling time, which we choose as 0.01 seconds. Note that this sampling
% time represent how often the motor is inquired with a new input. The
% motor uses an independent sampling time which dictates when new sensor
% measurement are updated and when new voltage requests are treated. Use
% Motor.setSamplingTime(yourvalue) to change the motor internal sampling
% time.

% Motor.setSamplingTime(dt); % choose an appropriate DC motor sampling time
% Motor.reset();  % reset the motor internal variable (including time)
Omega = 5*2*pi;
for t = time
    % Generate a cosine wave input at current time
    u = cos(Omega*t);
    
    % Drive motor for a duration of dt
    Motor.drive(u, t, dt);
end

%=== Plot example ===%

t = Motor.time;
y = Motor.velocity;
u = Motor.voltage;
delay = 5; % wait for output to reach steady-state.
figure(2)
clf;
hold on;
plot(t(t > delay), y(t > delay))
plot(t(t > delay), u(t > delay))
xlim([5,T])
legend(['Speed (' Motor.Units '/s)'], 'Voltage (V)')
grid on
% ginput(4)

figure(3) % Lissajous method
clf;
plot(y(t > delay), u(t > delay))
legend('Speed vs Voltage')
grid on
% ginput(2)


