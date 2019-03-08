% ------------------------------------------------------------------------%
% title    : Lab 3                                                        %
% subtitle : Identification of Qnet DC Motor                              %
% date     : Week of 09 October, 2017                                     %
% ------------------------------------------------------------------------%

%% Initialization
% Clear all input and output from the Command Window
clc

% Many helper functions are provided in the `Interface` sub-directory.
% We start by adding that directory to Matlab's search path.
addpath('Interface');

% Create an object handle to interface with Qnet DC Motor
Motor = QnetDCMotor();

%% 3 - Identification by Frequency Analysis

% Frequency vector
FreqVect = [0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0, 2.5, 5.0];
idxFreq = 1; % select the desired index inside the frequency vector

dt = 0.01;              % Choose an appropriate sampling time (in seconds)
duration  = 30;         % Choose an appropriate total duration of simulation (in seconds)
time  = 0:dt:duration;  % A vector containing all time samples

% Simulate driving DC motor
Freq = FreqVect(idxFreq); % select frequency from FreqVect
Motor.setSamplingTime(dt);
Motor.reset;
for t = time
    % Generate a cosinwave input at current time
    u = 2 + cos(2*pi*Freq*t);
    
    % Drive motor for a duration of dt
    Motor.drive(u, t, dt);
end
Motor.off;

%=== Plot example ===%

t = Motor.time;
y = Motor.velocity;
u = Motor.voltage;
delay = 0; % wait for output to reach steady-state.
figure(2)
clf;
yyaxis left
plot(t(t > delay & t <= duration), y(t > delay & t <= duration))
ylabel(['Speed(' Motor.Units '/s)'])
yyaxis right
plot(t(t > delay & t <= duration), u(t > delay & t <= duration))
ylabel('Voltage (V)')
xlim([0,duration])
xlabel('Time (sec)')
legend(['Speed (' Motor.Units '/s)'], 'Voltage (V)')
title(['Input and output vs time at frequency ' num2str(Freq) ' Hz'])
grid on

figure(3) % Lissajous method
clf;
yEllipse = y(t > delay & t <= duration);
uEllipse = u(t > delay & t <= duration);
plot(uEllipse, yEllipse)
xlabel('Voltage (V)')
ylabel(['Speed (' Motor.Units '/s)'])
title(['XY plot for frequency ' num2str(Freq) ' Hz'])
grid on

data = ginput(4); % click 4 times on the graph to retrieve 4 X,Y values

ay = (data(1,2) - data(2, 2))/2;
ax = (data(3,2) - data(4, 2))/2;

fprintf("Ay: %f\n", ay);
fprintf("Y*: %f\n", ax);
%% 4 - Reduced DC Motor Model %% 
s = tf('s');
Hin = 1/((1.16/1000)*s+8.4);
figure(4);
title('Step Response of Hin');
step(Hin);
stepinfo(Hin)
%% 5 - Identification by Step Response Analysis

Motor.reset();  % reset the motor internal variable (including time)
% Drive Motor with a an input voltage for a duration starting from a delay
Motor.drive(2, delay, 5);
Motor.off();

%=== Plot example ===%

t = Motor.time;
y = Motor.velocity;
u = Motor.voltage;
figure(1)
clf;
yyaxis left
plot(t(t>0), y(t>0))
ylim([0 1.3*max(y(t>0))])
ylabel(['Speed(' Motor.Units '/s)'])
yyaxis right
plot(t, u)
ylim([0 1.3*max(u(t>0))])
ylabel('Voltage (V)')
legend(['Speed (' Motor.Units '/s)'], 'Voltage (V)')
grid on
xlim([0 5])
xlabel('Time (sec)')
title('Step Response of Qnet DC Motor')

dcGain = mean(y(t>1 & t<5)) / 2; 

disp(dcGain);
