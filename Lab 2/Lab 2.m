

%% Question 1
k = 1
w = pi
s = tf('s')

for zeta = [0,0.5,1,1.5] 
    H = (k*w^2)/(s^2 + 2*zeta*w*s + w^2)
    step(H, 10)
    title("Step Response of " + zeta)
    grid on
    stepinfo(H, 'SettlingTimeThreshold', 0.05)
    figure
end



addpath('Interface');

Parameters.Resistance = 1.5;
Parameters.Inertia = 5e-4;

Motor = SimDCMotor(Parameters);


%%
freq = [0.01
0.1
0.2
0.4
0.5
0.7
1.0
2.5
5.0
];

b = [20
20.02
19.83
10.73
0.6421
-6.981
-4.716
-0.7967
-0.1965
];

Ay = [20
20.42
21.66
23.81
21.2
12.37
5.709
0.8269
0.2026
];

figure(10)
subplot(2,1,1)
% Magnitude Bode Plot
semilogx(freq, 20*log10(Ay));
title('Magnitude (db) vs Frequency (Hz)')
subplot(2,1,2)
% Phase Bode Plot
semilogx(freq, -acos(b./Ay)*180/pi);
title('Phase (deg) vs Frequency (Hz)')


