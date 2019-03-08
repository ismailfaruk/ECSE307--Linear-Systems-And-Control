%% Problem 1 a - f
s = tf('s');
G = 1/((s+1)*(s+2)*(s+3));
step(G);
figure;
rlocus(G);
stepinfo(G)
%% Problem 1 P Controller
C_P = pid(40);
open_loop = series(C_P, G);
H1 = feedback(open_loop,1);
hold on;
figure;
step(H1);
stepinfo(H1)
%% Problem 1 PD Controller
C_PD = pid(40,0,30);
open_loop_PD = series(C_PD, G);
H2 = feedback(open_loop_PD,1);
hold on;
figure;
step(H2);
stepinfo(H2)
%% Problem 1 PI Controller
C_PI = pid(10,10,0);
open_loop_PI = series(C_PI, G);
H3 = feedback(open_loop_PI,1);
hold on;
figure;
step(H3);
stepinfo(H3)
%% Problem 1 PID Controller
C_PID = pid(19,12,8);
open_loop_PID = series(C_PID, G);
H5 = feedback(open_loop_PID,1);
hold on;
figure;
step(H5);
stepinfo(H5)
%% Problem 2 PID Controller Tuning
% opts = pidtuneOptions('DesignFocus','disturbance-rejection');
opts = pidtuneOptions('DesignFocus','reference-tracking');
% opts = pidtuneOptions('DesignFocus','balanced');

% type = 'P';
% type = 'I';
% type = 'PI';
% type = 'PD';
type = 'PID';
C_auto = pidtune(G,type,opts);
open_loop_auto = series(C_auto, G);
H_auto = feedback(open_loop_auto,1);
hold on;
figure;
step(H_auto);
stepinfo(H_auto)

%% Problem 3
bode(G)

%% Problem 3 Zeigler






