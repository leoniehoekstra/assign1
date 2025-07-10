% ==============================================================
% Filename   : assign1prep.m
% Assignment : 1
% Students   : Steyn Fokkema, Leonie Hoekstra, Sam Rotgans
% ==============================================================
clear;
close all;
clc;

%% Assignment 1.a: 
% Initialise variables
ts = 0.01; fs = 1/ts; ws = 2*pi*fs;
N = 10000;

% PRBS is chosen because it excites a wide range of frequencies. 
% This makes the signal useful for both time-domain and frequency-domain analysis. 
u_prbs = idinput(N, 'prbs');
ref.time = []; 
ref.signals.values = u_prbs;

% Run the simulation
sim('assign1ol.mdl');

data_prbs = iddata(y.signals.values, u.signals.values, 1/fs);
data_prbs = detrend(data_prbs); % DC offset remove

figure(1);
impulse_prbs = cra(data_prbs, 200, 10);
title('Impulse response estimate (PRBS)');

figure(2);
freq_prbs = spa(data_prbs);
bodeplot(freq_prbs);
grid on;
title('Frequency response estimate (PRBS)');

% multi-sine
r = idinput(N,'sine',[4/50-1/N 40/50+1/N],0.1*[-1 1],[10 10 1]);
ref.signals.values = r - mean(r);

% Run the simulation of the open-loop
sim('assign1ol.mdl');

data_msine = iddata(y.signals.values, u.signals.values, 1/fs);
data_msine = detrend(data_msine);

figure(3);
freq_msine = etfe(data_msine);
bodeplot(freq_msine);
grid on;
title('Frequency Response Estimate (Multi-sine)');

% Interpretation
% Figure 2:
% At low frequencies, the system attenuates input signals and shows a phase
% shift around -90 degrees, which suggests it behaves like an integrator.
% Something you'd expect from a mass-spring-damper system where force is
% integrated into velocity and then position. In the mid-frequency range,
% there are two bumps in the magnitude plot that line up with quick drops
% in the phase, pointing to resonance happening at those frequencies. This
% fits with what you would expect from a fourth-order system that has two
% natural frequencies. At high frequencies, the system starts to block
% signals more strongly, and the magnitude also drops fast. So, it is
% acting like a low-pass filter. The phase is messy in this range.
% Probably, because the signal is so weak it gets lost in the noise. Which
% makes the phase info unreliable. 

%% Assignment 1.b: 
na = 4;
nb = 4;
nc = 4;
nd = 4;
nf = 4; 
nk = 1; % Input delay

% OE model
model_oe = oe(data_prbs, [nb nf nk]);

% ARX model
model_arx = arx(data_prbs, [na nb nk]);

% ARMAX model
model_armax = armax(data_prbs, [na nb nc nk]);

% Box-Jenkins (BJ) model
model_bj = bj(data_prbs, [nb nc nd nf nk]);

% Frequency domain
figure(4);
bodeplot(freq_prbs, model_oe, model_arx, model_armax, model_bj);
grid on;
legend('Non-parametric (spa)', 'OE(4,4,1)', 'ARX(4,4,1)', 'ARMAX(4,4,1)', 'BJ(4,4,1)');
title('Frequency response: parametric vs. non-parametric');

% Time domain validation
figure(5);
compare(data_prbs, model_oe, model_arx, model_armax, model_bj);
title('Time domain: measured output vs. simulated model outputs');

% OE plots
% Comparison in frequency domain OE
figure(6);
bodeplot(freq_prbs, model_oe); 
grid on;
legend('Non-parametric (spa)', 'Parametric (oe441)');
title('Frequency response: parametric vs. non-parametric');

% Comparison in time domain 
figure(7);
compare(data_prbs, model_oe);
title('Time domain: measured output vs. simulated model output');


% Pole-zero plot OE
figure(8);
pzmap(model_oe);
grid on;
title('Pole-zero plot for oe441 model');

[p, z] = pzmap(model_oe);
fprintf('Poles of the OE model:\n');
disp(p)

fprintf('Zeros of the OE model:\n');
disp(z)


% Based on the poles and zeros, it seems that the fourth-order model is a
% good fit. There is no overlap between the poles and zeros. When they are
% too close together, they can cancel each other out, which often means the
% model is more complex than it needs to be. This is not happening here. So
% the model is appropriate. The model is also stable because all the poles
% lie within the unit circle. The magnitudes of the poles are slightly
% below one, which means the system behaves well over time and does not
% converge. 

%% Assignment 1.c:
ts = 0.01;
Cz_num = [61.4, 3.554, -57.85];
Cz_den = [1, -1.082, 0.2925];

ref.time = [];
ref.signals.values = idinput(N, 'prbs');

% run simulation
sim('assign1cl.mdl');

data_cl = iddata(y.signals.values, u.signals.values, ts);
data_cl = detrend(data_cl); % Remove DC offset

% direct identification method:
model_oe_cl = oe(data_cl, [4 4 1]);

figure(9);
bodeplot(model_oe, model_oe_cl);
grid on;
legend('Open-loop estimate (model OE)', 'Closed-loop estimate (model OE_{cl})');
title('Frequency response: open-loop vs. direct closed-loop identification');

% Also, compare the pole-zero maps
figure(10);
pzmap(model_oe, model_oe_cl);
grid on;
legend('Open-loop estimate', 'Closed-loop estimate');
title('Pole-zero map: Open-loop vs. Closed-loop identification');


% It can be seen that when using the direct identification method on data
% collected using the direct identification method on data collected under 
% closed-loop control, the resulting model turns out to be inaccurate and 
% biased because the controller creates a hidden link between the system 
% input and the measurement noise

%% Assignment 1.d:
data_T = iddata(y.signals.values, r.signals.values, ts);
data_T = detrend(data_T);
model_T = oe(data_T, [8 8 1]);

Cz = tf(Cz_num, Cz_den, ts);
model_oe_indirect = model_T /(Cz * (1 - model_T));
model_oe_indirect = minreal(model_oe_indirect); 

figure(11);
bodeplot(model_oe, model_oe_cl, model_oe_indirect);
grid on;
legend('Open-loop estimate', 'Direct closed-loop estimate', 'Indirect closed-loop estimate');
title('Frequency response: comparison of all identification methods');

%% Assignment 1.e:
model_etfe = etfe(data_msine);
[G_measured, w] = frdata(model_etfe, 'v');
s = 1i * w;
G_corrected = G_measured .* exp(s*ts/2);

num_iterations = 15;
theta = zeros(5, 1);

model_oe_c = d2c(model_oe, 'zoh');
[~, den_c] = tfdata(model_oe_c, 'v');

a1_init = den_c(2); 
a2_init = den_c(3); 
a3_init = den_c(4); 
a4_init = den_c(5);
A_est = s.^4 + a1_init*s.^3 + a2_init*s.^2 + a3_init*s + a4_init;

for iter = 1:num_iterations
    W = 1 ./ abs(A_est);
    
    Y = (s.^4) .* G_corrected;
    Phi = [-(s.^3) .* G_corrected, -(s.^2) .* G_corrected, -s .* G_corrected, -G_corrected, ones(size(s))];
       
    Y_w = Y .* W;
    Phi_w = Phi .* W;
    
    Y_stack = [real(Y_w); imag(Y_w)];
    Phi_stack = [real(Phi_w); imag(Phi_w)];
    
    theta = Phi_stack\Y_stack;
    
    a1 = theta(1); a2 = theta(2); a3 = theta(3); a4 = theta(4);
    A_est = s.^4 + a1*s.^3 + a2*s.^2 + a3*s + a4;
end

b1 = theta(5);
model_freq_improved = tf(b1, [1, a1, a2, a3, a4]);

figure(12);
bodeplot(model_oe, model_freq_improved);
grid on;
legend('Time-domain OE Model (from 1b)', 'Frequency-domain estimate');
title('Validation of frequency-domain identification');