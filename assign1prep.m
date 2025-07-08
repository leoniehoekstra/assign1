%% Script to set up variables for assignment 8 and run some simulations.
% Set up variables.
s=tf('s');

% First compute the transfer functions Gs and Cz for the simulation.
% Note that the numerical results have been copied and pasted into the
% Simulink model, so changing any parameter in this script does not
% automagically change the Simulink models. In the case you modify Gs and
% Cz, make sure that you also update the Simulink models manually!

% Sample time
ts = 0.01; fs = 1/ts; ws = 2*pi*fs;

% "Unknown" system is a flexible system with two resonance frequencies. The
% (effictive) mass is needed to tune the controller.
m = 0.30; 

% PD+ controller
fc = 3; wc = 2*pi*fc;
alpha = 0.1;
kp = m*wc^2/sqrt(1/alpha);
tz = 1/wc*sqrt(1/alpha);
tp = 1/wc/sqrt(1/alpha);
Cs = kp * (tz*s+1) / (tp*s+1)^2;

% Discrete systems
Cz = c2d(Cs,ts,'tustin');
% For Simulink models:
[Cz_num,Cz_den] = tfdata(Cz,'v'); % PD+ controller

%% Generate input or reference:
% Multi sine with 10 components from 4:4:40 Hz. 
% Nyquist frequency is 50 Hz, then the command below does the trick (don't
% ask me why).
% Of course you may change this signal in order to excite other
% frequencies, but note that you should not increase the amplitude
% significantly.
r = idinput(2000,'sine',[4/50-1/2000 40/50+1/2000],0.1*[-1 1],[10 10 1]);
% No means please
r = r - mean(r);
ref.time=[];
ref.signals(1).values=r;
% ref.signals(1).dimensions=1;
clear r

%% OK, let's do the job! Open loop - first

sim('assign1ol');
%Check useful number of periods
nper = floor(length(u.signals(1).values)/2000)-1;
% Compute indices in arrays
indi = length(u.signals(1).values) + ((-nper*2000+1):0);

uol = u.signals(1).values(indi);
yol = y.signals(1).values(indi);

%% OK, let's do the job! Closed loop - next

sim('assign1cl');
%Check useful number of periods
nper = floor(length(r.signals(1).values)/2000)-1;
% Compute indices in arrays
indi = length(r.signals(1).values) + ((-nper*2000+1):0);

rcl = r.signals(1).values(indi);
ucl = u.signals(1).values(indi);
ycl = y.signals(1).values(indi);

%% And temporarly "switch off" the ref-input - only if data is needed

% rsave = ref.signals(1).values;
% ref.signals(1).values = 0*rsave;
% sim('assign1cl');
% rclnr = r.signals(1).values(indi);
% uclnr = u.signals(1).values(indi);
% yclnr = y.signals(1).values(indi);
% ref.signals(1).values = rsave;
% clear rsave
