clear;
% 1. Physical Properties
mass = 1.0;               % kg
Ixx  = 0.041501;  Iyy = 0.033831; % Kg*m^2 (Inertia tensor components)
Izz  = 0.073763;  Ixz = 0.00176; 
S    = 0.265;              % Wing reference area (m^2)
c    = 0.211;              % Mean aerodynamic chord (m)
b    = 1.3;              % Wingspan (m)
rho  = 1.225;             % Air density at sea level (kg/m^3)

s = tf('s');
% u2&u3 and q
TF_q_u2u3 = (-2.362*s^3 -26.81*s^2 -3.892*s) / (s^4 + 40.64*s^3 + 1644*s^2 +166.4*s +608.3);
TF_q_u2u3_d = c2d(TF_q_u2u3,0.02);

%% Inner Pitch Rate
% Weights
w_pitch_rate = 4.0;

M_pitch_rate = 8000;               
A_pitch_rate = 0.001;      

W1_pitch_rate = tf(M_pitch_rate*w_pitch_rate,[1,w_pitch_rate]);
W3_pitch_rate = tf([w_pitch_rate,0],[A_pitch_rate,w_pitch_rate]);

pitch_rate_aug = augw(TF_q_u2u3,W1_pitch_rate,[],W3_pitch_rate);

[K_pitch_rate, CL, gamma] = hinfsyn(pitch_rate_aug,1,1);
K_pitch_rate_d = c2d(K_pitch_rate,0.0025);
[Ak_pitch_rate,Bk_pitch_rate,Ck_pitch_rate,Dk_pitch_rate] = ssdata(K_pitch_rate_d);

%% Pitch OL

OL_pitch = feedback(K_pitch_rate*TF_q_u2u3,1);

% Weights
w_pitch = 4.0;

M_pitch = 3500.0;               
A_pitch = 10.0;      

W1_pitch = tf(M_pitch*w_pitch,[1,w_pitch]);
W3_pitch = tf([w_pitch,0],[A_pitch,w_pitch]);

pitch_aug = augw(OL_pitch,W1_pitch,[],W3_pitch);

[K_pitch, CL, gamma] = hinfsyn(pitch_aug,1,1);
K_pitch_d = c2d(K_pitch,0.0025);
[Ak_pitch,Bk_pitch,Ck_pitch,Dk_pitch] = ssdata(K_pitch_d);