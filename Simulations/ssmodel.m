clc
clear
close all

g = 981;
m_c = 710;
l_c = 6.28;
Io_c = 37200;
I_w = 2740;
C_f = 10000;
C_w = 3000;
%K_t = 700000;
K_t = 500000;
dt = 0.01;

A = [0              1           0;
    m_c*l_c*g/Io_c, -C_f/Io_c,  C_w/Io_c;
    -m_c*l_c*g/Io_c, C_f/Io_c,  -C_w/I_w - C_w/Io_c];
B = [0;
    -K_t/Io_c;
    K_t/I_w - K_t/Io_c];

% A = [0              1           0;
%     m_c*l_c*g/Io_c, -C_f/Io_c,  C_w/Io_c;
%     0,              0,      -C_w/I_w];
% B = [0;
%     -K_t/Io_c;
%     K_t/I_w];

C = eye(3);
D = 0;

Co = ctrb(A,B);

[v,e] = eig(Co);

sys = ss(A,B,C,D);
d_sys = c2d(sys, dt);

A_d = d_sys.A;
B_d = d_sys.B;
C_d = d_sys.C;
D_d = d_sys.D;

