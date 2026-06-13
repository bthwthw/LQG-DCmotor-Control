load("motor.mat");
[num, den] = tfdata(motor, 'v');

sys_c = tf(num, den); 
sys_ss_c = ss(sys_c);
[A, B, C, D] = ssdata(sys_ss_c);
disp('Ma tran A:'); disp(A);
disp('Ma tran B:'); disp(B);
disp('Ma tran C:'); disp(C);
disp('Ma tran D:'); disp(D);

temp = B;
B = C;
C = temp;
disp('Ma tran B sau khi doi lai:'); disp(B);
disp('Ma tran C sau khi doi lai:'); disp(C);

sys_ss_c = ss(A, B, C, D);

T = readtable('thong_so2.csv');

data_pwm_500_full = T.Raw_RPM(T.PWM_Output == 500);
% figure;
% plot(data_pwm_500_full); 
data_tinh = data_pwm_500_full(590:end); 
vk = var(data_tinh);

%% roi rac hoa 
Ts = 0.01;
sys_ss_d = c2d(sys_ss_c, Ts, 'zoh');

[Ad, Bd, Cd, Dd] = ssdata(sys_ss_d);

disp('Ma tran A roi rac:'); disp(Ad);
disp('Ma tran B roi rac:'); disp(Bd);
disp('Ma tran C roi rac:'); disp(Cd);
disp('Ma tran D roi rac:'); disp(Dd);

%% LQR
Q = 100;
R = 1;

K_d = dlqr(Ad, Bd, Q, R);

%% kalman 
wk = 0.2 * vk;
G = 1;
L_d = dlqe(Ad, G, Cd, wk, vk);

%% precompensation 
N = 1 / (Cd * inv(1 - (Ad - Bd * K_d)) * Bd);

disp('Do loi LQR: '); disp(K_d);
disp('Do loi kalman: '); disp(L_d);
disp('Do loi tien bu: '); disp(N);