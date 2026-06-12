close all;

T = readtable('thong_so3.csv');
idx = find(T.PWM_Output == 500);
Z = T.Raw_RPM(idx(1:200)); 
N = length(Z);

TR_list = [0.01, 0.2, 0.5];
colors = {'b', 'r', 'm'};

X_hat_all = zeros(length(TR_list), N);

for i = 1:length(TR_list)
    W = TR_list(i) * vk; 
    
    % khoi tao 
    x_est = Z(1); 
    P = 1.0; 
    
    for k = 1:N
        % pred
        x_pred = x_est;
        P_pred = P + W;
        
        % update 
        K_gain = P_pred / (P_pred + vk);
        x_est = x_pred + K_gain * (Z(k) - x_pred);
        P = (1 - K_gain) * P_pred;
        
        X_hat_all(i, k) = x_est;
    end
end

% subplot(3,1,1);
% figure('Name', 'Bo loc voi wk khac nhau', 'Color', 'w');
% hold on; grid on;
% plot(1:N, Z, 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5, 'DisplayName',
% 'Raw RPM');

for i = 1:length(TR_list)
    subplot(3,1,i); 
    plot(1:N, Z, 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5, 'DisplayName', 'Raw RPM');
    hold on; grid on;
    plot(1:N, X_hat_all(i, :), colors{i}, 'LineWidth', 2, ...
        'DisplayName', ['Kalman PWM (TR = ', num2str(TR_list(i)), ')']);
    ylabel('Vận tốc (RPM)');
    legend('Location', 'best');
end

xlabel('Mẫu thời gian (k)');
xlim([1 N]);

disp('  TR    |  RMSE (Độ trễ pha)  |  Chattering (Độ nhiễu)');
for i = 1:length(TR_list)
    rmse_val = sqrt(mean((Z' - X_hat_all(i,:)).^2));
    % Chattering Index (std of dao ham bac 1)
    chat_val = std(diff(X_hat_all(i,:))); 
    fprintf(' %4.2f   | %16.2f    | %18.2f\n', TR_list(i), rmse_val, chat_val);
end