close all;

N_steps = 100; % Mô phỏng trong 100 mẫu (1 giây)

Q_list = [1, 10, 100, 1000];
R = 1;
colors = {'b', 'g', 'r', 'm'};

figure('Name', 'Thuc Nghiem Tune LQR', 'Color', 'w');

for i = 1:length(Q_list)
    Q_test = Q_list(i);
    
    K_d = dlqr(Ad, Bd, Q_test, R);
    
    % 2. Tính hệ số bù N_bar để triệt tiêu sai số tĩnh (Setpoint Tracking)
    % Vì LQR gốc chỉ kéo về 0, nên cần N_bar để kéo lên Setpoint
    N_bar = 1 / (Cd * inv(1 - (Ad - Bd * K_d)) * Bd);
    
    % 3. Mô phỏng đáp ứng với Setpoint = 500 RPM
    setpoint = 200;
    x = 0; % Vận tốc ban đầu
    
    Y_history = zeros(1, N_steps);
    U_history = zeros(1, N_steps);
    
    for k = 1:N_steps
        % Tính tín hiệu điều khiển u(k)
        u = -K_d * x + N_bar * setpoint;
        if u > 999
            u = 999;
        elseif u < 0   % Giới hạn dưới nếu động cơ chỉ quay 1 chiều từ 0-360 độ
            u = 0;
        end
        % Cập nhật trạng thái động cơ x(k+1)
        x = Ad * x + Bd * u;
        y = Cd * x;
        
        Y_history(k) = y;
        U_history(k) = u;
    end
    
    % 4. Vẽ đồ thị Vận tốc (Đáp ứng hệ thống)
    subplot(2,1,1); hold on; grid on;
    plot(1:N_steps, Y_history, 'Color', colors{i}, 'LineWidth', 2, ...
        'DisplayName', ['Q = ', num2str(Q_test), ', R = 1']);
    
    % 5. Vẽ đồ thị PWM (Năng lượng tiêu tốn)
    subplot(2,1,2); hold on; grid on;
    plot(1:N_steps, U_history, 'Color', colors{i}, 'LineWidth', 1.5, ...
        'DisplayName', ['Q = ', num2str(Q_test), ', R = 1']);
end

% Trang trí đồ thị 1
subplot(2,1,1);
yline(setpoint, 'k--', 'Setpoint 200 RPM', 'LineWidth', 1.5);
title('Đáp ứng vận tốc động cơ với các mức Q khác nhau');
ylabel('Vận tốc (RPM)');
legend('Location', 'southeast');

% Trang trí đồ thị 2
subplot(2,1,2);
title('Tín hiệu điều khiển PWM');
xlabel('Mẫu thời gian k');
ylabel('PWM Output');
legend('Location', 'northeast');