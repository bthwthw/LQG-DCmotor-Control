SP = 200;
sim('LQG.slx');

t = squeeze(ans.tout); 
y = squeeze(ans.yk_hat.Data); 
u = squeeze(ans.uk.Data); 
sp = squeeze(ans.SP.Data); 

y_max = max(y);
if y_max > SP
    Overshoot = ((y_max - SP) / SP) * 100;
else
    Overshoot = 0;
end

% --- Tiêu chuẩn 5% ---
upper_5 = SP * 1.05;
lower_5 = SP * 0.95;

out_of_bounds_5 = find(y > upper_5 | y < lower_5);

if isempty(out_of_bounds_5)
    ts_5 = 0;
else
    last_idx_5 = out_of_bounds_5(end); 
    if last_idx_5 < length(t)
        ts_5 = t(last_idx_5 + 1); 
    else
        ts_5 = NaN;
    end
end

% --- Tiêu chuẩn 2% ---
upper_2 = SP * 1.02;
lower_2 = SP * 0.98;

out_of_bounds_2 = find(y > upper_2 | y < lower_2);

if isempty(out_of_bounds_2)
    ts_2 = 0; 
else
    last_idx_2 = out_of_bounds_2(end);
    if last_idx_2 < length(t)
        ts_2 = t(last_idx_2 + 1);
    else
        ts_2 = NaN;
    end
end

fprintf('Setpoint             : %d RPM\n', SP);
fprintf('Độ vọt lố (Overshoot): %.2f %%\n', Overshoot);
fprintf('Thời gian xác lập 5%% : %.4f giây\n', ts_5);
fprintf('Thời gian xác lập 2%% : %.4f giây\n', ts_2);


subplot(2,1,1);
plot(t, y, 'b', 'LineWidth', 1.5); hold on; grid on;
plot(t, sp, 'r--', 'LineWidth', 1.5);
title('Đáp ứng vận tốc động cơ đã qua bộ lọc Kalman');
ylabel('Vận tốc (RPM)');
legend('Vận tốc yk', 'Tốc độ đặt (RPM)', 'Location', 'southeast');
xlim([0 t(end)]);

subplot(2,1,2);
plot(t, u, 'm', 'LineWidth', 1.5); grid on;
title('Tín hiệu điều khiển động cơ');
xlabel('Thời gian (giây)');
ylabel('PWM Output');
legend('Tín hiệu u_k', 'Location', 'northeast');
xlim([0 t(end)]);
ylim([-50 1050]); 
