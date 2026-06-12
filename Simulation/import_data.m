data = readtable('thong_so.csv', 'VariableNamingRule', 'preserve');

t = 0.01;
u = data.PWM_Output;  
y = data.Actual_RPM;

data_pwm_500_full = T.Raw_RPM(T.PWM_Output == 500);
figure;
plot(data_pwm_500_full); 
data_tinh = data_pwm_500_full(590:end); 
vk = var(data_tinh);

disp('Done');