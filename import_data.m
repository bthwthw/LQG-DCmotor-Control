data = readtable('thong_so.csv', 'VariableNamingRule', 'preserve');

t = 0.01;
u = data.PWM_Output;  
y = data.Actual_RPM;


disp('Done');