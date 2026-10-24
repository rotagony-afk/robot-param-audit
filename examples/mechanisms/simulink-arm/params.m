% params.m - 由 rsw-audit 生成，模型里用到的所有数值都在这里
% 建议把这里改成结构体或 .mat，方便批量扫描

dof = 6;
q1_min = -2.96706;
q1_max = 2.96706;
tau1_max = 60;
q2_min = -1.919862;
q2_max = 1.919862;
tau2_max = 60;
q3_min = -2.617994;
q3_max = 2.617994;
tau3_max = 22;
q4_min = -3.141593;
q4_max = 3.141593;
tau4_max = 6;
q5_min = -2.094395;
q5_max = 2.094395;
tau5_max = 1.5;
q6_min = -3.141593;
q6_max = 3.141593;
tau6_max = 2;
% 标称关节惯量（请用 CAD 或厂家参数替换）
J_nominal = 0.05;   % [kg*m^2]
