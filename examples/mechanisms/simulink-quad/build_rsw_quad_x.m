function build_rsw_quad_x()
%BUILD_RSW_QUAD_X  由 rsw-audit 自动生成，请勿手工修改。
%   在 MATLAB 中运行本文件即可生成并保存 rsw_quad_x.slx，然后点 Run。
%   生成时间与参数来源见同目录 params.m 与 README.txt。

    model = 'rsw_quad_x';
    here = fileparts(mfilename('fullpath'));
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);
    set_param(model, 'StopTime', '5');

    add_block('simulink/Sources/Step', [model '/' 'rpm_cmd'], 'Time', '0', 'Before', '0', 'After', '9000', 'Position', [40 80 120 120]);
    add_block('simulink/Math Operations/Gain', [model '/' 'gain_rad'], 'Gain', '0.10471976', 'Position', [190 80 270 120]);
    add_block('simulink/Math Operations/Sum', [model '/' 'sum_lag'], 'Inputs', '+-', 'Position', [340 150 420 190]);
    add_block('simulink/Math Operations/Gain', [model '/' 'gain_tau_rotor'], 'Gain', '20', 'Position', [490 150 570 190]);
    add_block('simulink/Continuous/Integrator', [model '/' 'integ_omega'], 'InitialCondition', '0', 'Position', [640 150 720 190]);
    add_block('simulink/User-Defined Functions/Fcn', [model '/' 'thrust_map'], 'Expr', '0.000012*u(1)*u(1)', 'Position', [790 220 870 260]);
    add_block('simulink/Sinks/Scope', [model '/' 'scope_thrust'], 'Position', [940 220 1020 260]);
    add_block('simulink/Sinks/To Workspace', [model '/' 'ws_thrust'], 'VariableName', 'thrust_out', 'SaveFormat', 'Array', 'Position', [1090 220 1170 260]);
    add_block('simulink/Sinks/Scope', [model '/' 'scope_omega'], 'Position', [790 150 870 190]);
    add_block('simulink/User-Defined Functions/Fcn', [model '/' 'torque_map'], 'Expr', '0.0000002*u(1)*u(1)', 'Position', [790 290 870 330]);
    add_block('simulink/Sinks/Scope', [model '/' 'scope_torque'], 'Position', [940 290 1020 330]);

    add_line(model, 'thrust_map/1', 'ws_thrust/1', 'autorouting', 'on');
    add_line(model, 'integ_omega/1', 'torque_map/1', 'autorouting', 'on');
    add_line(model, 'torque_map/1', 'scope_torque/1', 'autorouting', 'on');
    add_line(model, 'rpm_cmd/1', 'gain_rad/1', 'autorouting', 'on');
    add_line(model, 'gain_rad/1', 'sum_lag/1', 'autorouting', 'on');
    add_line(model, 'integ_omega/1', 'sum_lag/2', 'autorouting', 'on');
    add_line(model, 'sum_lag/1', 'gain_tau_rotor/1', 'autorouting', 'on');
    add_line(model, 'gain_tau_rotor/1', 'integ_omega/1', 'autorouting', 'on');
    add_line(model, 'integ_omega/1', 'thrust_map/1', 'autorouting', 'on');
    add_line(model, 'thrust_map/1', 'scope_thrust/1', 'autorouting', 'on');
    add_line(model, 'integ_omega/1', 'scope_omega/1', 'autorouting', 'on');

    target = fullfile(here, [model '.slx']);
    save_system(model, target);
    fprintf('已生成: %s\n', target);
    fprintf('模块数: %d, 连线数: %d\n', 11, 11);
end
