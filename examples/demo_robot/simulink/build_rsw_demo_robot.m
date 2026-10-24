function build_rsw_demo_robot()
%BUILD_RSW_DEMO_ROBOT  由 rsw-audit 自动生成，请勿手工修改。
%   在 MATLAB 中运行本文件即可生成并保存 rsw_demo_robot.slx，然后点 Run。
%   生成时间与参数来源见同目录 params.m 与 README.txt。

    model = 'rsw_demo_robot';
    here = fileparts(mfilename('fullpath'));
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);
    set_param(model, 'StopTime', '5');

    add_block('simulink/Sources/Step', [model '/' 'v_cmd'], 'Time', '0', 'Before', '0', 'After', '0.3', 'Position', [40 80 120 120]);
    add_block('simulink/Math Operations/Sum', [model '/' 'sum_v'], 'Inputs', '+-', 'Position', [190 150 270 190]);
    add_block('simulink/Math Operations/Gain', [model '/' 'gain_tau'], 'Gain', '33.333333', 'Position', [340 150 420 190]);
    add_block('simulink/Continuous/Integrator', [model '/' 'integ_v'], 'InitialCondition', '0', 'Position', [490 150 570 190]);
    add_block('simulink/Continuous/Integrator', [model '/' 'integ_x'], 'InitialCondition', '0', 'Position', [640 220 720 260]);
    add_block('simulink/Math Operations/Gain', [model '/' 'gain_odom'], 'Gain', '0.98425197', 'Position', [790 220 870 260]);
    add_block('simulink/Sinks/Scope', [model '/' 'scope_v'], 'Position', [640 150 720 190]);
    add_block('simulink/Sinks/Scope', [model '/' 'scope_x'], 'Position', [940 220 1020 260]);
    add_block('simulink/Sinks/Scope', [model '/' 'scope_odom'], 'Position', [940 290 1020 330]);

    add_line(model, 'v_cmd/1', 'sum_v/1', 'autorouting', 'on');
    add_line(model, 'integ_v/1', 'sum_v/2', 'autorouting', 'on');
    add_line(model, 'sum_v/1', 'gain_tau/1', 'autorouting', 'on');
    add_line(model, 'gain_tau/1', 'integ_v/1', 'autorouting', 'on');
    add_line(model, 'integ_v/1', 'integ_x/1', 'autorouting', 'on');
    add_line(model, 'integ_v/1', 'scope_v/1', 'autorouting', 'on');
    add_line(model, 'integ_x/1', 'scope_x/1', 'autorouting', 'on');
    add_line(model, 'integ_x/1', 'gain_odom/1', 'autorouting', 'on');
    add_line(model, 'gain_odom/1', 'scope_odom/1', 'autorouting', 'on');

    target = fullfile(here, [model '.slx']);
    save_system(model, target);
    fprintf('已生成: %s\n', target);
    fprintf('模块数: %d, 连线数: %d\n', 9, 9);
end
