% Dump taroz estimate_pos_vel_ambiguity_PDC internals for LibGNSS++ parity checks.
%
% Environment overrides:
%   GNSSPP_TAROZ_ROOT                         Root of taroz/PPC-Dataset checkout.
%   GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR Directory containing estimate_pos_vel_ambiguity_PDC.m.
%   GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR    Data directory for rover/base/nav/reference/seed files.
%   GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR     Output directory for CSV dumps.
%   GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS Skip leading fixed-interval epochs before solving.
%   GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS  Limit solved fixed-interval epochs; 0 means all.

taroz_root = getenv("GNSSPP_TAROZ_ROOT");
if strlength(taroz_root) == 0
    taroz_root = "/tmp/taroz_gtsam_gnss";
end

example_dir = getenv("GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR");
if strlength(example_dir) == 0
    example_dir = fullfile(taroz_root, "examples");
end

data_dir = getenv("GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR");
if strlength(data_dir) == 0
    data_dir = fullfile(example_dir, "data");
end

out_dir = getenv("GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR");
if strlength(out_dir) == 0
    out_dir = fullfile(pwd, "output", "dogfood", "taroz_matlab_pos_vel_amb_pdc_debug");
end
if ~isfolder(out_dir)
    mkdir(out_dir);
end

dump_skip_epochs = read_nonnegative_integer_env("GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS", 0);
dump_max_epochs = read_nonnegative_integer_env("GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS", 0);
configure_matlab_paths(taroz_root, example_dir);

set(0, "DefaultFigureVisible", "off");
cd(example_dir);
default_data_dir = string(fullfile(example_dir, "data"));
if dump_skip_epochs > 0 || dump_max_epochs > 0 || string(data_dir) ~= default_data_dir
    configured_script = write_configured_estimator_script(example_dir, data_dir, out_dir, ...
                                                          dump_skip_epochs, dump_max_epochs);
    run(configured_script);
else
    estimate_pos_vel_ambiguity_PDC;
end
close all force;

out_dir = getenv("GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR");
if strlength(out_dir) == 0
    out_dir = fullfile(pwd, "output", "dogfood", "taroz_matlab_pos_vel_amb_pdc_debug");
end
if ~isfolder(out_dir)
    mkdir(out_dir);
end
dump_skip_epochs = read_nonnegative_integer_env("GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS", 0);
dump_max_epochs = read_nonnegative_integer_env("GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS", 0);

v_est = nan(n, 3);
for i = 1:n
    v_est(i,:) = results.atVector(sym('v',i))';
end

graph_factors = scalar_or_nan(@() graph.size);
graph_values = scalar_or_nan(@() initials.size);
initial_cost = scalar_or_nan(@() graph.error(initials));
final_cost = scalar_or_nan(@() graph.error(results));
optimizer_error = scalar_or_nan(@() optimizer.error);
iterations = scalar_or_nan(@() optimizer.iterations);
valid_position_epochs = nnz(all(isfinite(x_est), 2));
valid_velocity_epochs = nnz(all(isfinite(v_est), 2));
valid_ambiguity_states = nnz(b_est ~= 0 & isfinite(b_est));
fixed_epochs = nnz(idxfix);

graph_table = table(n, nsat, dump_skip_epochs, dump_max_epochs, ...
                    graph_factors, graph_values, initial_cost, ...
                    final_cost, optimizer_error, iterations, ...
                    valid_position_epochs, valid_velocity_epochs, ...
                    valid_ambiguity_states, fixed_epochs);
writetable(graph_table, fullfile(out_dir, "graph_detail.csv"));
write_optimizer_cost_trace(out_dir, graph, initials, initial_cost, final_cost, iterations);

epoch = (1:n)';
gps_week = obs.time.week(:);
gps_tow = obs.time.tow(:);
spp_x_m = x_ini(:,1);
spp_y_m = x_ini(:,2);
spp_z_m = x_ini(:,3);
float_x_m = x_est(:,1);
float_y_m = x_est(:,2);
float_z_m = x_est(:,3);
fixed_x_m = xa_est(:,1);
fixed_y_m = xa_est(:,2);
fixed_z_m = xa_est(:,3);
fgo_vx_mps = v_est(:,1);
fgo_vy_mps = v_est(:,2);
fgo_vz_mps = v_est(:,3);
ratio_col = ratio(:);
idxfix_col = idxfix(:);
candidate_count = zeros(n, 1);
fixed_ambiguity_count = zeros(n, 1);
for i = 1:n
    candidate_count(i) = nnz(b_est(i,:) ~= 0 & isfinite(b_est(i,:)));
    fixed_ambiguity_count(i) = nnz(isfinite(ba_est(i,:)));
end

epoch_table = table(epoch, gps_week, gps_tow, spp_x_m, spp_y_m, spp_z_m, ...
                    float_x_m, float_y_m, float_z_m, ...
                    fixed_x_m, fixed_y_m, fixed_z_m, ...
                    fgo_vx_mps, fgo_vy_mps, fgo_vz_mps, ...
                    ratio_col, idxfix_col, candidate_count, fixed_ambiguity_count, ...
                    'VariableNames', {'epoch','gps_week','gps_tow', ...
                                      'spp_x_m','spp_y_m','spp_z_m', ...
                                      'float_x_m','float_y_m','float_z_m', ...
                                      'fixed_x_m','fixed_y_m','fixed_z_m', ...
                                      'fgo_vx_mps','fgo_vy_mps','fgo_vz_mps', ...
                                      'ratio','idxfix','candidate_count', ...
                                      'fixed_ambiguity_count'});
writetable(epoch_table, fullfile(out_dir, "per_epoch_state.csv"));

satstr = string(obs.satstr);
rows = n * nsat;
epoch_col = zeros(rows, 1);
gps_week_col = zeros(rows, 1);
gps_tow_col = zeros(rows, 1);
satellite = strings(rows, 1);
reference = strings(rows, 1);
system = strings(rows, 1);
snr_dbhz = nan(rows, 1);
elevation_deg = nan(rows, 1);
reference_elevation_deg = nan(rows, 1);
sigma_pdd_m = nan(rows, 1);
sigma_ldd_m = nan(rows, 1);
sigma_dsd_mps = nan(rows, 1);
res_pdd = nan(rows, 1);
res_ldd = nan(rows, 1);
res_dsd_mps = nan(rows, 1);
res_lsd_m = nan(rows, 1);
tdcp_sd_m = nan(rows, 1);
raw_p_rover_m = nan(rows, 1);
raw_l_rover_cycles = nan(rows, 1);
raw_d_rover_hz = nan(rows, 1);
wavelength_m = nan(rows, 1);
ambiguity_initial = nan(rows, 1);
ambiguity_estimate = nan(rows, 1);
ambiguity_fixed = nan(rows, 1);
valid_pseudorange_dd = false(rows, 1);
valid_carrier_dd = false(rows, 1);
valid_doppler_sd = false(rows, 1);
valid_tdcp_sd = false(rows, 1);
los_x = nan(rows, 1);
los_y = nan(rows, 1);
los_z = nan(rows, 1);

k = 0;
for i = 1:n
    for j = 1:nsat
        ref_idx = reference_index_for_observation(refsatidx, i, j);
        k = k + 1;
        epoch_col(k) = i;
        gps_week_col(k) = obs.time.week(i);
        gps_tow_col(k) = obs.time.tow(i);
        satellite(k) = satstr(j);
        if isfinite(ref_idx) && ref_idx >= 1 && ref_idx <= nsat
            reference(k) = satstr(ref_idx);
            reference_elevation_deg(k) = sat.el(i,ref_idx);
        end
        system(k) = string(obs.sys(j));
        snr_dbhz(k) = obs.L1.S(i,j);
        elevation_deg(k) = sat.el(i,j);
        sigma_pdd_m(k) = sigmaP(i,j);
        sigma_ldd_m(k) = sigmaL(i,j);
        sigma_dsd_mps(k) = sigmaD(i,j);
        res_pdd(k) = obs.L1.resPdd(i,j);
        res_ldd(k) = obs.L1.resLdd(i,j);
        res_dsd_mps(k) = resDsd(i,j);
        res_lsd_m(k) = resLsd(i,j);
        raw_p_rover_m(k) = obs.L1.P(i,j);
        raw_l_rover_cycles(k) = obs.L1.L(i,j);
        raw_d_rover_hz(k) = obs.L1.D(i,j);
        wavelength_m(k) = obs.L1.lam(j);
        ambiguity_initial(k) = b_ini(i,j);
        ambiguity_estimate(k) = b_est(i,j);
        ambiguity_fixed(k) = ba_est(i,j);
        valid_pseudorange_dd(k) = ~isnan(obs.L1.resPdd(i,j));
        valid_carrier_dd(k) = ~isnan(obs.L1.resLdd(i,j));
        valid_doppler_sd(k) = ~isnan(resDsd(i,j));
        los_x(k) = -exdd(i,j);
        los_y(k) = -eydd(i,j);
        los_z(k) = -ezdd(i,j);
        if i > 1
            tdcp_sd_m(k) = resLsd(i,j) - resLsd(i-1,j);
            valid_tdcp_sd(k) = ~isnan(tdcp_sd_m(k)) && tdcp_sd_m(k) ~= 0;
        end
    end
end

sat_table = table(epoch_col, gps_week_col, gps_tow_col, satellite, reference, ...
                  system, snr_dbhz, elevation_deg, reference_elevation_deg, ...
                  sigma_pdd_m, sigma_ldd_m, sigma_dsd_mps, ...
                  res_pdd, res_ldd, res_dsd_mps, res_lsd_m, tdcp_sd_m, ...
                  raw_p_rover_m, raw_l_rover_cycles, raw_d_rover_hz, ...
                  wavelength_m, ambiguity_initial, ambiguity_estimate, ...
                  ambiguity_fixed, valid_pseudorange_dd, valid_carrier_dd, ...
                  valid_doppler_sd, valid_tdcp_sd, los_x, los_y, los_z, ...
                  'VariableNames', {'epoch','gps_week','gps_tow', ...
                                    'satellite','reference','system', ...
                                    'snr_dbhz','elevation_deg', ...
                                    'reference_elevation_deg', ...
                                    'sigma_pdd_m','sigma_ldd_m', ...
                                    'sigma_dsd_mps','res_pdd','res_ldd', ...
                                    'res_dsd_mps','res_lsd_m','tdcp_sd_m', ...
                                    'raw_p_rover_m','raw_l_rover_cycles', ...
                                    'raw_d_rover_hz','wavelength_m', ...
                                    'ambiguity_initial','ambiguity_estimate', ...
                                    'ambiguity_fixed','valid_pseudorange_dd', ...
                                    'valid_carrier_dd','valid_doppler_sd', ...
                                    'valid_tdcp_sd','los_x','los_y','los_z'});
writetable(sat_table, fullfile(out_dir, "per_sat_detail.csv"));

lambda_rows = 0;
for i = 1:n
    valid_idx = find(b_est(i,:) ~= 0 & isfinite(b_est(i,:)));
    candidate_n = length(valid_idx);
    if candidate_n > minobs_th
        lambda_rows = lambda_rows + candidate_n * candidate_n;
    end
end

lambda_epoch = zeros(lambda_rows, 1);
lambda_gps_week = zeros(lambda_rows, 1);
lambda_gps_tow = zeros(lambda_rows, 1);
solved = false(lambda_rows, 1);
fixed_epoch = false(lambda_rows, 1);
ratio_dump = nan(lambda_rows, 1);
candidate_count_dump = zeros(lambda_rows, 1);
row_dump = zeros(lambda_rows, 1);
col_dump = zeros(lambda_rows, 1);
local_index = zeros(lambda_rows, 1);
other_local_index = zeros(lambda_rows, 1);
satellite_dump = strings(lambda_rows, 1);
other_satellite = strings(lambda_rows, 1);
ambiguity_float = nan(lambda_rows, 1);
fixed_ambiguity = nan(lambda_rows, 1);
covariance = nan(lambda_rows, 1);
position_covariance_x = nan(lambda_rows, 1);
position_covariance_y = nan(lambda_rows, 1);
position_covariance_z = nan(lambda_rows, 1);

k = 0;
for i = 1:n
    valid_idx = find(b_est(i,:) ~= 0 & isfinite(b_est(i,:)));
    candidate_n = length(valid_idx);
    if candidate_n <= minobs_th
        continue;
    end
    Qfafa = covxb_est(valid_idx, valid_idx, i);
    Qxfa = covxb_est(end-2:end, valid_idx, i);
    lambda_solved = false;
    lambda_fixed = false;
    lambda_ratio = NaN;
    a = nan(2, candidate_n);
    try
        [a,s] = rtklib.lambda(2, b_est(i,valid_idx), Qfafa);
        lambda_solved = true;
        lambda_ratio = s(2) / s(1);
        lambda_fixed = lambda_ratio > ratio_th;
    catch
        lambda_solved = false;
    end
    for r = 1:candidate_n
        for c = 1:candidate_n
            k = k + 1;
            sat_idx = valid_idx(r);
            other_sat_idx = valid_idx(c);
            lambda_epoch(k) = i;
            lambda_gps_week(k) = obs.time.week(i);
            lambda_gps_tow(k) = obs.time.tow(i);
            solved(k) = lambda_solved;
            fixed_epoch(k) = lambda_fixed;
            ratio_dump(k) = lambda_ratio;
            candidate_count_dump(k) = candidate_n;
            row_dump(k) = r - 1;
            col_dump(k) = c - 1;
            local_index(k) = sat_idx - 1;
            other_local_index(k) = other_sat_idx - 1;
            satellite_dump(k) = satstr(sat_idx);
            other_satellite(k) = satstr(other_sat_idx);
            ambiguity_float(k) = b_est(i,sat_idx);
            if lambda_solved
                fixed_ambiguity(k) = a(1,r);
            end
            covariance(k) = Qfafa(r,c);
            position_covariance_x(k) = Qxfa(1,r);
            position_covariance_y(k) = Qxfa(2,r);
            position_covariance_z(k) = Qxfa(3,r);
        end
    end
end

lambda_table = table(lambda_epoch, lambda_gps_week, lambda_gps_tow, ...
                     solved, fixed_epoch, ratio_dump, candidate_count_dump, ...
                     row_dump, col_dump, local_index, other_local_index, ...
                     satellite_dump, other_satellite, ambiguity_float, ...
                     fixed_ambiguity, covariance, position_covariance_x, ...
                     position_covariance_y, position_covariance_z, ...
                     'VariableNames', {'epoch','gps_week','gps_tow', ...
                                       'solved','fixed_epoch','ratio', ...
                                       'candidate_count','row','col', ...
                                       'local_index','other_local_index', ...
                                       'satellite','other_satellite', ...
                                       'ambiguity_float','fixed_ambiguity', ...
                                       'covariance','position_covariance_x', ...
                                       'position_covariance_y', ...
                                       'position_covariance_z'});
writetable(lambda_table, fullfile(out_dir, "per_lambda_detail.csv"));

fprintf("Wrote taroz position/velocity ambiguity PDC debug CSVs to %s\n", out_dir);

function value = read_nonnegative_integer_env(name, default_value)
    raw = getenv(name);
    if strlength(raw) == 0
        value = default_value;
        return;
    end
    value = str2double(raw);
    if ~isfinite(value) || value < 0 || round(value) ~= value
        error("%s must be a non-negative integer", name);
    end
end

function configured_script = write_configured_estimator_script(example_dir, data_dir, out_dir, skip_epochs, max_epochs)
    source_script = fullfile(example_dir, "estimate_pos_vel_ambiguity_PDC.m");
    source_text = fileread(source_script);
    function_dir_text = matlab_single_quoted_path(fullfile(example_dir, "function"));
    data_dir_text = matlab_single_quoted_path(string(data_dir) + filesep);
    source_text = replace(source_text, "addpath ./function", ...
                          "addpath('" + function_dir_text + "')");
    source_text = replace(source_text, 'datapath = "./data/";', ...
                          "datapath = '" + data_dir_text + "';");
    source_text = replace(source_text, ...
                          "sol_spp = sol_spp.fixedInterval(sol_spp.dt);", ...
                          "sol_spp = sol_spp.fixedInterval(sol_spp.dt);" + newline + ...
                          "sol_spp = sol_spp.sameTime(obs.time);");
    marker = "[obs,obsb] = obs.commonSat(obsb);";
    window_lines = [
        marker
        "gnsspp_window_skip_epochs = " + string(skip_epochs) + ";"
        "gnsspp_window_max_epochs = " + string(max_epochs) + ";"
        "gnsspp_window_start = min(obs.n + 1, gnsspp_window_skip_epochs + 1);"
        "gnsspp_window_stop = obs.n;"
        "if gnsspp_window_max_epochs > 0"
        "    gnsspp_window_stop = min(obs.n, gnsspp_window_start + gnsspp_window_max_epochs - 1);"
        "end"
        "gnsspp_window_idx = gnsspp_window_start:gnsspp_window_stop;"
        "if isempty(gnsspp_window_idx)"
        "    error('GNSSPP taroz ambiguity PDC window selected no epochs');"
        "end"
        "if gnsspp_window_skip_epochs > 0 && (obsb.n ~= obs.n || any(obsb.time.t ~= obs.time.t))"
        "    obsb = obsb.interp(obs.time);"
        "end"
        "obs = obs.selectTime(gnsspp_window_idx);"
        "obsb = obsb.selectTime(gnsspp_window_idx);"
    ];
    window_code = strjoin(window_lines, newline) + newline;

    if ~contains(source_text, marker)
        error("Unable to inject GNSSPP window slice into %s", source_script);
    end
    window_text = replace(source_text, marker, window_code);
    configured_script = fullfile(out_dir, "estimate_pos_vel_ambiguity_PDC_configured.m");
    fid = fopen(configured_script, "w");
    if fid < 0
        error("Unable to write configured taroz script: %s", configured_script);
    end
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, "%s", window_text);
    clear cleaner;
end

function value = matlab_single_quoted_path(path_value)
    value = replace(string(path_value), "'", "''");
end

function configure_matlab_paths(taroz_root, example_dir)
    add_matlab_path_if_folder(example_dir);
    add_matlab_path_if_folder(fullfile(example_dir, "function"));
    add_matlab_path_if_folder(fullfile(taroz_root, "examples"));
    add_matlab_path_if_folder(fullfile(taroz_root, "examples", "function"));
    add_matlab_path_if_folder(fullfile(taroz_root, "install", "gtsam_gnss", "gtsam_toolbox"));
    add_matlab_path_if_folder(fullfile(taroz_root, "matlab_local", "gtsam_toolbox"));
    add_matlab_path_if_folder(fullfile(taroz_root, "..", "matlab_local", "gtsam_toolbox"));
    add_matlab_path_if_folder(fullfile(taroz_root, "..", "MatRTKLIB"));
    add_matlab_path_if_folder("/usr/local/gtsam_toolbox");
    add_matlab_path_list(getenv("GNSSPP_TAROZ_MATLAB_PATHS"));
    add_matlab_path_list(getenv("GNSSPP_TAROZ_MATLAB_EXTRA_PATHS"));
end

function add_matlab_path_if_folder(path_value)
    path_value = string(path_value);
    if strlength(path_value) > 0 && isfolder(path_value)
        addpath(char(path_value));
    end
end

function add_matlab_path_list(path_list)
    path_list = string(path_list);
    if strlength(path_list) == 0
        return;
    end
    paths = split(path_list, pathsep);
    for i = 1:numel(paths)
        add_matlab_path_if_folder(strtrim(paths(i)));
    end
end

function write_optimizer_cost_trace(out_dir, graph, initials, initial_cost, final_cost, iterations)
    max_iterations = 0;
    if isfinite(iterations)
        max_iterations = max(0, round(iterations));
    end

    phase = repmat("gtsam", max_iterations + 1, 1);
    local_iteration = (0:max_iterations)';
    global_iteration = local_iteration;
    time_s = NaN(max_iterations + 1, 1);
    cost = NaN(max_iterations + 1, 1);
    absolute_decrease = NaN(max_iterations + 1, 1);
    relative_decrease = NaN(max_iterations + 1, 1);
    lambda = NaN(max_iterations + 1, 1);
    actual_rows = 0;

    try
        trace_params = gtsam.LevenbergMarquardtParams;
        trace_params.setVerbosity('TERMINATION');
        trace_params.setMaxIterations(1000);
        trace_optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initials, trace_params);

        trace_timer = tic;
        actual_rows = 1;
        time_s(1) = 0.0;
        cost(1) = trace_optimizer.error;
        lambda(1) = trace_optimizer.lambda;
        for iter = 1:max_iterations
            trace_optimizer.iterate();
            actual_rows = iter + 1;
            time_s(actual_rows) = toc(trace_timer);
            cost(actual_rows) = trace_optimizer.error;
            lambda(actual_rows) = trace_optimizer.lambda;
        end
    catch trace_error
        warning("Failed to generate GTSAM optimizer cost trace: %s", trace_error.message);
    end

    if actual_rows == 0
        phase = ["gtsam"; "gtsam"];
        local_iteration = [0; max_iterations];
        global_iteration = local_iteration;
        time_s = [0.0; NaN];
        cost = [initial_cost; final_cost];
        absolute_decrease = [NaN; initial_cost - final_cost];
        relative_decrease = [NaN; (initial_cost - final_cost) / max(1.0, initial_cost)];
        lambda = [NaN; NaN];
    else
        phase = phase(1:actual_rows);
        local_iteration = local_iteration(1:actual_rows);
        global_iteration = global_iteration(1:actual_rows);
        time_s = time_s(1:actual_rows);
        cost = cost(1:actual_rows);
        lambda = lambda(1:actual_rows);
        absolute_decrease = absolute_decrease(1:actual_rows);
        relative_decrease = relative_decrease(1:actual_rows);
        for row = 2:actual_rows
            if isfinite(cost(row - 1)) && isfinite(cost(row))
                absolute_decrease(row) = cost(row - 1) - cost(row);
                if cost(row - 1) > 0
                    relative_decrease(row) = absolute_decrease(row) / cost(row - 1);
                else
                    relative_decrease(row) = absolute_decrease(row);
                end
            end
        end
    end

    cost_trace_table = table(phase, local_iteration, global_iteration, time_s, ...
                             cost, absolute_decrease, relative_decrease, lambda);
    writetable(cost_trace_table, fullfile(out_dir, "optimizer_cost_trace.csv"));
end

function value = scalar_or_nan(callback)
    try
        value = callback();
    catch
        value = NaN;
    end
end

function ref_idx = reference_index_for_observation(refsatidx, epoch_index, satellite_index)
    if isscalar(refsatidx)
        ref_idx = refsatidx;
    elseif isvector(refsatidx)
        if numel(refsatidx) >= satellite_index
            ref_values = refsatidx(:);
            ref_idx = ref_values(satellite_index);
        elseif numel(refsatidx) >= epoch_index
            ref_values = refsatidx(:);
            ref_idx = ref_values(epoch_index);
        else
            ref_idx = NaN;
        end
    elseif size(refsatidx, 1) >= epoch_index && size(refsatidx, 2) >= satellite_index
        ref_idx = refsatidx(epoch_index, satellite_index);
    elseif size(refsatidx, 1) >= epoch_index
        ref_idx = refsatidx(epoch_index, 1);
    elseif size(refsatidx, 2) >= satellite_index
        ref_idx = refsatidx(1, satellite_index);
    else
        ref_idx = NaN;
    end
end
