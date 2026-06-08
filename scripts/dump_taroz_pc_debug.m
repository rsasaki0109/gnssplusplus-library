% Dump taroz estimate_pos_ambiguity_PC internals for LibGNSS++ parity checks.
%
% Environment overrides:
%   GNSSPP_TAROZ_ROOT           Root of taroz/PPC-Dataset checkout.
%   GNSSPP_TAROZ_PC_EXAMPLE_DIR Directory containing estimate_pos_ambiguity_PC.m.
%   GNSSPP_TAROZ_PC_OUT_DIR     Output directory for CSV dumps.

taroz_root = getenv("GNSSPP_TAROZ_ROOT");
if strlength(taroz_root) == 0
    taroz_root = "/tmp/taroz_gtsam_gnss";
end

example_dir = getenv("GNSSPP_TAROZ_PC_EXAMPLE_DIR");
if strlength(example_dir) == 0
    example_dir = fullfile(taroz_root, "examples");
end

out_dir = getenv("GNSSPP_TAROZ_PC_OUT_DIR");
if strlength(out_dir) == 0
    out_dir = fullfile(pwd, "output", "dogfood", "taroz_matlab_pc_debug");
end
if ~isfolder(out_dir)
    mkdir(out_dir);
end

set(0, "DefaultFigureVisible", "off");
cd(example_dir);
estimate_pos_ambiguity_PC;
close all force;

out_dir = getenv("GNSSPP_TAROZ_PC_OUT_DIR");
if strlength(out_dir) == 0
    out_dir = fullfile(pwd, "output", "dogfood", "taroz_matlab_pc_debug");
end
if ~isfolder(out_dir)
    mkdir(out_dir);
end

graph_factors = scalar_or_nan(@() graph.size);
graph_values = scalar_or_nan(@() initials.size);
initial_cost = scalar_or_nan(@() graph.error(initials));
final_cost = scalar_or_nan(@() graph.error(results));
optimizer_error = scalar_or_nan(@() optimizer.error);
iterations = scalar_or_nan(@() optimizer.iterations);
valid_ambiguity_states = nnz(b_est ~= 0 & isfinite(b_est));
valid_solution_epochs = nnz(all(isfinite(x_est), 2));

graph_table = table(n, nsat, graph_factors, graph_values, initial_cost, ...
                    final_cost, optimizer_error, iterations, ...
                    valid_ambiguity_states, valid_solution_epochs);
writetable(graph_table, fullfile(out_dir, "graph_detail.csv"));

satstr = string(obs.satstr);

candidate_counts = zeros(n, 1);
candidate_lists = strings(n, 1);
fixed_counts = zeros(n, 1);
fixed_lists = strings(n, 1);
position_delta_m = nan(n, 1);
float_x_m = x_est(:,1);
float_y_m = x_est(:,2);
float_z_m = x_est(:,3);
fixed_x_m = xa_est(:,1);
fixed_y_m = xa_est(:,2);
fixed_z_m = xa_est(:,3);

for i = 1:n
    candidates = find(b_est(i,:) ~= 0 & isfinite(b_est(i,:)));
    candidate_counts(i) = numel(candidates);
    if ~isempty(candidates)
        candidate_lists(i) = strjoin(satstr(candidates), ';');
    else
        candidate_lists(i) = "";
    end

    fixed = find(isfinite(ba_est(i,:)));
    fixed_counts(i) = numel(fixed);
    if ~isempty(fixed)
        fixed_lists(i) = strjoin(satstr(fixed), ';');
    else
        fixed_lists(i) = "";
    end

    if all(isfinite(x_est(i,:))) && all(isfinite(xa_est(i,:)))
        position_delta_m(i) = norm(xa_est(i,:) - x_est(i,:));
    end
end

epoch_table = table((1:n)', obs.time.week, obs.time.tow, ratio, idxfix, ...
                    candidate_counts, candidate_lists, fixed_counts, fixed_lists, ...
                    position_delta_m, float_x_m, float_y_m, float_z_m, ...
                    fixed_x_m, fixed_y_m, fixed_z_m, ...
                    'VariableNames', {'epoch','gps_week','gps_tow','ratio','idxfix', ...
                                      'candidate_count','candidate_list', ...
                                      'fixed_count','fixed_list','position_delta_m', ...
                                      'float_x_m','float_y_m','float_z_m', ...
                                      'fixed_x_m','fixed_y_m','fixed_z_m'});
writetable(epoch_table, fullfile(out_dir, "per_epoch_fix_detail.csv"));

rows = n * nsat;
epoch_col = zeros(rows, 1);
gps_week_col = zeros(rows, 1);
gps_tow_col = zeros(rows, 1);
satellite = strings(rows, 1);
reference = strings(rows, 1);
rover_snr = nan(rows, 1);
elevation_deg = nan(rows, 1);
sigma_pdd_m = nan(rows, 1);
sigma_ldd_m = nan(rows, 1);
res_pdd = nan(rows, 1);
res_ldd = nan(rows, 1);
ambiguity_initial = nan(rows, 1);
ambiguity_estimate = nan(rows, 1);

k = 0;
for i = 1:n
    for j = 1:nsat
        k = k + 1;
        ref_idx = reference_index_for_observation(refsatidx, i, j);
        epoch_col(k) = i;
        gps_week_col(k) = obs.time.week(i);
        gps_tow_col(k) = obs.time.tow(i);
        satellite(k) = satstr(j);
        if isfinite(ref_idx) && ref_idx >= 1 && ref_idx <= nsat
            reference(k) = satstr(ref_idx);
        else
            reference(k) = "";
        end
        rover_snr(k) = obs.L1.S(i,j);
        elevation_deg(k) = sat.el(i,j);
        sigma_pdd_m(k) = sigmaP(i,j);
        sigma_ldd_m(k) = sigmaL(i,j);
        res_pdd(k) = obs.L1.resPdd(i,j);
        res_ldd(k) = obs.L1.resLdd(i,j);
        ambiguity_initial(k) = b_ini(i,j);
        ambiguity_estimate(k) = b_est(i,j);
    end
end

sat_table = table(epoch_col, gps_week_col, gps_tow_col, satellite, reference, ...
                  rover_snr, elevation_deg, sigma_pdd_m, sigma_ldd_m, ...
                  res_pdd, res_ldd, ambiguity_initial, ambiguity_estimate, ...
                  'VariableNames', {'epoch','gps_week','gps_tow', ...
                                    'satellite','reference','rover_snr', ...
                                    'elevation_deg','sigma_pdd_m','sigma_ldd_m', ...
                                    'res_pdd','res_ldd','ambiguity_initial', ...
                                    'ambiguity_estimate'});
writetable(sat_table, fullfile(out_dir, "per_sat_detail.csv"));

lambda_path = fullfile(out_dir, "per_lambda_detail.csv");
fid = fopen(lambda_path, 'w');
assert(fid > 0);
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, ['epoch_index,gps_week,gps_tow,solved,fixed_epoch,ratio,' ...
              'candidate_count,row,col,satellite,other_satellite,local_index,' ...
              'other_local_index,ambiguity_float,fixed_ambiguity,covariance,' ...
              'position_covariance_x,position_covariance_y,position_covariance_z\n']);

for i = 1:n
    candidate_idx = find(b_est(i,:) ~= 0 & isfinite(b_est(i,:)));
    candidate_n = length(candidate_idx);
    if candidate_n <= minobs_th
        continue;
    end

    Qfafa = covxb_est(candidate_idx, candidate_idx, i);
    Qxfa = covxb_est(end-2:end, candidate_idx, i);
    lambda_solved = false;
    lambda_fixed = false;
    lambda_ratio = NaN;
    a = nan(2, candidate_n);
    try
        [a,s] = rtklib.lambda(2, b_est(i,candidate_idx), Qfafa);
        lambda_solved = true;
        lambda_ratio = s(2) / s(1);
        lambda_fixed = lambda_ratio > ratio_th;
    catch
        lambda_solved = false;
    end

    for row = 1:candidate_n
        local_index = candidate_idx(row) - 1;
        for col = 1:candidate_n
            other_local_index = candidate_idx(col) - 1;
            fprintf(fid, ['%d,%d,%.15f,%d,%d,%.15f,%d,%d,%d,%s,%s,%d,%d,' ...
                          '%.15f,%.15f,%.15f,%.15f,%.15f,%.15f\n'], ...
                    i - 1, obs.time.week(i), obs.time.tow(i), ...
                    lambda_solved, lambda_fixed, lambda_ratio, candidate_n, ...
                    row - 1, col - 1, char(satstr(candidate_idx(row))), ...
                    char(satstr(candidate_idx(col))), local_index, ...
                    other_local_index, b_est(i,candidate_idx(row)), ...
                    a(1,row), Qfafa(row,col), Qxfa(1,row), Qxfa(2,row), ...
                    Qxfa(3,row));
        end
    end
end

fprintf("Wrote taroz PC debug CSVs to %s\n", out_dir);

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
        ref_values = refsatidx(:);
        if numel(ref_values) >= satellite_index
            ref_idx = ref_values(satellite_index);
        elseif numel(ref_values) >= epoch_index
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
