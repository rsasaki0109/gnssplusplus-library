% Dump taroz estimate_pos_PD internals for LibGNSS++ parity checks.
%
% Environment overrides:
%   GNSSPP_TAROZ_ROOT               Root of taroz/PPC-Dataset checkout.
%   GNSSPP_TAROZ_POS_PD_EXAMPLE_DIR Directory containing estimate_pos_PD.m.
%   GNSSPP_TAROZ_POS_PD_OUT_DIR     Output directory for CSV dumps.

taroz_root = getenv("GNSSPP_TAROZ_ROOT");
if strlength(taroz_root) == 0
    taroz_root = "/tmp/taroz_gtsam_gnss";
end

example_dir = getenv("GNSSPP_TAROZ_POS_PD_EXAMPLE_DIR");
if strlength(example_dir) == 0
    example_dir = fullfile(taroz_root, "examples");
end

out_dir = getenv("GNSSPP_TAROZ_POS_PD_OUT_DIR");
if strlength(out_dir) == 0
    out_dir = fullfile(pwd, "output", "dogfood", "taroz_matlab_pos_pd_debug");
end
if ~isfolder(out_dir)
    mkdir(out_dir);
end

set(0, "DefaultFigureVisible", "off");
cd(example_dir);
estimate_pos_PD;
close all force;

out_dir = getenv("GNSSPP_TAROZ_POS_PD_OUT_DIR");
if strlength(out_dir) == 0
    out_dir = fullfile(pwd, "output", "dogfood", "taroz_matlab_pos_pd_debug");
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
valid_position_epochs = nnz(all(isfinite(x_est), 2));
valid_clock_epochs = nnz(all(isfinite(c_est), 2));

graph_table = table(n, nsat, graph_factors, graph_values, initial_cost, ...
                    final_cost, optimizer_error, iterations, ...
                    valid_position_epochs, valid_clock_epochs);
writetable(graph_table, fullfile(out_dir, "graph_detail.csv"));

epoch = (1:n)';
gps_week = obs.time.week(:);
gps_tow = obs.time.tow(:);
spp_x_m = x_ini(:,1);
spp_y_m = x_ini(:,2);
spp_z_m = x_ini(:,3);
fgo_x_m = x_est(:,1);
fgo_y_m = x_est(:,2);
fgo_z_m = x_est(:,3);
fgo_c_gps_m = c_est(:,1);
fgo_c_glo_m = c_est(:,2);
fgo_c_gal_m = c_est(:,3);
fgo_c_qzs_m = c_est(:,4);
fgo_c_bds_m = c_est(:,5);
clock_jump = clockjump(:);

epoch_table = table(epoch, gps_week, gps_tow, spp_x_m, spp_y_m, spp_z_m, ...
                    fgo_x_m, fgo_y_m, fgo_z_m, ...
                    fgo_c_gps_m, fgo_c_glo_m, fgo_c_gal_m, ...
                    fgo_c_qzs_m, fgo_c_bds_m, clock_jump);
writetable(epoch_table, fullfile(out_dir, "per_epoch_state.csv"));

satstr = string(obs.satstr);
rows = n * nsat;
epoch_col = zeros(rows, 1);
gps_week_col = zeros(rows, 1);
gps_tow_col = zeros(rows, 1);
satellite = strings(rows, 1);
system = strings(rows, 1);
snr_dbhz = nan(rows, 1);
elevation_deg = nan(rows, 1);
sigma_p_m = nan(rows, 1);
res_pc_m = nan(rows, 1);
valid_pseudorange = false(rows, 1);
los_x = nan(rows, 1);
los_y = nan(rows, 1);
los_z = nan(rows, 1);

k = 0;
for i = 1:n
    for j = 1:nsat
        k = k + 1;
        epoch_col(k) = i;
        gps_week_col(k) = obs.time.week(i);
        gps_tow_col(k) = obs.time.tow(i);
        satellite(k) = satstr(j);
        system(k) = string(obs.sys(j));
        snr_dbhz(k) = obs.L1.S(i,j);
        elevation_deg(k) = sat.el(i,j);
        sigma_p_m(k) = sigmaP(i,j);
        res_pc_m(k) = obs.L1.resPc(i,j);
        valid_pseudorange(k) = ~isnan(obs.L1.resPc(i,j));
        los_x(k) = -sat.ex(i,j);
        los_y(k) = -sat.ey(i,j);
        los_z(k) = -sat.ez(i,j);
    end
end

p_table = table(epoch_col, gps_week_col, gps_tow_col, satellite, system, ...
                snr_dbhz, elevation_deg, sigma_p_m, res_pc_m, ...
                valid_pseudorange, los_x, los_y, los_z, ...
                'VariableNames', {'epoch','gps_week','gps_tow', ...
                                  'satellite','system','snr_dbhz', ...
                                  'elevation_deg','sigma_p_m','res_pc_m', ...
                                  'valid_pseudorange','los_x','los_y','los_z'});
writetable(p_table, fullfile(out_dir, "per_sat_detail.csv"));

d_rows = (n - 1) * nsat;
epoch_col = zeros(d_rows, 1);
previous_epoch = zeros(d_rows, 1);
gps_week_col = zeros(d_rows, 1);
gps_tow_col = zeros(d_rows, 1);
midpoint_gps_week = zeros(d_rows, 1);
midpoint_gps_tow = zeros(d_rows, 1);
satellite = strings(d_rows, 1);
system = strings(d_rows, 1);
snr_dbhz = nan(d_rows, 1);
elevation_deg = nan(d_rows, 1);
midpoint_elevation_deg = nan(d_rows, 1);
sigma_d_mps = nan(d_rows, 1);
res_d_mps = nan(d_rows, 1);
raw_d_hz = nan(d_rows, 1);
wavelength_m = nan(d_rows, 1);
measured_range_rate_mps = nan(d_rows, 1);
modeled_range_rate_mps = nan(d_rows, 1);
satellite_clock_drift_mps = nan(d_rows, 1);
valid_doppler = false(d_rows, 1);
los_x = nan(d_rows, 1);
los_y = nan(d_rows, 1);
los_z = nan(d_rows, 1);

k = 0;
for i = 2:n
    for j = 1:nsat
        k = k + 1;
        epoch_col(k) = i;
        previous_epoch(k) = i - 1;
        gps_week_col(k) = obs.time.week(i);
        gps_tow_col(k) = obs.time.tow(i);
        midpoint_gps_week(k) = obsi.time.week(i-1);
        midpoint_gps_tow(k) = obsi.time.tow(i-1);
        satellite(k) = satstr(j);
        system(k) = string(obs.sys(j));
        snr_dbhz(k) = obsi.L1.S(i-1,j);
        elevation_deg(k) = sat.el(i,j);
        midpoint_elevation_deg(k) = sati.el(i-1,j);
        sigma_d_mps(k) = sigmaD(i,j);
        res_d_mps(k) = obsi.L1.resD(i-1,j);
        raw_d_hz(k) = obsi.L1.D(i-1,j);
        wavelength_m(k) = obsi.L1.lam(j);
        measured_range_rate_mps(k) = -obsi.L1.D(i-1,j) * obsi.L1.lam(j);
        modeled_range_rate_mps(k) = sati.rate(i-1,j);
        satellite_clock_drift_mps(k) = sati.ddts(i-1,j);
        valid_doppler(k) = ~isnan(obsi.L1.resD(i-1,j)) && ~clockjump(i);
        los_x(k) = -sati.ex(i-1,j);
        los_y(k) = -sati.ey(i-1,j);
        los_z(k) = -sati.ez(i-1,j);
    end
end

d_table = table(epoch_col, previous_epoch, gps_week_col, gps_tow_col, ...
                midpoint_gps_week, midpoint_gps_tow, satellite, system, ...
                snr_dbhz, elevation_deg, midpoint_elevation_deg, ...
                sigma_d_mps, res_d_mps, raw_d_hz, wavelength_m, ...
                measured_range_rate_mps, modeled_range_rate_mps, ...
                satellite_clock_drift_mps, valid_doppler, ...
                los_x, los_y, los_z, ...
                'VariableNames', {'epoch','previous_epoch','gps_week','gps_tow', ...
                                  'midpoint_gps_week','midpoint_gps_tow', ...
                                  'satellite','system','snr_dbhz', ...
                                  'elevation_deg','midpoint_elevation_deg', ...
                                  'sigma_d_mps','res_d_mps','raw_d_hz', ...
                                  'wavelength_m','measured_range_rate_mps', ...
                                  'modeled_range_rate_mps', ...
                                  'satellite_clock_drift_mps', ...
                                  'valid_doppler','los_x','los_y','los_z'});
writetable(d_table, fullfile(out_dir, "per_doppler_detail.csv"));

fprintf("Wrote taroz position PD debug CSVs to %s\n", out_dir);

function value = scalar_or_nan(callback)
    try
        value = callback();
    catch
        value = NaN;
    end
end
