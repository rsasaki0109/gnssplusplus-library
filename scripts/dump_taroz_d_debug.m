% Dump taroz estimate_vel_D internals for LibGNSS++ parity checks.
%
% Environment overrides:
%   GNSSPP_TAROZ_ROOT          Root of taroz/PPC-Dataset checkout.
%   GNSSPP_TAROZ_D_EXAMPLE_DIR Directory containing estimate_vel_D.m.
%   GNSSPP_TAROZ_D_OUT_DIR     Output directory for CSV dumps.

taroz_root = getenv("GNSSPP_TAROZ_ROOT");
if strlength(taroz_root) == 0
    taroz_root = "/tmp/taroz_gtsam_gnss";
end

example_dir = getenv("GNSSPP_TAROZ_D_EXAMPLE_DIR");
if strlength(example_dir) == 0
    example_dir = fullfile(taroz_root, "examples");
end

out_dir = getenv("GNSSPP_TAROZ_D_OUT_DIR");
if strlength(out_dir) == 0
    out_dir = fullfile(pwd, "output", "dogfood", "taroz_matlab_d_debug");
end
if ~isfolder(out_dir)
    mkdir(out_dir);
end

set(0, "DefaultFigureVisible", "off");
cd(example_dir);
estimate_vel_D;
close all force;

out_dir = getenv("GNSSPP_TAROZ_D_OUT_DIR");
if strlength(out_dir) == 0
    out_dir = fullfile(pwd, "output", "dogfood", "taroz_matlab_d_debug");
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
valid_velocity_epochs = nnz(all(isfinite(v_est), 2));

graph_table = table(n, nsat, graph_factors, graph_values, initial_cost, ...
                    final_cost, optimizer_error, iterations, ...
                    valid_velocity_epochs);
writetable(graph_table, fullfile(out_dir, "graph_detail.csv"));

epoch = (1:n)';
gps_week = obs.time.week(:);
gps_tow = obs.time.tow(:);
spp_x_m = x_ini(:,1);
spp_y_m = x_ini(:,2);
spp_z_m = x_ini(:,3);
fgo_vx_mps = v_est(:,1);
fgo_vy_mps = v_est(:,2);
fgo_vz_mps = v_est(:,3);
fgo_clock_drift_mps = d_est(:,1);

epoch_table = table(epoch, gps_week, gps_tow, spp_x_m, spp_y_m, spp_z_m, ...
                    fgo_vx_mps, fgo_vy_mps, fgo_vz_mps, ...
                    fgo_clock_drift_mps);
writetable(epoch_table, fullfile(out_dir, "per_epoch_vel.csv"));

satstr = string(obs.satstr);
rows = n * nsat;
epoch_col = zeros(rows, 1);
gps_week_col = zeros(rows, 1);
gps_tow_col = zeros(rows, 1);
satellite = strings(rows, 1);
system = strings(rows, 1);
snr_dbhz = nan(rows, 1);
elevation_deg = nan(rows, 1);
sigma_d_mps = nan(rows, 1);
res_d_mps = nan(rows, 1);
raw_d_hz = nan(rows, 1);
wavelength_m = nan(rows, 1);
measured_range_rate_mps = nan(rows, 1);
modeled_range_rate_mps = nan(rows, 1);
satellite_clock_drift_mps = nan(rows, 1);
valid_doppler = false(rows, 1);
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
        sigma_d_mps(k) = sigmaD(i,j);
        res_d_mps(k) = obs.L1.resD(i,j);
        raw_d_hz(k) = obs.L1.D(i,j);
        wavelength_m(k) = obs.L1.lam(j);
        measured_range_rate_mps(k) = -obs.L1.D(i,j) * obs.L1.lam(j);
        modeled_range_rate_mps(k) = sat.rate(i,j);
        satellite_clock_drift_mps(k) = sat.ddts(i,j);
        valid_doppler(k) = ~isnan(obs.L1.resD(i,j));
        los_x(k) = -sat.ex(i,j);
        los_y(k) = -sat.ey(i,j);
        los_z(k) = -sat.ez(i,j);
    end
end

sat_table = table(epoch_col, gps_week_col, gps_tow_col, satellite, system, ...
                  snr_dbhz, elevation_deg, sigma_d_mps, res_d_mps, ...
                  raw_d_hz, wavelength_m, measured_range_rate_mps, ...
                  modeled_range_rate_mps, satellite_clock_drift_mps, ...
                  valid_doppler, los_x, los_y, los_z, ...
                  'VariableNames', {'epoch','gps_week','gps_tow', ...
                                    'satellite','system','snr_dbhz', ...
                                    'elevation_deg','sigma_d_mps','res_d_mps', ...
                                    'raw_d_hz','wavelength_m', ...
                                    'measured_range_rate_mps', ...
                                    'modeled_range_rate_mps', ...
                                    'satellite_clock_drift_mps', ...
                                    'valid_doppler','los_x','los_y','los_z'});
writetable(sat_table, fullfile(out_dir, "per_sat_detail.csv"));

fprintf("Wrote taroz D debug CSVs to %s\n", out_dir);

function value = scalar_or_nan(callback)
    try
        value = callback();
    catch
        value = NaN;
    end
end
