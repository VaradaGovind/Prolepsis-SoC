% ========================================================================
%  OrionRV System Simulation
% ========================================================================
clc;
clear;
close all;

%% =====================================================================
%%  GLOBAL PARAMETERS
%% =====================================================================
N  = 2000;              
dt = 0.1;               
Tamb = 25;              
T_crit    = 85;         
T_warn    = 80;         
T_recover = 75;         
max_power = 100;        

%% =====================================================================
%%  PHYSICAL PARAMETERS
%% =====================================================================
C1  = 2.50;             
C2  = 10.0;             
C3  = 50.0;             
R12 = 0.25;             
R23 = 0.20;             
R3a = 0.25;             
R_total = R12 + R23 + R3a;  

%% =====================================================================
%%  SENSOR VARIATIONS
%% =====================================================================
NUM_CORES = 4;
pvt_bias  = [ 0.5, -0.8,  1.2, -0.3];   
pvt_sigma = [ 1.8,  2.5,  1.5,  2.2];   

%% =====================================================================
%%  ESTIMATOR SETTINGS
%% =====================================================================
Q_noise = 0.05;                                    
R_per_core = pvt_sigma.^2;                         

F_ss = 1 - dt / (C1 * R12);                       
B_ss = dt / C1;                                    
G_ss = dt / (C1 * R12);                            

%% =====================================================================
%%  CONTROL THRESHOLDS
%% =====================================================================
dT_max  = dt * (1/C1) * max_power;                
T_L1    = T_crit - dT_max;                        
P_L1_cap = 0.25 * max_power;                      

P_safe = (T_warn - Tamb) / R_total;                
alpha     = 0.85;                                  
max_step  = 8;                                     
T_L1_release = T_L1 - 1.0;                        
T_L2_start   = T_warn - 3.0;                      

%% =====================================================================
%%  WORKLOAD PHASES
%% =====================================================================
PHASE_COMPUTE = 0;
PHASE_MEMORY  = 1;
PHASE_BRANCH  = 2;
PHASE_IDLE    = 3;

phase_weights = [
    +3, -1, -1, -2, -1;    
    -2, +1,  0, +3, -1;    
     0,  0, +4,  0, -1;    
    -4,  0,  0,  0, +1;    
];

%% =====================================================================
%%  MIGRATION SETTINGS
%% =====================================================================
migration_cooldown = 100;     
migration_cost     = 50;      
hysteresis_margin  = 5;       

%% =====================================================================
%%  GENERATE WORKLOADS
%% =====================================================================
rng(42);  
workload = zeros(NUM_CORES, N);
phase_gt = zeros(NUM_CORES, N);  

for c = 1:NUM_CORES
    base = 0.5 + 0.1 * (c - 1);       
    burst_period = 300 + 100 * c;      
    burst = 0.35 * square(2*pi*(1:N)/burst_period);
    wn = 0.05 * randn(1, N);
    workload(c, :) = max(0.05, min(1.0, base + burst + wn));
end

workload(1, 800:1000) = 0.95;   

for c = 1:NUM_CORES
    for k = 1:N
        if workload(c,k) < 0.10
            phase_gt(c,k) = PHASE_IDLE;
        elseif workload(c,k) < 0.35
            phase_gt(c,k) = PHASE_MEMORY;
        elseif workload(c,k) > 0.75
            phase_gt(c,k) = PHASE_COMPUTE;
        else
            phase_gt(c,k) = PHASE_BRANCH;
        end
    end
end

base_load = 0.6;
bursts_1 = 0.4 * square(2*pi*(1:N)/400);
noise_1  = 0.05 * randn(1, N);
workload_1 = max(0.1, min(1.0, base_load + bursts_1 + noise_1));

%% =====================================================================
%%  SIMULATION I: SINGLE CORE TEST
%% =====================================================================

Tj_base = Tamb*ones(1,N); Ts_base = Tamb*ones(1,N); Tp_base = Tamb*ones(1,N);
P_base  = zeros(1,N);

Tj_dvfs = Tamb*ones(1,N); Ts_dvfs = Tamb*ones(1,N); Tp_dvfs = Tamb*ones(1,N);
P_dvfs  = zeros(1,N);

Tj_react = Tamb*ones(1,N); Ts_react = Tamb*ones(1,N); Tp_react = Tamb*ones(1,N);
P_react  = zeros(1,N);
react_throttled = false;

Tj_kf = Tamb*ones(1,N); Ts_kf = Tamb*ones(1,N); Tp_kf = Tamb*ones(1,N);
P_kf  = zeros(1,N);
T_est_kf  = Tamb*ones(1,N);   
T_meas_kf = Tamb*ones(1,N);   
Cov_kf    = ones(1,N);        
L1_active = false(1,N);       
l1_kf_latch = false;          

Ts_est_kf   = Tamb*ones(1,N); 

update_3node = @(Tj, Ts, Tp, P) deal( ...
    Tj + dt/C1 * (P - (Tj - Ts)/R12), ...
    Ts + dt/C2 * ((Tj - Ts)/R12 - (Ts - Tp)/R23), ...
    Tp + dt/C3 * ((Ts - Tp)/R23  - (Tp - Tamb)/R3a) );

for k = 1:N-1
    P_base(k) = max_power;
    [Tj_base(k+1), Ts_base(k+1), Tp_base(k+1)] = ...
        update_3node(Tj_base(k), Ts_base(k), Tp_base(k), P_base(k));

    P_dvfs(k) = workload_1(k) * max_power;
    [Tj_dvfs(k+1), Ts_dvfs(k+1), Tp_dvfs(k+1)] = ...
        update_3node(Tj_dvfs(k), Ts_dvfs(k), Tp_dvfs(k), P_dvfs(k));

    T_meas_react = Tj_react(k) + pvt_bias(1) + pvt_sigma(1) * randn();
    if T_meas_react >= T_crit
        react_throttled = true;
    elseif T_meas_react <= T_recover
        react_throttled = false;
    end
    if react_throttled
        P_react(k) = 0.20 * max_power;
    else
        P_react(k) = workload_1(k) * max_power;
    end
    [Tj_react(k+1), Ts_react(k+1), Tp_react(k+1)] = ...
        update_3node(Tj_react(k), Ts_react(k), Tp_react(k), P_react(k));

    T_meas_kf(k) = Tj_kf(k) + pvt_bias(1) + pvt_sigma(1) * randn();
    
    K_gain    = Cov_kf(k) / (Cov_kf(k) + R_per_core(1));
    T_est_upd = T_est_kf(k) + K_gain * (T_meas_kf(k) - T_est_kf(k));
    Cov_upd   = (1 - K_gain) * Cov_kf(k);

    if T_meas_kf(k) >= T_crit
        P_kf(k) = 0;
        L1_active(k) = true;
        l1_kf_latch = true;
    elseif T_meas_kf(k) >= T_L1 || (l1_kf_latch && T_meas_kf(k) >= T_L1_release)
        P_kf(k) = P_L1_cap;
        L1_active(k) = true;
        l1_kf_latch = true;
    else
        l1_kf_latch = false;
        P_desired = workload_1(k) * max_power;
        T_ahead = T_est_upd + dt/C1 * ...
            (P_desired - (T_est_upd - Ts_est_kf(k))/R12);
        if T_est_upd >= T_L2_start
            throttle_frac = min(1, (T_est_upd - T_L2_start) / (T_L1 - T_L2_start));
            P_ceiling = P_safe - throttle_frac * (P_safe - P_L1_cap);
            P_kf_target = min(P_desired, P_ceiling);
        elseif T_ahead >= T_L2_start
            P_kf_target = min(P_desired, P_safe);
        else
            P_kf_target = P_desired;
        end
        if k == 1
            P_kf(k) = P_kf_target;
        else
            delta_kf = max(min(P_kf_target - P_kf(k-1), max_step), -max_step);
            P_kf(k) = P_kf(k-1) + delta_kf;
        end
        L1_active(k) = false;
    end

    [Tj_kf(k+1), Ts_kf(k+1), Tp_kf(k+1)] = ...
        update_3node(Tj_kf(k), Ts_kf(k), Tp_kf(k), P_kf(k));

    T_est_kf(k+1) = F_ss * T_est_upd + B_ss * P_kf(k) + G_ss * Ts_est_kf(k);
    Cov_kf(k+1)   = F_ss^2 * Cov_upd + Q_noise;

    Ts_est_kf(k+1) = Ts_est_kf(k) + dt/C2 * ...
        ((T_est_upd - Ts_est_kf(k))/R12 - (Ts_est_kf(k) - Tamb)/(R23 + R3a));
end

P_base(N) = P_base(N-1);  P_dvfs(N) = P_dvfs(N-1);
P_react(N) = P_react(N-1); P_kf(N) = P_kf(N-1);
T_meas_kf(N) = Tj_kf(N) + pvt_bias(1) + pvt_sigma(1) * randn();

%% =====================================================================
%%  VISUALIZATION: SINGLE CORE
%% =====================================================================
fprintf('\n=== EVALUATION: Single Core Performance ===\n\n');
fprintf('  %-22s %12s %12s %12s %12s\n', ...
    'Control Type', 'Peak Temp', 'Avg Temp', 'Overheats', 'Throttles');
fprintf('  %s\n', repmat('-', 1, 72));
labels = {'No Control', 'Simple Scaling', 'Reactive Control', 'OrionRV'};
temps  = {Tj_base, Tj_dvfs, Tj_react, Tj_kf};
for i = 1:4
    pk   = max(temps{i});
    av   = mean(temps{i});
    viol = sum(temps{i} > T_crit);
    if i == 4
        l1_cnt = sum(L1_active);
    else
        l1_cnt = 0;
    end
    fprintf('  %-22s %10.2f C  %10.2f C  %10d  %10d\n', ...
        labels{i}, pk, av, viol, l1_cnt);
end
fprintf('\n');

figure('Position', [50 50 1400 700], 'Name', 'Single Core Comparison');
tiled1 = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
time = 1:N;

ax1 = nexttile;
hold on; grid on; box on;
fill([0 N N 0], [T_crit T_crit 130 130], ...
    [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
fill([0 N N 0], [T_L1 T_L1 T_crit T_crit], ...
    [1 0.95 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
plot(time, Tj_base,  ':',  'LineWidth', 2, 'Color', [0.5 0.5 0.5]);
plot(time, Tj_dvfs,        'LineWidth', 1.5, 'Color', [0.0 0.6 0.8]);
plot(time, Tj_react,       'LineWidth', 2.0, 'Color', [0.9 0.5 0.1]);
plot(time, Tj_kf,          'LineWidth', 2.5, 'Color', [0.1 0.2 0.8]);
yline(T_warn, 'k--', 'Warning',  'LineWidth', 1.5, 'FontSize', 10);
yline(T_L1,   'm--', 'Throttle Limit', 'LineWidth', 1.5, 'FontSize', 10);
yline(T_crit, 'r-',  'Critical',  'LineWidth', 1.5, 'FontSize', 10);
title('Single Core Temperature', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('Temperature (C)', 'FontSize', 13, 'FontWeight', 'bold');
ylim([20 120]); xlim([0 N]);
legend('Danger Zone', 'Safety Buffer', ...
    'No Control', 'Simple Scaling', 'Reactive Control', 'OrionRV', ...
    'Location', 'eastoutside');

ax2 = nexttile;
hold on; grid on; box on;
area(time, P_dvfs, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'FaceColor', [0.6 0.6 0.6]);
plot(time, P_react, 'LineWidth', 1.5, 'Color', [0.9 0.5 0.1]);
plot(time, P_kf,    'LineWidth', 2.5, 'Color', [0.1 0.2 0.8]);

l1_idx = find(L1_active);
if ~isempty(l1_idx)
    scatter(l1_idx, P_kf(l1_idx), 20, 'r', 'filled', 'MarkerFaceAlpha', 0.6);
end
yline(P_L1_cap, 'r--', 'Power Cap', 'LineWidth', 1.5);
title('Single Core Power', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Time Steps', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Power (%)', 'FontSize', 13, 'FontWeight', 'bold');
ylim([0 110]); xlim([0 N]);
if ~isempty(l1_idx)
    legend('Required Power', 'Reactive Control', 'OrionRV', ...
        'Safety Throttling', 'Location', 'eastoutside');
else
    legend('Required Power', 'Reactive Control', 'OrionRV', ...
        'Location', 'eastoutside');
end
linkaxes([ax1 ax2], 'x');

%% =====================================================================
%%  SIMULATION II: MULTI-CORE SYSTEM
%% =====================================================================

Tj = Tamb * ones(NUM_CORES, N);
Ts = Tamb * ones(NUM_CORES, N);
Tp = Tamb * ones(NUM_CORES, N);
P_core = zeros(NUM_CORES, N);

T_est   = Tamb * ones(NUM_CORES, N);
T_meas  = Tamb * ones(NUM_CORES, N);
Cov     = ones(NUM_CORES, N);
Ts_est  = Tamb * ones(NUM_CORES, N);  

phase_detected = zeros(NUM_CORES, N);

L1_events       = zeros(NUM_CORES, N);
migration_log   = [];  
last_migration   = -migration_cooldown * ones(1, NUM_CORES);
power_budget     = max_power * ones(NUM_CORES, N);
throttle_level   = ones(NUM_CORES, N);   

for k = 1:N-1
    for c = 1:NUM_CORES
        headroom = T_warn - T_est(c, k);
        power_budget(c, k) = max(P_L1_cap, min(max_power, ...
            max_power * headroom / (T_warn - Tamb)));
    end

    if k > 1
        compute_cores = find(phase_detected(:, k-1) == PHASE_COMPUTE);
        for c = 1:NUM_CORES
            if phase_detected(c, k-1) == PHASE_MEMORY || phase_detected(c, k-1) == PHASE_IDLE
                excess = power_budget(c, k) - P_core(c, k-1);
                if excess > 0 && ~isempty(compute_cores)
                    per_core_share = (excess * 0.5) / numel(compute_cores);
                    actually_given = 0;
                    for cc = compute_cores'
                        accepted = min(per_core_share, max_power - power_budget(cc, k));
                        power_budget(cc, k) = power_budget(cc, k) + accepted;
                        actually_given = actually_given + accepted;
                    end
                    power_budget(c, k) = power_budget(c, k) - actually_given;
                end
            end
        end
    end

    for c = 1:NUM_CORES
        T_meas(c, k) = Tj(c, k) + pvt_bias(c) + pvt_sigma(c) * randn();
        
        K_gain = Cov(c, k) / (Cov(c, k) + R_per_core(c));
        T_est_upd = T_est(c, k) + K_gain * (T_meas(c, k) - T_est(c, k));
        Cov_upd   = (1 - K_gain) * Cov(c, k);

        ipc        = workload(c, k);
        cache_miss = max(0, 0.3 - 0.3 * workload(c, k) + 0.05*randn());
        branch_miss= max(0, 0.1 + 0.05*randn());
        stall_ratio= max(0, 1 - workload(c, k) + 0.05*randn());
        features = [ipc, cache_miss, branch_miss, stall_ratio];
        scores = phase_weights(:, 1:4) * features' + phase_weights(:, 5);
        [~, best] = max(scores);
        phase_detected(c, k) = best - 1;  

        if T_meas(c, k) >= T_crit
            P_core(c, k) = 0;
            L1_events(c, k) = 1;
            throttle_level(c, k) = 0;
        elseif T_meas(c, k) >= T_L1
            P_core(c, k) = P_L1_cap;
            L1_events(c, k) = 1;
            throttle_level(c, k) = 0.25;
        else
            P_desired = workload(c,k) * max_power;
            T_ahead = T_est_upd + dt/C1 * ...
                (P_desired - (T_est_upd - Ts_est(c,k))/R12);
            
            if T_est_upd >= T_L2_start
                throttle_frac = min(1, (T_est_upd - T_L2_start) / (T_L1 - T_L2_start));
                P_ceiling = P_safe - throttle_frac * (P_safe - P_L1_cap);
                P_target = min(P_desired, P_ceiling);
            elseif T_ahead >= T_L2_start
                P_target = min(P_desired, P_safe);
            else
                P_target = P_desired;
            end

            P_target = min(P_target, power_budget(c, k));
            
            if k == 1
                P_core(c, k) = P_target;
            else
                delta = max(min(P_target - P_core(c, k-1), max_step), -max_step);
                P_core(c, k) = P_core(c, k-1) + delta;
            end
            throttle_level(c, k) = P_core(c, k) / max_power;
        end

        Tj(c, k+1) = Tj(c,k) + dt/C1 * (P_core(c,k) - (Tj(c,k)-Ts(c,k))/R12);
        Ts(c, k+1) = Ts(c,k) + dt/C2 * ((Tj(c,k)-Ts(c,k))/R12 - (Ts(c,k)-Tp(c,k))/R23);
        Tp(c, k+1) = Tp(c,k) + dt/C3 * ((Ts(c,k)-Tp(c,k))/R23 - (Tp(c,k)-Tamb)/R3a);

        T_est(c, k+1) = F_ss * T_est_upd + B_ss * P_core(c,k) + G_ss * Ts_est(c,k);
        Cov(c, k+1)   = F_ss^2 * Cov_upd + Q_noise;

        Ts_est(c, k+1) = Ts_est(c,k) + dt/C2 * ...
            ((T_est_upd - Ts_est(c,k))/R12 - (Ts_est(c,k) - Tamb)/(R23 + R3a));
    end

    if k > 1
        for c = 1:NUM_CORES
            phase_changed = (phase_detected(c, k) ~= phase_detected(c, k-1));
            if phase_changed && phase_detected(c, k) == PHASE_COMPUTE && ...
                    (k - last_migration(c)) > migration_cooldown
                
                headrooms = T_warn - T_est(:, k);
                headrooms(c) = -inf;  
                [best_headroom, dst] = max(headrooms);
                
                benefit = T_est(c, k) - T_est(dst, k);
                if benefit > hysteresis_margin && best_headroom > hysteresis_margin
                    stall_end = min(N, k + migration_cost);
                    workload(c, k+1:stall_end) = 0.05;
                    workload(dst, k+1:stall_end) = 0.05;
                    
                    temp_wl = workload(c, stall_end+1:end);
                    workload(c, stall_end+1:end) = workload(dst, stall_end+1:end);
                    workload(dst, stall_end+1:end) = temp_wl;
                    
                    temp_ph = phase_gt(c, stall_end+1:end);
                    phase_gt(c, stall_end+1:end) = phase_gt(dst, stall_end+1:end);
                    phase_gt(dst, stall_end+1:end) = temp_ph;
                    
                    migration_log = [migration_log; k, c, dst];
                    last_migration(c)   = k;
                    last_migration(dst) = k;
                end
            end
        end
    end
end

P_core(:, N) = P_core(:, N-1);
for c = 1:NUM_CORES
    T_meas(c, N)  = Tj(c, N) + pvt_bias(c) + pvt_sigma(c) * randn();
    Ts_est(c, N)  = Ts_est(c, N-1);
    T_est(c, N)   = T_est(c, N-1);
    Cov(c, N)     = Cov(c, N-1);
end

%% =====================================================================
%%  VISUALIZATION: MULTI-CORE SYSTEM
%% =====================================================================
fprintf('\n=== EVALUATION: Multi-Core Operations ===\n\n');
fprintf('  %-8s %10s %10s %10s %10s %12s\n', ...
    'Core', 'Peak Temp', 'Avg Temp', 'Overheats', 'Throttles', 'Avg Power');
fprintf('  %s\n', repmat('-', 1, 65));
for c = 1:NUM_CORES
    fprintf('  Core %d   %8.2f C  %8.2f C  %8d  %10d  %10.1f%%\n', ...
        c-1, max(Tj(c,:)), mean(Tj(c,:)), sum(Tj(c,:) > T_crit), ...
        sum(L1_events(c,:)), mean(throttle_level(c,:))*100);
end
fprintf('\n  Task Moves Logged: %d\n', size(migration_log, 1));

core_colors = [0.8 0.2 0.2;  0.2 0.6 0.8;  0.3 0.7 0.3;  0.7 0.4 0.8];
figure('Position', [80 30 1400 700], 'Name', 'Multi-Core System Response');
tiled2 = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax2_1 = nexttile;
hold on; grid on; box on;
fill([0 N N 0], [T_crit T_crit 130 130], ...
    [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
fill([0 N N 0], [T_L1 T_L1 T_crit T_crit], ...
    [1 0.95 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.2);
for c = 1:NUM_CORES
    plot(time, Tj(c,:), 'LineWidth', 2, 'Color', core_colors(c,:));
end

for m = 1:size(migration_log, 1)
    xline(migration_log(m,1), 'k--', sprintf('Move %d->%d', ...
        migration_log(m,2)-1, migration_log(m,3)-1), ...
        'LineWidth', 1.5, 'FontSize', 9, 'LabelOrientation', 'horizontal');
end

yline(T_warn, 'k--', 'LineWidth', 1);
yline(T_L1,   'm--', 'LineWidth', 1);
yline(T_crit, 'r-',  'LineWidth', 1.5);
title('Multi-Core Temperature (OrionRV)', ...
    'FontSize', 15, 'FontWeight', 'bold');
ylabel('Temperature (C)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([20 100]); xlim([0 N]);
legend('Danger Zone', 'Safety Buffer', ...
    'Core 0', 'Core 1', 'Core 2', 'Core 3', 'Location', 'eastoutside');

ax2_2 = nexttile;
hold on; grid on; box on;
for c = 1:NUM_CORES
    plot(time, P_core(c,:), 'LineWidth', 1.8, 'Color', core_colors(c,:));
end
yline(P_L1_cap, 'r--', 'Power Cap', 'LineWidth', 1.5);
title('Multi-Core Power', ...
    'FontSize', 15, 'FontWeight', 'bold');
xlabel('Time Steps', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Power (%)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([0 110]); xlim([0 N]);
legend('Core 0', 'Core 1', 'Core 2', 'Core 3', 'Location', 'eastoutside');
linkaxes([ax2_1 ax2_2], 'x');

fprintf('\n=== EXECUTION COMPLETE ===\n\n');