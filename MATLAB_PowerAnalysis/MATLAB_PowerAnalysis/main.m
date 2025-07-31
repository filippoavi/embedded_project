clear all; close all; clc;


% Initialize the folder path containing the CSV files
name_folder = 'long_recording_csv';
folder_path = ['MATLAB_PowerAnalysis\data_PowerAnalysis\' name_folder];

%% Load
% Load the data using the plot_data_function
try
    [combined_data, all_data] = plot_data_function(folder_path);
    
    fprintf('Function executed successfully!\n');
    fprintf('Combined data: %d rows x %d columns\n\n', size(combined_data));
    
catch ME
    fprintf('Error: %s\n', ME.message);
end

%% Process data

% Ignore first 10 seconds of data
ignore_seconds = 10;
% sampling_rate = combined_data(end, 1) / size(combined_data, 1); % Calculate sampling rate
combined_data = combined_data(combined_data(:, 1) >= ignore_seconds, :);

% Define threshold values for channel B and C
threshold_B = 1.5;
threshold_C = 1.5;


% Channel B is in column 3 and channel C is in column 4
channel_B = combined_data(:,3);
channel_C = combined_data(:,4);

% Cases:
% 1. B high, C low
idx_Bhigh_Clow = (channel_B > threshold_B) & (channel_C <= threshold_C);
% 2. B low, C high
idx_Blow_Chigh = (channel_B <= threshold_B) & (channel_C > threshold_C);
% 3. Both high
idx_Bhigh_Chigh = (channel_B > threshold_B) & (channel_C > threshold_C);
% 4. Both low
idx_Blow_Clow = (channel_B <= threshold_B) & (channel_C <= threshold_C);

% Extract data for each case
data_Bhigh_Clow = combined_data(idx_Bhigh_Clow, :);
data_Blow_Chigh = combined_data(idx_Blow_Chigh, :);
data_Bhigh_Chigh = combined_data(idx_Bhigh_Chigh, :);
data_Blow_Clow = combined_data(idx_Blow_Clow, :);

% Display the number of cases for each category
fprintf('B high, C low: %d rows\n', sum(idx_Bhigh_Clow));
fprintf('B low, C high: %d rows\n', sum(idx_Blow_Chigh));
fprintf('Both high: %d rows\n', sum(idx_Bhigh_Chigh));
fprintf('Both low: %d rows\n', sum(idx_Blow_Clow));

% Statistics for each case (Channel A)
fprintf('\nStatistics for each case (Channel A):\n');
fprintf('   B high, C low: Mean = %.4f, Std = %.4f\n', mean(data_Bhigh_Clow(:, 2)), std(data_Bhigh_Clow(:, 2)));
fprintf('   B low, C high: Mean = %.4f, Std = %.4f\n', mean(data_Blow_Chigh(:, 2)), std(data_Blow_Chigh(:, 2)));
fprintf('   Both high:     Mean = %.4f, Std = %.4f\n', mean(data_Bhigh_Chigh(:, 2)), std(data_Bhigh_Chigh(:, 2)));
fprintf('   Both low:      Mean = %.4f, Std = %.4f\n', mean(data_Blow_Clow(:, 2)), std(data_Blow_Clow(:, 2)));

% All Statistics (Channel A)
fprintf('\nOverall Statistics (Channel A):\n');
fprintf('   Mean = %.4f, Std = %.4f\n', mean(combined_data(:, 2)), std(combined_data(:, 2)));



%% Plots
% Extract columns
time = combined_data(:, 1);
channel_A = combined_data(:, 2);
channel_B = combined_data(:, 3);
channel_C = combined_data(:, 4);

% Create the plot
figure('Name', ['Combined CSV Data - ', name_folder], 'NumberTitle', 'off');
% Set figure size
set(gcf, 'Position', [100, 10, 1200, 800]);

% Main plot with all channels
subplot(2, 2, [1, 2]);
hold on;
plot(time, channel_B, 'r-', 'LineWidth', 0.5, 'DisplayName', 'Channel B', 'Color', [1 0 0 0.25]);
plot(time, channel_C, 'g-', 'LineWidth', 0.5, 'DisplayName', 'Channel C', 'Color', [0 1 0 0.25]);
plot(time, channel_A, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Channel A');
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title(['All Channels - Combined Data - ', name_folder], 'Interpreter', 'none');
legend('show', 'Location', 'best');

% Separate plots for each channel
subplot(2, 2, 3);
plot(time, channel_A, 'b-', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Channel A');

subplot(2, 2, 4);
plot(time, channel_B, 'r-', 'LineWidth', 1);
hold on;
plot(time, channel_C, 'g-', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Channel B & C');
legend('Channel B', 'Channel C', 'Location', 'best');

%% Save Plots/Stats

% Save statistics to CSV in 'StatsAnalysisResults' folder
results_folder = 'MATLAB_PowerAnalysis\StatsAnalysisResults';
if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end
results_filename = fullfile(results_folder, [name_folder, '_stats.csv']);

stats_data = {
    'Case',         'Mean',                      'Std';
    'B_high,C_low', mean(data_Bhigh_Clow(:,2)), std(data_Bhigh_Clow(:,2));
    'B_low,C_high', mean(data_Blow_Chigh(:,2)), std(data_Blow_Chigh(:,2));
    'Both_high',     mean(data_Bhigh_Chigh(:,2)), std(data_Bhigh_Chigh(:,2));
    'Both_low',      mean(data_Blow_Clow(:,2)), std(data_Blow_Clow(:,2));
    'Overall',       mean(combined_data(:,2)),    std(combined_data(:,2));
};

% Write to CSV
fid = fopen(results_filename, 'w');
for i = 1:size(stats_data,1)
    if i == 1
        fprintf(fid, '%s,%s,%s\n', stats_data{i,:});
    else
        fprintf(fid, '%s,%.6f,%.6f\n', stats_data{i,:});
    end
end
fclose(fid);
fprintf('Statistics saved to %s\n', results_filename);

% Save the figure as PNG and FIG in the results folder
fig_filename_png = fullfile(results_folder, [name_folder, '_plot.png']);
fig_filename_fig = fullfile(results_folder, [name_folder, '_plot.fig']);
saveas(gcf, fig_filename_png);
saveas(gcf, fig_filename_fig);
fprintf('Plot saved to %s and %s\n', fig_filename_png, fig_filename_fig);
