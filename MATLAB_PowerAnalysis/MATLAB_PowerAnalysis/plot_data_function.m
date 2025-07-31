function [combined_data, all_data] = plot_data_function(folder_path)
    % PLOT_DATA_FUNCTION Reads all CSV files in a folder, merges them, and plots the data
    %
    % Input:
    %   folder_path - Path to the folder containing the CSV files
    %
    % Output:
    %   combined_data - Matrix with all merged data
    %   all_data - Cell array with data from each individual file
    %
    % Example:
    %   [data, all] = plot_data_function('C:\path\to\csv\folder');
    
    % Check if the folder exists
    if ~isfolder(folder_path)
        error('The specified folder does not exist: %s', folder_path);
    end
    
    % Find all CSV files in the folder
    csv_files = dir(fullfile(folder_path, '*.csv'));
    
    if isempty(csv_files)
        error('No CSV files found in the folder: %s', folder_path);
    end
    
    fprintf('Found %d CSV files in the folder.\n', length(csv_files));
    
    % Initialize variables
    all_data = cell(length(csv_files), 1);
    combined_data = [];
    time_offset = 0; % Offset to make times consecutive
    
    % Read each CSV file
    for i = 1:length(csv_files)
        file_path = fullfile(csv_files(i).folder, csv_files(i).name);
        fprintf('Reading file: %s\n', csv_files(i).name);
        
        try
            % Read the CSV file
            % Skip the first 2 rows (header and units)
            data = readmatrix(file_path, 'NumHeaderLines', 2);
            
            % Check if the data is valid
            if isempty(data)
                warning('Empty or corrupted file: %s', csv_files(i).name);
                continue;
            end
            
            % Save the original data of the single file
            all_data{i} = data;
            
            % Scale the times to make them consecutive
            if i > 1
                % For files after the first, add the offset
                data(:, 1) = data(:, 1) - min(data(:, 1)) + time_offset;
            else
                % For the first file, normalize starting from 0
                data(:, 1) = data(:, 1) - min(data(:, 1));
            end
            
            % Calculate the offset for the next file
            if ~isempty(data)
                time_offset = max(data(:, 1)) + mean(diff(data(:, 1)));
                %fprintf('  File duration: %.6f s, Next offset: %.6f s\n', ...
                %    max(data(:, 1)) - min(data(:, 1)), time_offset);
            end
            
            % Add the data to the combined matrix
            if isempty(combined_data)
                combined_data = data;
            else
                % Concatenate the data vertically
                combined_data = [combined_data; data];
            end
            
        catch ME
            warning('Error reading file %s: %s', csv_files(i).name, ME.message);
        end
    end
    
    % Check if there is data to plot
    if isempty(combined_data)
        error('No valid data found in the CSV files.');
    end
    
    
    % Extract columns
    time = combined_data(:, 1);
    channel_A = combined_data(:, 2);
    channel_B = combined_data(:, 3);
    channel_C = combined_data(:, 4);
    
    % Create the plot
    figure('Name', 'Raw Combined CSV Data', 'NumberTitle', 'off');
    
    % Main plot with all channels
    subplot(2, 2, [1, 2]);
    plot(time, channel_A, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Channel A');
    hold on;
    plot(time, channel_B, 'r-', 'LineWidth', 0.5, 'DisplayName', 'Channel B', 'Color', [1 0 0 0.25]);
    plot(time, channel_C, 'g-', 'LineWidth', 0.5, 'DisplayName', 'Channel C', 'Color', [0 1 0 0.25]);
    grid on;
    xlabel('Time (s)');
    ylabel('Voltage (V)');
    title('All Channels - Combined Data');
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
    
    % Print statistics
    PRINT_STATISTICS = false; % Set to false to disable statistics printing
    if PRINT_STATISTICS
        fprintf('\n=== DATA STATISTICS ===\n');
        fprintf('Number of files processed: %d\n', length(csv_files));
        fprintf('Time interval: %.6f s to %.6f s\n', min(time), max(time));
        fprintf('Total duration: %.6f s\n', max(time) - min(time));
        fprintf('Number of samples: %d\n', length(time));
        fprintf('Average sampling frequency: %.2f Hz\n', length(time) / (max(time) - min(time)));
        
        fprintf('\nChannel A - Min: %.6f V, Max: %.6f V, Mean: %.6f V\n', ...
            min(channel_A), max(channel_A), mean(channel_A));
        fprintf('Channel B - Min: %.6f V, Max: %.6f V, Mean: %.6f V\n', ...
            min(channel_B), max(channel_B), mean(channel_B));
        fprintf('Channel C - Min: %.6f V, Max: %.6f V, Mean: %.6f V\n', ...
            min(channel_C), max(channel_C), mean(channel_C));
    end    
    
    % Save the combined data with scaled times

end
