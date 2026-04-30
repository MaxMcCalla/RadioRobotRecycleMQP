%% RFID Board Scan
clear; clc;

%% Setup
load('Fingerprint_Library.mat', 'Fingerprint_Library', 'Library_data');
PlutoID = 'usb:0';
Fs = 10e6; Fc = 915e6; 
TxGain = 0; RxGain = 70; 
NumCaptures = 8;    
InputFile = '25-Baseline.csv'; 
truthData = readtable(InputFile);
num_tags = height(truthData);
tx = sdrtx('Pluto', 'RadioID', PlutoID, 'CenterFrequency', Fc, 'BasebandSampleRate', Fs, 'Gain', TxGain);
rx = sdrrx('Pluto', 'RadioID', PlutoID, 'CenterFrequency', Fc, 'BasebandSampleRate', Fs, 'Gain', RxGain);
rx.SamplesPerFrame = round(Fs * 0.4);

%% Clear Scan
fprintf('\nCLEAR SCAN\n');
input('Enter...', 's');
tx(complex(ones(1000,1),0)); pause(0.5);
N = 2^nextpow2(rx.SamplesPerFrame);
psd_acc_clear = zeros(N, 1);
for c = 1:NumCaptures
    data = double(rx());
    psd_acc_clear = psd_acc_clear + abs(fftshift(fft(data .* taylorwin(length(data)), N))).^2;
end
psd_clear = 10*log10(psd_acc_clear / NumCaptures);
f_vec = linspace(Fc - Fs/2, Fc + Fs/2, N);
live_freqs = zeros(num_tags, 1);
live_pwr = zeros(num_tags, 1);
SearchWindow = 80e3; 
for i = 1:num_tags
    target_f = truthData.Freq_MHz(i);
    win_idx = (f_vec >= target_f - SearchWindow) & (f_vec <= target_f + SearchWindow);
    [max_p, max_idx] = max(psd_clear(win_idx));
    f_window = f_vec(win_idx);

    live_freqs(i) = f_window(max_idx);
    live_pwr(i) = max_p;
end
release(tx); release(rx);
%% ROS 
% fprintf('\nROS node\n');
% try
%     rf_node = ros2node("/rf_occlusion_node");
%     fprintf('node created.\n');
% catch
%     fprintf('already exists.\n');
% end
% roi_pub = ros2publisher(rf_node, "/rf_roi", "geometry_msgs/Polygon");
%% Sorting Loop
sorting_active = true;
while sorting_active
    %% Occluded Scan
    fprintf('\nOCCLUDED SCAN\n');
    input('Enter...', 's');
    tx = sdrtx('Pluto', 'RadioID', PlutoID, 'CenterFrequency', Fc, 'BasebandSampleRate', Fs, 'Gain', TxGain);
    rx = sdrrx('Pluto', 'RadioID', PlutoID, 'CenterFrequency', Fc, 'BasebandSampleRate', Fs, 'Gain', RxGain);
    rx.SamplesPerFrame = round(Fs * 0.4);
    tx(complex(ones(1000,1),0)); pause(0.5);
    psd_acc_occ = zeros(N, 1);
    for c = 1:NumCaptures
        data = double(rx());
        psd_acc_occ = psd_acc_occ + abs(fftshift(fft(data .* taylorwin(length(data)), N))).^2;
    end
    psd_occ = 10*log10(psd_acc_occ / NumCaptures);
    occ_freqs = zeros(num_tags, 1);
    occ_pwr = zeros(num_tags, 1);
    NarrowWindow = 12e3;
    for i = 1:num_tags
        target_f = live_freqs(i); 
        win_idx = (f_vec >= target_f - NarrowWindow) & (f_vec <= target_f + NarrowWindow);
        [max_p, max_idx] = max(psd_occ(win_idx));
        f_window = f_vec(win_idx);
        
        occ_freqs(i) = f_window(max_idx);
        occ_pwr(i) = max_p;
    end
    release(tx); release(rx);

%% PROCESSING TRIAL 3

    live_delta = occ_pwr - live_pwr;
    live_centered = live_delta - mean(live_delta); 

    spatial_confidence = zeros(num_tags, 1);
    best_match_labels = strings(num_tags, 1);

    for t = 1:num_tags
        tag_id = truthData.Tag_IDX{t};
        tag_indices = contains(Library_data, sprintf('_on_%s_', tag_id));
        tag_signatures = Fingerprint_Library(:, tag_indices);
        tag_labels = Library_data(tag_indices);

        best_score = -1;
        best_label = "";

        for k = 1:size(tag_signatures, 2)
            lib_sig = tag_signatures(:, k);

            lib_centered = lib_sig - mean(lib_sig);

            dot_prod = dot(live_centered, lib_centered);
            mag = norm(live_centered) * norm(lib_centered);
            score = dot_prod / (mag + 1e-9); 

            if score > best_score
                best_score = score;
                best_label = tag_labels(k);
            end
        end
        spatial_confidence(t) = best_score;
        best_match_labels(t) = best_label;
    end

    [sortedScores, sortedIdx] = sort(spatial_confidence, 'descend');

    fprintf('\n(ROI)\n');
    for k = 1:num_tags
        fprintf('Tag %s | Confidence: %5.1f%%\n', ...
                truthData.Tag_IDX{sortedIdx(k)}, sortedScores(k)*100);
    end

    fprintf('\nTOP 3\n');
    top_3_X = zeros(3,1);
    top_3_Y = zeros(3,1);
    for i = 1:min(3, num_tags)
        tag_id = truthData.Tag_IDX{sortedIdx(i)};
        top_3_X(i) = truthData.X(sortedIdx(i));
        top_3_Y(i) = truthData.Y(sortedIdx(i));
        fprintf('Priority %d: %s [X: %5.2f, Y: %5.2f]\n', ...
                i, tag_id, top_3_X(i), top_3_Y(i));
    end
    
    %% Heatmap
    figure(1); clf;
    buffer = 5; 
    [xq, yq] = meshgrid(linspace(min(truthData.X)-buffer, max(truthData.X)+buffer, 100), ...
                        linspace(min(truthData.Y)-buffer, max(truthData.Y)+buffer, 100));
    vq = griddata(truthData.X, truthData.Y, spatial_confidence, xq, yq, 'v4');
    contourf(xq, yq, vq, 15, 'LineColor', 'none'); 
    colormap(turbo); colorbar; hold on;
    plot(truthData.X, truthData.Y, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
    plot(top_3_X, top_3_Y, 'r*', 'MarkerSize', 12, 'LineWidth', 2);
    title('RF Heatmap');
    xlabel('X (mm)'); ylabel('Y (mm)');
    axis image; grid on;
    %% ROS
    % roi_msg = ros2message(roi_pub);
    % for i = 1:3
    %     point = ros2message("geometry_msgs/Point32");
    %     point.x = single(top_3_X(i) / 100); 
    %     point.y = single(top_3_Y(i) / 100);
    %     point.z = single(0.0);
    %     roi_msg.points(i) = point;
    % end
    % send(roi_pub, roi_msg);
    % fprintf('Sent to CV.\n');
    % 
    % cmd = input('\n[P] to Rescan, [Q] to Quit: ', 's');
    % if strcmpi(cmd, 'q')
    %     sorting_active = false; 
    % end
end