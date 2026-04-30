clear; clc;

%% Setup
PlutoID = 'usb:0'; 
Fs = 10e6; Fc = 915e6; 
TxGain = 0; RxGain = 70; 
NumCaptures = 8;

tx = sdrtx('Pluto', 'RadioID', PlutoID, 'CenterFrequency', Fc, 'BasebandSampleRate', Fs, 'Gain', TxGain);
rx = sdrrx('Pluto', 'RadioID', PlutoID, 'CenterFrequency', Fc, 'BasebandSampleRate', Fs, 'Gain', RxGain);
rx.SamplesPerFrame = round(Fs * 0.4);
N = 2^nextpow2(rx.SamplesPerFrame);
f_vec = linspace(Fc - Fs/2, Fc + Fs/2, N);
SearchWin = 80e3; NarrowWin = 12e3;

InputFile = '25-Baseline.csv'; 
truthData = readtable(InputFile);
num_tags = height(truthData);

num_objects = 3; % cardboard, plastic bottle, glass bottle, aluminum can
num_poses = 3; % upright, flat, angle

total_signatures = num_objects * num_tags * num_poses;
Fingerprint_Library = zeros(num_tags, total_signatures);
Library_data = strings(total_signatures, 1);

%% Baseline
fprintf('CLEAR BASELINE\n');
input('Press [Enter]...', 's');

tx(complex(ones(1000,1),0)); pause(0.5);
psd_acc_clear = zeros(N, 1);
for c = 1:NumCaptures
    data = double(rx());
    psd_acc_clear = psd_acc_clear + abs(fftshift(fft(data .* taylorwin(length(data)), N))).^2;
end
psd_clear = 10*log10(psd_acc_clear / NumCaptures);

live_freqs = zeros(num_tags, 1);
live_pwr = zeros(num_tags, 1);
for i = 1:num_tags
    target_f = truthData.Freq_MHz(i);
    win_idx = (f_vec >= target_f - SearchWin) & (f_vec <= target_f + SearchWin);
    [max_p, max_idx] = max(psd_clear(win_idx));
    f_window = f_vec(win_idx);
    live_freqs(i) = f_window(max_idx);
    live_pwr(i) = max_p;
end
release(tx); release(rx);
fprintf('Baseline done.\n');

%% ML Loop
fprintf('DATA COLLECTION LOOP\n');

sig_counter = 1;

for obj = 1:num_objects
    obj_name = input(sprintf('\nEnter name for Object %d (e.g. "PlasticBottle"): ', obj), 's');
    
    for t = 1:num_tags
        tag_id = truthData.Tag_IDX{t};
        fprintf('\n%s on Tag %s\n', obj_name, tag_id);
        
        for p = 1:num_poses
            prompt_str = sprintf('Place %s on %s (Pose %d/%d). Press [Enter]...', obj_name, tag_id, p, num_poses);
            input(prompt_str, 's');
            
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
            
            occ_pwr = zeros(num_tags, 1);
            for i = 1:num_tags
                target_f = live_freqs(i); 
                win_idx = (f_vec >= target_f - NarrowWin) & (f_vec <= target_f + NarrowWin);
                [max_p, ~] = max(psd_occ(win_idx));
                occ_pwr(i) = max_p;
            end
            release(tx); release(rx);
            
            power_delta = occ_pwr - live_pwr;
            
            Fingerprint_Library(:, sig_counter) = power_delta;
            Library_data(sig_counter) = sprintf('%s_on_%s_Pose%d', obj_name, tag_id, p);
            
            fprintf(' %d/%d saved.\n', sig_counter, total_signatures);
            sig_counter = sig_counter + 1;
        end
    end
end

%% Save
save('Fingerprint_Library.mat', 'Fingerprint_Library', 'Library_data', 'truthData');
fprintf('Saved.\n');