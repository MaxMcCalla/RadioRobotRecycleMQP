%% Truth Table Setup
clear; clc;

%% Setup
PlutoID = 'usb:0';
Fs = 10e6; Fc = 915e6; 
TxGain = 0; RxGain = 50;
CaptureTime = 0.8; 

InputFile = 'RFID_TruthTable.csv';
tagTable = readtable(InputFile);
num_tags = height(tagTable);

%% Scan
tx = sdrtx('Pluto', 'RadioID', PlutoID, 'CenterFrequency', Fc, 'BasebandSampleRate', Fs, 'Gain', TxGain);
rx = sdrrx('Pluto', 'RadioID', PlutoID, 'CenterFrequency', Fc, 'BasebandSampleRate', Fs, 'Gain', RxGain);
rx.SamplesPerFrame = round(Fs * CaptureTime);

for i = 1:num_tags
    current_tag = tagTable.Tag_IDX{i};
    fprintf('\n[TARGET: %s]\n', current_tag);

    input('Tx OFF. Press [Enter]...', 's');
    noise_data = double(rx());
    
    input(['Tx 3" above ', current_tag, '. Press [Enter]...'], 's');
    tx(complex(ones(1000,1),0)); pause(0.4);
    tag_data = double(rx());
    release(tx);
    
    N = 2^nextpow2(length(tag_data));
    win = taylorwin(length(tag_data));
    if size(tag_data,1) ~= size(win,1), win = win'; end
    
    psd_noise = 10*log10(abs(fftshift(fft(noise_data .* win, N))).^2 + eps);
    psd_tag = 10*log10(abs(fftshift(fft(tag_data .* win, N))).^2 + eps);
    f_vec = linspace(Fc - Fs/2, Fc + Fs/2, N);
    
    diff_psd = psd_tag - psd_noise; 
    
    search_window = 1e6; 
    null_width = 200e3;
    
    valid_idx = (f_vec >= Fc - search_window) & (f_vec <= Fc + search_window) & ...
                ~((f_vec > Fc - null_width/2) & (f_vec <= Fc + null_width/2));
                
    diff_psd(~valid_idx) = -100; 
    
    [max_change, peak_idx] = max(diff_psd);
    
    tagTable.Freq_MHz(i) = f_vec(peak_idx);
    tagTable.BaseRSSI(i) = psd_tag(peak_idx);
    
    % figure(1); clf;
    % plot(f_vec/1e6, psd_tag, 'b', 'LineWidth', 1); hold on;
    % plot(f_vec/1e6, psd_noise, 'Color', [0.7 0.7 0.7]); 
    % xline(f_vec(peak_idx)/1e6, 'g--', 'Detected Tag');
    % 
    % grid on; xlim([(Fc - search_window)/1e6, (Fc + search_window)/1e6]);
    % xlabel('Frequency (MHz)'); ylabel('Power (dB)');
    % title(['Live vs Noise for ', current_tag]);
    % legend('Live', 'Noise');
    
    fprintf('Found %s at %.4f MHz (Peak is %.1f dB above noise)\n', ...
            current_tag, tagTable.Freq_MHz(i)/1e6, max_change);
end

release(rx);
writetable(tagTable, '25-Baseline.csv');
fprintf('\nDone.\n');