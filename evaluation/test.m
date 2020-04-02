load('siamFC_original.mat');
load('siamFC_kalman_TU.mat');


threshold = (0:5:100)/100;

nVideos = size(bboxes, 1);

cnt = 0;
for i = 1:nVideos
    for j = 1:size(bboxes{i, 1}, 1)
        cnt = cnt + 1;
        bbox_siamFC(cnt, :) = bboxes{i, 1}(j, :);
        bbox_kalman(cnt, :) = bboxes_kalman{i, 1}(j, :);
        [cx, cy, w, h] = get_axis_aligned_BB(ground_truth{i, 1}(j, :));
        gt(cnt, :) = [cx-w/2, cy-h/2, w, h];
    end
end

successRate = cell(length(threshold), 1);
failure_siam = zeros(length(threshold), 1);
failure_kalman = zeros(length(threshold), 1);

nFrames = size(gt, 1);
res_siam = zeros();
for j = 1:length(threshold)
    result_siamFC = zeros(nFrames, 1);
    result_kalman = zeros(nFrames, 1);
    
    res_siamFC = zeros(nFrames, 1);
    res_kalman = zeros(nFrames, 1);
    for i = 1:nFrames
        gt_bbox = gt(i, :);
        siam_bbox = bbox_siamFC(i, :);
        kalman_bbox = bbox_kalman(i, :);
        
        res_siam(i, 1) = getBboxOverlap(gt_bbox, siam_bbox);
        res_kalman(i, 1) = getBboxOverlap(gt_bbox, kalman_bbox);
        
        if res_siam(i, 1) > threshold(j)
            result_siamFC(i, 1) = 1;
        else
            result_siamFC(i, 1) = 0;
        end
        
        if res_kalman(i, 1) > threshold(j)
            result_kalman(i, 1) = 1;
        else
            result_kalman(i, 1) = 0;
        end
        
    end
    successRate{j, 1} = mean(result_siamFC);
    successRate{j, 2} = mean(result_kalman);
    
    failure_siam(j, 1) = length(result_siamFC)-sum(result_siamFC);
    failure_kalman(j, 1) = length(result_siamFC)-sum(result_siamFC);
end

accuracy_siam = mean(res_siam)*100;
fprintf('SiamFC accuracy = %0.2f\n', accuracy_siam);
accuracy_kalman = mean(res_kalman)*100;
fprintf('SiamFC_Kalman accuracy = %0.2f\n', accuracy_kalman);


figure;
plot(threshold, cell2mat(successRate(:, 1)), 'g', 'Linewidth', 3); hold on
plot(threshold, cell2mat(successRate(:, 2)), 'r', 'Linewidth', 3); 
legend({'siamFC', 'siamFC\_Kalman'}, 'Fontsize', 16);
axis([0, 1, -0.1, 1.1]);

