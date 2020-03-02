% -------------------------------------------------------------------------------------------------
function [imgs, pos, target_sz] = load_video_info(base_path, video)
%LOAD_VOT_VIDEO_INFO
%   Loads all the relevant information for the video in the given path:
%   the list of image files (cell array of strings), initial position
%   (1x2), target size (1x2), the ground truth information for precision
%   calculations (Nx4, for N frames), and the path where the images are
%   located. The ordering of coordinates and sizes is always [y, x].
%
%   Joao F. Henriques, 2014
%   http://www.isr.uc.pt/~henriques/
% -------------------------------------------------------------------------------------------------
	%full path to the video's files
	if base_path(end) ~= '/' && base_path(end) ~= '\', ...
		base_path(end+1) = '/';
	end
	video_path = [base_path video '/imgs/'];

    %load all jpg files in the folder
	img_files = dir([video_path '*.png']);
	assert(~isempty(img_files), 'No image files to load.')
	img_files = sort({img_files.name});
    
    
	%load ground truth from text file
	ground_truth = dlmread([base_path '/' video '/' 'groundtruth.csv']);
    
    figure(1); imshow([video_path, cell2mat(img_files(1))]);
	region =  getrect();%ground_truth(1, :);
    
    %ped - [1217.75,197.25,126,313.5];
    %truckSign - [516,179, 33, 32];
    
	[cx, cy, w, h] = get_axis_aligned_BB(region);
    pos = [cy cx]; % centre of the bounding box
    target_sz = [h w];

	%eliminate frame 0 if it exists, since frames should only start at 1
	img_files(strcmp('00000000.jpg', img_files)) = [];
    img_files = strcat(video_path, img_files);
    % read all frames at once
    imgs = vl_imreadjpeg(img_files,'numThreads', 12);
end

