clear;
cd(fileparts(which('annoCheck.m'))); dataDir='../../data/';

posGtDir=[dataDir 'Train' '/annotation/light/'];
posImgDir=[dataDir 'Train' '/image/light/'];
fs={posImgDir,posGtDir};
fs=bbGt('getFiles',fs); nImg=size(fs,2); assert(nImg>0);

%% draw bbs
color = ['y', 'y', 'g', 'g', 'g', 'm', 'm', 'r', 'r', 'r', 'b', 'b', 'k', 'k'];
% class = ['ball', 'cube', 'cube', 'cylinder', 'hollow', 'cross', 'star', ...
%          'cylinder', 'hollow', 'ball', 'cube', 'triangle', 'cross', 'star'];
%      
outputVideo = VideoWriter('lab_maze2');
outputVideo.FrameRate = 10;
open(outputVideo)
for i = 1:nImg
    fId=fopen(fs{2,i});
    if(fId==-1), error(['unable to open file: ' fName]); end
    frmt='%f %f %f %f %f';
    in=textscan(fId,frmt);
    in=cell2mat(in);
    in(:, 2) = 640 * (in(:, 2) - 0.5*in(:,4));
    in(:, 3) = 480 * (in(:, 3) - 0.5*in(:,5));
    in(:, 4) = 640 * in(:, 4);
    in(:, 5) = 480 * in(:, 5);
    
    fh = figure(10);
    I=imread(fs{1,i});
    imshow(I);
    for j = 1:size(in, 1)
        bbApply('draw', [in(j, 2:5), in(j,1)], color(in(j,1)+1));
    end 
    frm = getframe(fh);
    writeVideo(outputVideo, frm);
    fclose(fId);
end

close(outputVideo);
