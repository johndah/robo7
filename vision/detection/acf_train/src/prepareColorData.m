%% load dataset
clear;
cd(fileparts(which('prepareClassifyData.m'))); dataDir='../../data/';

posGtDir=[dataDir 'Train' '/annotation'];
posImgDir=[dataDir 'Train' '/image'];
outImgDir=[dataDir '/ColorClass/train/'];

fs={posImgDir,posGtDir};
fs=bbGt('getFiles',fs); nImg=size(fs,2); assert(nImg>0);

imgSize = [640 480];
outputSize = [80 80]; % [w h]
% mul = 1.5;
%% Process dataset
names = {'Y_ball', 'Y_cube', ...
         'G_cube', 'G_cylinder', 'G_hollow', ...
         'O_cross', 'O_star', ...
         'R_cylinder', 'R_hollow', 'R_ball', ...
         'B_cube', 'B_triangle', ...
         'P_cross', 'P_star'};
accNum = zeros(14);
for i = 1:nImg
    % load image 
    I = imread(fs{1, i});
    
    % load labels
    in = loadLabels(fs{2, i}, imgSize);
    
    % the width and height should large than 50
    in = in(in(:,4)>=50,:);
    in = in(in(:,5)>=50,:);
    
    for j = 1:size(in, 1)
        obj = in(j, 1);
        bbs = in(j, 2:5);
        bbr = bbApply('squarify', bbs, 4, 1 );
%         bbr = bbApply('resize', bbr, mul, mul);
    
        Is=bbApply('crop',I,bbr,'replicate', outputSize);
        
        imgName = [names{obj+1}, num2str(accNum(obj+1)), '.png'];
        imwrite(Is{1}, [outImgDir, imgName], 'png');
        
        accNum(obj+1) = accNum(obj+1) + 1;
    end
    
    
    
    
end

