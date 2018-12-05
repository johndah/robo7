%% load dataset
clear;
cd(fileparts(which('prepareClassifyData.m'))); dataDir='../../data/';

posGtDir=[dataDir 'Train_clear' '/annotation'];
posImgDir=[dataDir 'Train_clear' '/image'];
outImgDir=[dataDir '/Classify/train/'];

fs={posImgDir,posGtDir};
fs=bbGt('getFiles',fs); nImg=size(fs,2); assert(nImg>0);

imgSize = [640 480];
outputSize = [160 160]; % [w h]
mul = 1.2; % enlarge the bbx of ratio 1.5
shift = 0.25;
small = 0.1;
large = 0.2;
NumIncr = 5;
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
    disp(i);
    
    % load labels
    in = loadLabels(fs{2, i}, imgSize);
    
    for j = 1:size(in, 1)
        obj = in(j, 1);
        bbs = in(j, 2:5);
        
        if bbs(3) < 50 || bbs(4) < 50
            continue;
        end
        aspRatio = bbs(3) / bbs(4);
        if aspRatio < 0.6 || aspRatio > 1.6
            continue;
        end
        
        bbr = bbApply('squarify', bbs, 4, 1 );
        

        
        % labeled image
        Newbbr = bbApply('resize', bbr, mul, mul);
        Is = cropImage(I, Newbbr, outputSize);
        imgName = [names{obj+1}, num2str(accNum(obj+1)), '.png'];
        imwrite(Is, [outImgDir, imgName], 'png');
        
        % generate some random samples
        xyshift = normrnd(0, shift/2, 2, NumIncr);
        xyshift(xyshift > shift) = shift;
        xyshift(xyshift < -shift) = -shift;
        
        flag = randi(3,1, NumIncr);
        bbxScale = zeros(1, NumIncr);
        for ii = 1:NumIncr
            if flag(ii) == 1
                scale = normrnd(0, small/2);
                if scale > 0
                    scale = -scale;
                end
                if scale < -0.1
                    scale = -0.1;
                end
                bbxScale(ii) = 1 + scale;
            else
                scale = normrnd(0, large/2);
                if scale < 0
                    scale = -scale;
                end
                if scale > 0.2
                    scale = 0.2;
                end
                bbxScale(ii) = 1 + scale;
            end    
            % shift
            Newbbr = bbr;
            Newbbr(1,1) = bbr(1,1) + xyshift(1, ii) * bbr(1, 3);
            Newbbr(1,2) = bbr(1,2) + xyshift(2, ii) * bbr(1, 4);
            % resize
            Newbbr = bbApply('resize', Newbbr, bbxScale(ii), bbxScale(ii));
            
            % enlarge with 1.5 times
            Newbbr = bbApply('resize', Newbbr, mul, mul);
            
            % crop the image
            Is = cropImage(I, Newbbr, outputSize);
%             imshow(Is);

            imgName = [names{obj+1}, num2str(accNum(obj+1)), '_', num2str(ii),'.png'];
            imwrite(Is, [outImgDir, imgName], 'png');
        end                   
     
        accNum(obj+1) = accNum(obj+1) + 1;
    end  
    
end
