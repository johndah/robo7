% in [class, topLeftx, topLefty, width, height]

function bbs = loadLabels(dir, imgSize)
    imgWidth = imgSize(1);
    imgHeight = imgSize(2);

    fId=fopen(dir);
    if(fId==-1), error(['unable to open file: ' fName]); end
    frmt='%f %f %f %f %f';
    in=textscan(fId,frmt);
    in=cell2mat(in); 
    in(:, 2) = imgWidth * (in(:, 2) - 0.5*in(:,4));
    in(:, 3) = imgHeight * (in(:, 3) - 0.5*in(:,5));
    in(:, 4) = imgWidth * in(:, 4);
    in(:, 5) = imgHeight * in(:, 5);
    
    for i = 1:size(in, 1)
        if in(i, 2) <= 0
            in(i, 4) = in(i, 4) + in(i, 2);
            in(i, 2) = 1;
        end
        if in(i, 3) <= 0
            in(i, 5) = in(i, 5) + in(i, 3);
            in(i, 3) = 1;
        end
        if (in(i, 2) + in(i, 4)) > imgWidth
            in(i, 4) = imgWidth - in(i, 2);
        end
        if (in(i, 3) + in(i, 5)) > imgHeight
            in(i, 5) = imgHeight - in(i, 3);
        end
    end        
    
    bbs = in;
    fclose(fId);
end