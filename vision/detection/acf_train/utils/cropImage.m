function Is = cropImage(I, bbr, outputSize)
    height = size(I, 1);
    width = size(I, 2);

    if bbr(4) > height
        bbr(1) = bbr(1) - (bbr(3) - height) / 2;
        bbr(2) = 1;
        bbr(3) = height;
        bbr(4) = height;
    end

    result_bbx = bbr;
    
    if bbr(1) <= 0
        result_bbx(1) = 1;
%         result_bbx(3) = bbr(3) - bbr(1) + 1;
    end
    if bbr(2) <= 0
        result_bbx(2) = 1;
%         result_bbx(4) = bbr(4) - bbr(2) + 1;
    end
    if (bbr(1) + bbr(3)) > width
        result_bbx(1) = width - bbr(3);
    end
    if (bbr(2) + bbr(4)) > height
        result_bbx(2) = height - bbr(4);
    end

    Is = imcrop(I, result_bbx);
    Is=imResample(Is,[outputSize(2),outputSize(1)]);
end
