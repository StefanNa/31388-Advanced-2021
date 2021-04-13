loadWorldInfo

guidemarkFd = fopen("smrGuidemarks", "w+");

fprintf(guidemarkFd, 'array "guidemarkX" %d\n', size(guidemarks,1));
fprintf(guidemarkFd, 'array "guidemarkY" %d\n', size(guidemarks,1));
fprintf(guidemarkFd, 'array "guidemarkTh" %d\n', size(guidemarks,1));

for idx=1:size(guidemarks,1)
    fprintf(guidemarkFd, "guidemarkX[%d]=%.2f\n", idx, guidemarks(idx,1));
    fprintf(guidemarkFd, "guidemarkY[%d]=%.2f\n", idx, guidemarks(idx,2));
    fprintf(guidemarkFd, "guidemarkTh[%d]=%.2f\n", idx, guidemarks(idx,3)/180*pi);
end