loadWorldInfo

guidemarkFd = fopen("smrGuidemarks", "w+");

fprintf(guidemarkFd, 'array "gmx" %d\n', size(guidemarks,1)+1);
fprintf(guidemarkFd, 'array "gmy" %d\n', size(guidemarks,1)+1);
fprintf(guidemarkFd, 'array "gmth" %d\n', size(guidemarks,1)+1);

for idx=1:size(guidemarks,1)
    fprintf(guidemarkFd, "gmx[%d]=%.2f\n", idx, guidemarks(idx,1));
    fprintf(guidemarkFd, "gmy[%d]=%.2f\n", idx, guidemarks(idx,2));
    fprintf(guidemarkFd, "gmth[%d]=%.2f\n", idx, guidemarks(idx,3)/180*pi);
end