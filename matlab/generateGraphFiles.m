loadWorldInfo

graphFd = fopen("laserGraph", "w+");

fprintf(graphFd, 'laser "resetplanner"\n');

for idx=1:size(graphPoints,1)
    fprintf(graphFd, 'laser "addpoint pno=%d x=%.2f y=%.2f"\n', idx,  graphPoints(idx,:));
end

fprintf(graphFd, "\n");

for idx=1:size(graphEdges,1)
    fprintf(graphFd, 'laser "addcon pno1=%d pno2=%d"\n', graphEdges(idx,:));
    fprintf(graphFd, 'laser "addcon pno1=%d pno2=%d"\n', graphEdges(idx, [2 1]));
end

fprintf(graphFd, 'laser "calculatecost"\n');

fclose(graphFd);