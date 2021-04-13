loadWorldInfo

wallFd = fopen("wallFinal", "w+");
laserFd = fopen("laserWalls", "w+");

for idx =1:size(walls,1)
    fprintf(wallFd, "%.1f %.1f %.1f %.1f\n", walls(idx,:));
    fprintf(laserFd, 'laser "addline startx=%.1f starty=%.1f endx=%.1f endy=%.1f"\n', walls(idx,:));
end

%fprintf(laserFd, 'laser "setinitpose x=0.3 y=0.3 theta=0"\n');
%fprintf(laserFd, 'laser "setinitcov Cx=0.001 Cy=0.001 Cth=0.001"\n');
%fprintf(laserFd, 'laser "push t=''1'' cmd=''localize''"\n');

fclose(wallFd);
fclose(laserFd);