%% Wall information
walls = [0 0 1.8 0;
    2.2 0 4 0;
    0 0 0 1.8;
    4 0 4 1.8;
    0 3.2 0 5;
    4 3.2 4 5;
    0 5 1.8 5;
    2.2 5 4 5;
    1.7 2.5 1.7 3.1;
    1.7 3.1 0.9 3.1;
    0.9 3.1 0.9 4.3;
    0.9 4.3 3.1 4.3;
    3.1 4.3 3.1 3.1;
    3.1 3.1 2.3 3.1;
    2.3 3.1 2.3 2.5;
    2 4.3 2 3.7;
    1.5 3.7 2.5 3.7];

%% Graph information
graphPoints = [0.3 0.3; %1: start
    0.5 1.5; %2: G2
    0.5 2.25; %3: Left opening
    0.45 3.5; %4: G6
    0.45 4.65; %5: G9-11
    3.55 4.65; %6: G10-12
    3.55 3.5; %7: G7
    3.5 2.25; %8: Right openeing
    3.5 1.5; %9: G3
    3.5 0.3; %10: Bottom right
    -0.5 1.5; %11: G1
    -0.5 2.25; %12: Left outside
    -0.5 3.5; %13: G5
    4.5 1.5; %14: G4
    4.5 2.25; %15: Right outside
    4.5 3.5; %16: G8
    2 2.25; %17: Maze entrance
    2 3.4; %18: Maze middle
    1.2 3.4; %19: Maze left
    1.2 4; %20: Maze top left
    2.8 3.4; %21: Maze right
    2.8 4; %22: Maze top right
    ];

graphEdges = [1 2; %Start to G2
    1 10; %Start to Bottom right
    2 3; %G2 to left opening
    3 4; %Left openeing to G6
    3 12; %Left opening to left outside
    12 11; %Left outside to G1
    12 13; %Left outside to G5
    4 5; %G6 to G9-11
    5 6; %G9-11 to G10-G12
    6 7; %G10-12 to G7
    7 8; %G7 to Right opening
    8 9; %Right opening to G3
    9 10; %G3 to bottom right
    8 15; %Right opening to Right outside
    15 14; %Right outside to G4
    15 16; %Right outside to G8
    3 17; %Left opening to maze entrance
    17 8; %Maze entrance to right opening
    17 18; %Maze entrance to maze middle
    18 19; %Maze middle to maze left
    19 20; %Maze left to maze top left
    18 21; %Maze middle to maze right
    21 22; %Maze right to maze top right
    ];


%% Guidemarks
guidemarks = [
    graphPoints([11 2 9 14 13 4 7 16 5 6 5 6 20 22],:)];
guidemarks(:,3) = [0 180 0 180 0 180 0 180 180 0 90 90 0 180]';
plot(walls(:,[1,3])', walls(:,[2,4])', 'b-')
axis([-1 5 -0.5 5.5])
grid on
hold on
for idx=1:size(graphEdges,1)
    leftEdge = graphPoints(graphEdges(idx,:),1);
    rightEdge = graphPoints(graphEdges(idx,:),2);
    plot(leftEdge, rightEdge, 'r-o')
end