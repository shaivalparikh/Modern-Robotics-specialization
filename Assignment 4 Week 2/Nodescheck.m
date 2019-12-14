function nodes = nodescheck(sample,obstacles,start,goal)

%Takes a sample set of points as input, an obstacle list and a start and
%goal configuration.
%The function removes any nodes lying on the perimeter of the obstacles or
%inside them returning only those points which are in the free space. It
%then calcultes the heuristic cost to go to the goal from each node.
%Finally it returns a list of nodes with its heuristic cost to go and
%writes a csv file with the name 'nodes'.

sizeobstacles = size(obstacles);
sizesample = size(sample);
invalidnodes= [];
for i=1:sizeobstacles(1,1)
    pointA = obstacles(i,[1 2]);
    for j = 1:sizesample(1,1)
        pointB = sample(j,:);
        d = straightlinedistance(pointA,pointB);  %Calculates the straightline distance between the center of an obstacle
        point = sample(j,:);                      %to the node at present.
        if d <= (obstacles(i,3)/2)
            invalidnodes = [invalidnodes;point];  %if distance is less than or equal to radius the node is added to a list
        end                                       %called invalidnodes.
    end
end


invalidnodes = unique(invalidnodes,'rows');       %Any repeating node is removed from the list.
x = size(invalidnodes);
y = size(sample);
row = [];
for i = 1:x(1,1)
    a = invalidnodes(i,1);
    b = invalidnodes(i,2);
    for j=1:y(1,1)
        c = sample(j,1);
        d = sample(j,2);
        if a==c && b==d
            row = [row;j];
        end
    end
end
temp = sample;
temp([row],:) = [];                               %The set of invalid nodes is removed from the node list.
nodes = temp;
nodes= [start;nodes];                             %Adding the start node at the top and the goal node at the bottom of the
nodes=[nodes;goal];                               %list.

nodesize = size(nodes);
id=[];
for i = 1:nodesize(1,1)
    pointA = nodes(i,[1 2]);
    d = straightlinedistance(pointA,goal);        %Heuristic cost to go is calculated and added to the list for the
    nodes(i,3)=d;                                 %respective nodes.
    id = [id;i];
end
nodes=[id nodes];                                 %ID's are given to each node.
nodes;
csvwrite('nodes.csv',nodes);
end