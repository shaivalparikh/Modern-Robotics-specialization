function [edgelist] = validedges(nodes,obstacles,maxdistancebetweennodes)
%takes in the nodes,obstacles and a parameter called maxdistance between
%nodes which basically makes sure that nodes connect to nearby nodes only.
%OUTPUT : The function returns a list of edges after evaluating a number of
%conditions.
%We start by selecting a node from the nodes list and joing it to another
%node if the distance is greater than 0 and less than maxdistance b/w
%nodes.

nodesize = size(nodes);
obstaclesize = size(obstacles);
invalidedge = [];
edgelist = [];

for i =1:nodesize(1,1)
    pointA = nodes(i,[2 3]);
    for j = 1:nodesize(1,1)
        pointB = nodes(j,[2 3]);
        d = straightlinedistance(pointA,pointB);
        if d>0 && d<maxdistancebetweennodes
            edgelisttemp = [nodes(i,1) nodes(j,1) d];
            edgelist = [edgelist;edgelisttemp]; %We create a temporary edge list containg all possible edges.
            x1 = pointA(1,1);
            x2 = pointB(1,1);
            y1 = pointA(1,2);
            y2 = pointB(1,2);
            xmin = min(x1,x2);
            xmax = max(x1,x2);
            ymin = min(y1,y2);
            ymax = max(y1,y2);
%We now check each edge is it passes through an obstacle.
            for k = 1:obstaclesize(1,1)         
                x0 = obstacles(k,1);
                y0 = obstacles(k,2);
                radius = obstacles(k,3)/2;
                a = y1-y2;
                b = x2-x1;
                c = (((x1-x2)*y1) +(x1*(y2-y1)));
                
%We check for the shortest distance between the line and the circle. IF 
%the distance is less than or equal to the radius of the circle then the
%the line intersects the circle at at least one point. And hence we find
%the root.
                shortestdistance = (abs(a*x0 + b*y0 + c))/sqrt(a^2+b^2);
                if shortestdistance <= radius
                    A = (1+((a^2)/(b^2)));
                    B = 2*((a*c/b^2)+(a*y0/b)-x0);
                    C = ((x0^2)+(y0^2)+(c^2/b^2)+(2*c*y0/b)-(radius^2));
                    P = [A B C];
                    r = roots(P);
                    rootsize = size(r);
                    for m=1:rootsize(1,1)
                        y = -((a*r(m,1)+c)/b);
                        r(m,2)=y;
                    end
                    intpoints = r;
                    intpointsize = size(intpoints);
                    for n = 1:intpointsize(1,1)
                        rootx = intpoints(n,1);
                        rooty = intpoints(n,2);
                        
%We know the bounding box made by the line segment.Therfore now we test 
%whether the roots lie on the line segment or not. If yes then this edge
%is added to a list called 'invalidedge'.

                        if rootx<=xmax && rootx>=xmin && rooty<=ymax && rooty>=ymin
                            invedge = [nodes(i,1) nodes(j,1)];
                            invalidedge = [invalidedge;invedge];
                        end
                    end
                end
            end
        end
    end
end

%We make sure, only unique edges are present in the invalidedge list.

invalidedge;
q = unique(invalidedge,'rows');
invalidedge = q;

%We remove the invalid edges from the temporary edge list called 'edgelist'

x = size(invalidedge);
y = size(edgelist);
row = [];
for i = 1:x(1,1)
    a = invalidedge(i,1);
    b = invalidedge(i,2);
    for j=1:y(1,1)
        c = edgelist(j,1);
        d = edgelist(j,2);
        if a==c && b==d
            row = [row;j];
        end
    end
end

%We then remove any duplicates from the final edgelist.

temp = edgelist;
temp((row),:) = [];
edgelist = temp;
x = size(edgelist);
repeatrow = [];
for i = 1:x(1,1)
    x = size(edgelist);
    if i>x
        break
    end
    for j = 1:x(1,1)
        a = edgelist(i,1);
        b = edgelist(i,2);
        c = edgelist(j,1);
        d = edgelist(j,2);
        if a==d && b==c
            repeatrow = [repeatrow;j];
        end
    end
    temp = edgelist;
    temp((repeatrow),:) = [];
    edgelist = temp;
    repeatrow = [];
end

csvwrite('edges.csv',edgelist);
                    


end