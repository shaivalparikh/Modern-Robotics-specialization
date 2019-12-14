function distance = straightlinedistance(pointA,pointB)
%FindsThedistancebetweentwopoints
%Takes two points as inputs and find the distance between them.
x1 = pointA(1,1);
x2 = pointB(1,1);
y1 = pointA(1,2);
y2 = pointB(1,2);
distance = sqrt(((x1-x2)^2)+((y1-y2)^2));
end