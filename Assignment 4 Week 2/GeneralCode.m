%GeneralCode

%Please refer to the respective function codes in this folder to view their inner workings.


xlow = -0.5;
xhigh = 0.5;
ylow = -0.5;
yhigh = 0.5;
N = 30;
start = [-0.5,-0.5];
goal = [0.5,0.5];
obstacles = csv.read('C:\Users\harsh\Desktop\week 2\New folder\obstacles.csv');


sample = randomsample(xlow,xhigh,ylow,yhigh,N);
nodes = nodescheck(sample,obstacles,start,goal);
edges = validedges(nodes,obstacles,0.5);
path = Astar(nodes,edges);