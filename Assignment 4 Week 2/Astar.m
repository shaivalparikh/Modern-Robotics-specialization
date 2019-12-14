nodes = csvread('C:\Users\harsh\Desktop\week 2\New folder\nodes.csv');
edges = csvread('C:\Users\harsh\Desktop\week 2\New folder\edges.csv');


function path = Astar(nodes,edges)
%Chapter 10 : Astar function is used to find the most optimal path on a
%graph from a start point to a goal point.
% Takes nodes : The node matrix consists of the node ID, co-ordinated and
%               most optimistic cost.
%       edges : The edges matrix consists of node pairs and their costs.
%Returns path : The optimal path with respect to least cost to travel from
%               start point to goal point.
% Example : the sample files are used for the example without modifying the
%           edge file, hence the result is same in the example.
%clc;clear;
%nodes = csvread('C:\Users\saint\Desktop\Course4codeTestFiles\nodes.csv');
%edges = csvread('C:\Users\saint\Desktop\Course4codeTestFiles\edges.csv');
%path = Astar(nodes,edges)
%path =

%    1     3     4     7    10    12


path = 0;                           
nodesdata = nodes(:,[1,4]);        %Extracting only the nodes and the optimal cost from the nodes matrix
totalnodes = max(nodesdata(:,1));  %Calculating the total number of nodes

mastertable = zeros(5,totalnodes); %Creating a zero mastertable with 5 rows and totalnodes number column
for r = 1:5
	for c = 1:totalnodes
		if r==1
			mastertable(r,c) = nodesdata(c,1);  %Inserting row 1 with the node IDs.
		end
		if r==2                                 %Inserting row 2 with past cost which is 0 for ID1 
				mastertable(r,c) = 0;           %and infinity for others.
			if c==1
			else
				mastertable(r,c) = inf;
			end
		end
		if r ==3                                %Inserting row 3 with the respective optimistic costs
			mastertable(r,c) = nodesdata(c,2);  
		end
		if r ==4                                %Calculating the total estimated costs for each node
			mastertable(r,c) = mastertable(2,c)+mastertable(3,c);
		end
		if r ==5                                %Inserting the parent nodes which is equal to zero at start.
			mastertable(r,c) = 0;
		end
	end
end

openlist = [mastertable(1,1) mastertable(4,1)];  %Creating an open list with ID = 1 and total cost extracted from the
closed = [];                                     %mastertable's 4th row and 1st column. We set the start node as 1. 
ID2 = 1;                                         
                                                 
while ID2 < totalnodes                           %finding the rows in 'edges' where its element is ID2    
    [row,~] = find(edges==ID2);
    temp = edges(row,:);                         %Creating a temp matrix to hold the data of the rows extracted from 
    closesize = size(closed);                    %above step.
    row2 = [];                                   
    for i = 1:closesize(1,1)
        [p,~] = find(temp==closed(i));           %checking if the temp holds any nodes which are present in the closed
        row2 = [row2;p];                         %list and deleting them.
    end
    temp([row2],:)=[];
    x = size(temp);                            %Finding the size of temp especially the number of rows
    for b = 1:x(1,1)                           %Running the for loop till the last row of temp from start.
        if temp(b,1)==ID2
            ID1 = temp(b,2);                   %making sure the ID1 is other than ID2 that we are currently investigating
        else
            ID1 = temp(b,1);
        end
        newcost = mastertable(2,ID2)+ temp(b,3); %Adding the past cost and the new cost to travel to this new node.
        cost = mastertable(2,ID1);
        if mastertable(2,ID1)>newcost
            mastertable(2,ID1) = newcost;
            mastertable(4,ID1) = mastertable(2,ID1) + mastertable(3,ID1);  %Updating the mastertable
            mastertable(5,ID1) = ID2;
        end
        addnode = [ID1,mastertable(4,ID1)];           %Adding the new node to a temporary addnode for further check.
        y = openlist(:,1);                            %Extracts the IDs present in the open list.
        if ismember(ID1,openlist)==1                  %Checks if the currentID is already present.If yes then it finds
            k = find(y ==ID1);                        %its position and checks whether the past cost was greater than 
            if cost>newcost                              %the new cost. If yes then it updates the total cost.
                openlist(k,2)=newcost;
            end                                        
        else                                   
            openlist = [openlist;addnode];            %The new ID is added to the openlist and the list is updated.
        end
    end
    d = find(openlist(:,1)==ID2);
    openlist(d,:) = [];                                %deleting the current ID2 from the open list.
    f = openlist(:,2);
    g = min(f);                                        %selecting the new ID2 based on the minimum total cost in the list.
    h = find(f==g);
    closed = [closed;ID2];                             %Before changing the ID2 for the next iteration, we insert it into
    ID2 = openlist(h,1);                               %the closed list.Then we assign the new ID2 based on the min total
    ID2;                                               %cost in the openlist
end
n = totalnodes;
path = [totalnodes];
while n>1
    a = mastertable(5,n);                 %Writing the path matrix by back tracing from the last node till first.
    path = [path;a];
    n=a;
    

end
path = transpose(path);                   
path = flip(path);                        %Flips the path so that the robot moves forward rather than backward.
csvwrite('path.csv',path);                %Writing the path matrix to a csv file.
return;
end