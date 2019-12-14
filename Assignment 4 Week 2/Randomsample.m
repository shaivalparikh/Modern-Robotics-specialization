function sample = randomsample(xlow,xhigh,ylow,yhigh,N)
%Random sample is created from the range of x and y which the user
%inputs.And also a value 'N' which is equal to total number of sample
%points. A value of N=30 is found to be good enough. Higher values will be
%give more accurate results while lower values will be computationally
%efficient.

x = (xhigh-xlow).*rand(N,1)+xlow;
y = (yhigh-ylow).*rand(N,1)+ylow;
sample = [];

%x and y co-ordinates are then merged to form the new co-ordinates.

sample(:,1) = x;
sample(:,2) = y;
end