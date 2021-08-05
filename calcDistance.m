function distance = calcDistance(v1, v2)
%calculate distance between two points
%input: two points v1 and v2 
%output: d is the distance between v1 and v2
distance = norm(v2 - v1);
end