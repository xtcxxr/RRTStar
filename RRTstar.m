
clearvars
close all

% size of the arena
x_max = 144;
y_max = 60;


% create nexus, ramp, tower and obstacles
obstacles = [];
obstacles(1,:) = [22,45,100,3];
obstacles(2,:) = [68,20,6,8];
obstacles(3,:) = [0,20,5,8];
obstacles(4,:) = [139,20,5,8];
obstacles(5,:) = [30,10,20,2];
obstacles(6,:) = [95,10,20,2];
obstacles(7,:) = [30,35,20,2];
obstacles(8,:) = [95,35,20,2];
obstacles(9,:) = [38,34,5,5];
obstacles(10,:) = [22,55,100,3];
obstacles(11,:) = [38,9,5,5];
obstacles(12,:) = [102,9,5,5];
obstacles(13,:) = [102,34,5,5];
numOfVertices = 500;

% plot the map
figure(1)
axis([0 x_max 0 y_max])
rectangle('Position', obstacles(1,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(2,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(3,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(4,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(5,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(6,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(7,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(8,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(9,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(10,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(11,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(12,:),'FaceColor',[0, 0.4470, 0.7410])
rectangle('Position', obstacles(13,:),'FaceColor',[0, 0.4470, 0.7410])
hold on


step_size = 5;% step size
neighborhood = 10; % radius of neighborhoood

% initialize the coordinate, cost and paretn vertex for the start and end
% positions
v_start.coord = [8,25];
v_start.cost = 0;
v_start.parent = 0;
v_goal.coord = [135,25];
v_goal.cost = 0;
v_start.parent = 0;

% insert first vertex v_start
vertices(1) = v_start;

% randomly generate vertex v_random and plot
for i = 1:1:numOfVertices
    v_rand = [round(rand(1)*x_max),round(rand(1)*y_max)];
    plot(v_rand(1),v_rand(2),'x','Color',[0 1 0]);
    
    % check if the goal has been reached
    for j = 1:1:length(vertices)
        if vertices(j).coord == v_goal.coord
            break
        end
    end
    
    % find the closest vertex
    distances = [];
    for j = 1:1:length(vertices)
        vertex = vertices(j);
        distance = calcDistance(vertex.coord, v_rand);
        distances = [distances distance];
    end
    [vertex_distance,vertex_index] = min(distances);
    v_nearest = vertices(vertex_index);
    
    % initialize the coordinate, cost and parent for v_new
    v_new.coord = [0 0];
    v_new.cost = 0;
    v_new.parent = 0;
    
    % steer from v_random to v_nearest, interpolate if the distance between
    % two vertices is greater than the step size
    % otherwise v_new = v_rand
    if vertex_distance >= step_size
        v_new.coord(1) = v_nearest.coord(1) + (v_rand(1)-v_nearest.coord(1))*step_size/calcDistance(v_rand, v_nearest.coord);
        v_new.coord(2) = v_nearest.coord(2)+ (v_rand(2)-v_nearest.coord(2))*step_size/calcDistance(v_rand, v_nearest.coord);
    else
        v_new.coord(1) = v_rand(1);
        v_new.coord(2) = v_rand(2);
    end
    
    % check collision on the path form v_nearest to v_new, connect them if
    % no collisino is detected
    if notCollided(v_rand,v_nearest.coord,obstacles)
        line([v_nearest.coord(1), v_new.coord(1)],[v_nearest.coord(2), v_new.coord(2)],'Color','k','LineWidth',1)
        drawnow
        hold on
        
        
        % update the cost of v_new
        v_new.cost = norm(v_new.coord - v_nearest.coord) + v_nearest.cost;
        
        % check for neighbor vertices within the heighborhood
        % check collision for all path from neighbors to v_new
        neighbors = [];
        count = 1;
        for j = 1:1:length(vertices)
            if notCollided(vertices(j).coord, v_new.coord, obstacles) && calcDistance(vertices(j).coord, v_new.coord) <= neighborhood
            neighbors(count).coord = vertices(j).coord;      
            neighbors(count).cost = vertices(j).cost;
            count = count + 1;
            end
        end
        
        %initialize best neighbor v_min and minimum cost cost_min
        v_min.coord = v_nearest.coord;
        cost_min = v_new.cost;
        
        % find neighbor vertex with lower cost and check no collision on
        % the path
        % update v_min and connect to v_new
        for k = 1:1:length(neighbors)
            if notCollided(neighbors(k).coord, v_new.coord, obstacles) && neighbors(k).cost + calcDistance(neighbors(k).coord, v_new.coord) < cost_min       
                v_min.coord = neighbors(k).coord;
                cost_min = neighbors(k).cost + calcDistance(neighbors(k).coord, v_new.coord);
                line([v_min.coord(1), v_new.coord(1)], [v_min.coord(2), v_new.coord(2)], 'Color', 'b')
                hold on
            end
        end
        
        %update parent for v_new
        for j = 1:1:length(vertices)
            if vertices(j).coord == v_min.coord
                v_new.parent = j;
            end
        end  
    
    % update the tree, insert v_new
    vertices = [vertices v_new];
    end
end

% calculate the distance of all vertices in the tree
Distance = [];
for i = 1:1:length(vertices)
    dist = calcDistance(vertices(i).coord, v_goal.coord);
    Distance = [Distance dist];
end

% sort the distance list from smallest to largest
% connect the vertex closest to the goal directly to the goal if no
% collision is detected
% else choose the second closetst one, etc until goal is reached
sortedDistance = sort(Distance);
for j = 1:1:length(sortedDistance)
    if notCollided(vertices(j).coord, v_goal.coord, obstacles)    
        v_goal.parent = j;
        vertices = [vertices, v_goal];
        break
    end
          
end

% when the parent vertex of goal is found, connect them
while v_goal.parent ~= 0
    start = v_goal.parent;
    line([v_goal.coord(1), vertices(start).coord(1)], [v_goal.coord(2), vertices(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    v_goal = vertices(start);
end
    

