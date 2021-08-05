function noCollision = noCollision(v_1, v_2, obstacle)
    A = [v_2(1) v_2(2)];
    B = [v_1(1) v_1(2)];
    obstacles = [obstacle(1) obstacle(2) obstacle(1)+obstacle(3) obstacle(2)+obstacle(4)];
    
    C1 = [obstacles(1),obstacles(2)];
    D1 = [obstacles(1),obstacles(4)];
    C2 = [obstacles(1),obstacles(2)];
    D2 = [obstacles(3),obstacles(2)];
    C3 = [obstacles(3),obstacles(4)];
    D3 = [obstacles(3),obstacles(2)];
    C4 = [obstacles(3),obstacles(4)];
    D4 = [obstacles(1),obstacles(4)];
    
    % Check if path from v1 to v2 intersects any of the four edges of the
    % obstacle
    
    isIntersected1 = collisionCheck(A,C1,D1) ~= collisionCheck(B,C1,D1) && collisionCheck(A,B,C1) ~= collisionCheck(A,B,D1); 
    isIntersected2 = collisionCheck(A,C2,D2) ~= collisionCheck(B,C2,D2) && collisionCheck(A,B,C2) ~= collisionCheck(A,B,D2);
    isIntersected3 = collisionCheck(A,C3,D3) ~= collisionCheck(B,C3,D3) && collisionCheck(A,B,C3) ~= collisionCheck(A,B,D3);
    isIntersected4 = collisionCheck(A,C4,D4) ~= collisionCheck(B,C4,D4) && collisionCheck(A,B,C4) ~= collisionCheck(A,B,D4);
    if isIntersected1==0 && isIntersected2==0 && isIntersected3==0 && isIntersected4==0
        noCollision = 1;
    else
        noCollision = 0;
    end
end