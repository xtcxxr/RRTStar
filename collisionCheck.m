function val = collisionCheck(A,B,C)
%cross product for collision check
%input: three points A,B,C
%output: 1 for true or 0 for false 
    val = (C(2)-A(2)) * (B(1)-A(1)) > (B(2)-A(2)) * (C(1)-A(1));
end