function notCollided = notCollided(v1,v2,obstacles)

notCollided = 1;
sz = size(obstacles,1);
for i=1:1:sz
    notCollided = noCollision(v1,v2,obstacles(i,:));
    if notCollided == 0
        break;
    end
end

end