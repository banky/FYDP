function [a,b,ptsStore] = polygonal_world(posMinBound, posMaxBound, minLen, maxLen, numObsts, startPos, endPosns, obst_buffer, max_tries)
    % Creates an environment of rectangles that don't overlap and are
    % within a certain bound. Also, none of the rectangles can be ontop of the
    % start position or end positions (waypoints and plastic)

    a = zeros(numObsts,4,2);
    b = zeros(numObsts,4);

    for j = 1:1:numObsts
        a(j,1,:) = [0 -1];
        a(j,2,:) = [1 0];
        a(j,3,:) = [0 1];
        a(j,4,:) = [-1 0];
    end

    % Create the number of obsts
    count = 0;
    for i = 1:1:numObsts
        % Loop while there are collisions with obstacles
        while(1)
            % Generate random positions and lengths
            pos(i,:) = posMinBound + [rand(1)*(posMaxBound(1)-posMinBound(1)),rand(1)*(posMaxBound(2)-posMinBound(2))];
            len(i,:) = [rand(1)*(maxLen.a-minLen.a)+minLen.a,rand(1)*(maxLen.b-minLen.b)+minLen.b];
            fake_len(i,:) = len(i,:) + obst_buffer;
            theta(i) = rand(1)*pi;

            rotationMatrix = [cos(theta(i)) sin(theta(i)); -sin(theta(i)) cos(theta(i))];
            % Find the points
            pts = [-len(i,1)/2, -len(i,2)/2; len(i,1)/2, -len(i,2)/2; ...
                len(i,1)/2, len(i,2)/2; -len(i,1)/2, len(i,2)/2;];
            fake_pts = [-fake_len(i,1)/2, -fake_len(i,2)/2; fake_len(i,1)/2, -fake_len(i,2)/2; ...
                fake_len(i,1)/2, fake_len(i,2)/2; -fake_len(i,1)/2, fake_len(i,2)/2;];
            for j = 1:1:4
                pts(j,:) = (rotationMatrix*(pts(j,:)'))' + [pos(i,1) pos(i,2)];
                fake_pts(j,:) = (rotationMatrix*(fake_pts(j,:)'))' + [pos(i,1) pos(i,2)];
            end        
        
            % Check to see if it is outside the region
            if(min(fake_pts(:,1)) <= posMinBound(1) || max(fake_pts(:,1)) >= posMaxBound(1) || min(fake_pts(:,2)) <= posMinBound(2) || max(fake_pts(:,2)) >= posMaxBound(2))
                continue;
            end
            
            % Check to see if it is on top of the start position
            if(min(pts(:,1)) < startPos(1) && max(pts(:,1)) > startPos(1) && min(pts(:,2)) < startPos(2) && max(pts(:,2)) > startPos(2))
                continue;
            end

            % Check to see if it is on top of any of the end postions
            waypoint_collision = 0;
            for j = 1:length(endPosns)
                endPos = endPosns(j,:);
                if(min(pts(:,1)) < endPos(1) && max(pts(:,1)) > endPos(1) && min(pts(:,2)) < endPos(2) && max(pts(:,2)) > endPos(2))
                    waypoint_collision = 1
                    break;
                end
            end

            if(waypoint_collision)
                continue;
            end

            % Check to see if it collided with any of the other obstacles
            collided = 0;
            for j = 1:1:(i-1)
                if(polygons_overlap(fake_pts, ptsStore(:,(j-1)*2+1:(j-1)*2+2)))
                    collided = 1;
                    break;
                end
            end

            if(~collided)
                break;
            end

            count = count + 1;
            if (count >= max_tries)
                a = [];
                b = [];
                ptsStore = [];
                return;
            end
        end

        ptsStore(:,(i-1)*2+1:(i-1)*2+2) = pts;

        for j = 1:1:4
            next = j+1;

            if(j == 4)
                next = 1;
            end
            temp = [-(pts(j,2) - pts(next,2)); (pts(j,1) - pts(next,1))];
            temp = temp/norm(temp);
            a(i,j,1) = temp(1);
            a(i,j,2) = temp(2);
        end

        % Calculate the b matrix
        for k = 1:1:4
            for j = 1:1:2
                b(i,k) = b(i,k) + pts(k,j)*a(i,k,j);
            end
        end
    end


    %calculate the points of each rectangle
    for i = 1:1:numObsts
        pts = zeros(4,2);
        for j = 1:1:4
            if(j ~= 4)
                a1_1 = a(i,j,1);
                a1_2 = a(i,j,2);
                a2_1 = a(i,j+1,1);
                a2_2 = a(i,j+1,2);

                b1 = b(i,j);
                b2 = b(i,j+1);
            else
                a1_1 = a(i,j,1);
                a1_2 = a(i,j,2);
                a2_1 = a(i,1,1);
                a2_2 = a(i,1,2);

                b1 = b(i,j);
                b2 = b(i,1);
            end
            pts(j,:) = [-(a1_2*b2 - b1*a2_2)/(a1_1*a2_2 - a2_1*a1_2),   (a1_1*b2 - a2_1*b1)/(a1_1*a2_2 - a2_1*a1_2)];
        end
        ptsStore(:,(i-1)*2+1:(i-1)*2+2) = pts;
    end



