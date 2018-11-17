function[V, gV] = gen_potential_field(X, Y, end_pos, num_obsts, obs_centroid, obs_pts_store)
    % Generates a potential field for the given map and end position
    % Returns the complete potential field (V) and gradient (gV)

    K_att = 0.2; % Attractive potential gain
    K_rep = 1000; % Repulsive potential gain
    r0 = 0.5; % Radius of Repulsion
    rc0 = 4;
    Vmax = 50; % Upper bound on potential
    gVmax = 10;
    gVmin = -10;

    % Calculate potential field at each grid point
    V = zeros(size(X));
    gV = zeros(2,size(X));

    for i=1:length(X(:,1))
        for j=1:length(Y(1,:))
            % Current robot position
            pos = [X(i,j) Y(i,j)];
            % Attractive Potential
            V(i,j) = 1/2*Katt*norm(pos-end_pos)^2; % potential field
            gV(:,i,j) = Katt*(pos-end_pos);  % gradient of field

            % Repulsive potentials
            for k=1:num_obsts
                curobs = obs_pts_store(:,2*(k-1)+1:2*k);
                if (inpolygon(pos(1),pos(2),curobs(:,1),curobs(:,2)))
                    V(i,j) = Vmax;
                    gV(:,i,j) = [NaN NaN];
                else
                    % Find potential based on minimum distance to obstacle
                    curpoly = [curobs curobs([2:end, 1],:)];
                    [minD,minPt, d, pt, ind] = min_dist_to_edges(pos, curpoly);
                    if (minD < r0)
                        V(i,j) = V(i,j) + 1/2*Krep*(1/minD-1/r0)^2;
                        gV(:,i,j) = gV(:,i,j) + Krep*(-1/minD+1/r0)*(pos'-minPt')/minD^(3);
                    end
                    % Add potential of distance to center, to avoid getting
                    % stuck on flat walls
                    centD = norm(pos-obsCentroid(k,:));
                    if (centD < rc0)
                        V(i,j) =  V(i,j) + 1/2*Krep*(1/centD-1/rc0)^2;
                        gV(:,i,j) = gV(:,i,j) + Krep*(-1/centD+1/rc0)*(pos'-obsCentroid(k,:)')/centD^(3);
                    end
                end
            end
            V(i,j) = max(0,min(V(i,j),Vmax));
            gV(1,i,j) = max(gVmin,min(gV(1,i,j),gVmax));
            gV(2,i,j) = max(gVmin,min(gV(2,i,j),gVmax));
        end
    end


