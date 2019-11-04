function seen = canSee(CS, guard1, guard2)
    % This part uses a Matlab optmized version of Bresenham line algorithm.
    % (No loops) created by Aaron Wetzler
    % https://www.mathworks.com/matlabcentral/fileexchange/28190-bresenham-optimized-for-matlab
    guard1(1)=round(guard1(1)); guard2(1)=round(guard2(1));
    guard1(2)=round(guard1(2)); guard2(2)=round(guard2(2));
    dx=abs(guard2(1)-guard1(1));
    dy=abs(guard2(2)-guard1(2));
    steep=(abs(dy)>abs(dx));
    %if the line is steep (greater than 45) swap dx and dy
    if steep
        t=dx;
        dx=dy;
        dy=t;
    end
    %The main algorithm goes here.
    if dy==0 
        q=zeros(dx+1,1);
    else
        q=[0;diff(mod([floor(dx/2):-dy:-dy*dx+floor(dx/2)]',dx))>=0];
    end
    %and ends here.
    if steep
        if guard1(2)<=guard2(2) 
            y=[guard1(2):guard2(2)]'; 
        else
            y=[guard1(2):-1:guard2(2)]';
        end
        if guard1(1)<=guard2(1) 
            x=guard1(1)+cumsum(q);
        else
            x=guard1(1)-cumsum(q); 
        end
    else
        if guard1(1)<=guard2(1) 
            x=[guard1(1):guard2(1)]'; 
        else
            x=[guard1(1):-1:guard2(1)]'; 
        end
        if guard1(2)<=guard2(2) 
            y=guard1(2)+cumsum(q);
        else
            y=guard1(2)-cumsum(q);
        end
    end
    
    %check every pt if it is an obstacle
    i = 1;
    is_obstacle = false;
    while ((i <= length(x)) && (is_obstacle == false))
        if (CS(x(i),y(i)) == 1)
            is_obstacle = true;
        end
        i = i + 1;
    end
    if (is_obstacle == false)
        seen = true;
    else
        seen = false;
    end
return;
end