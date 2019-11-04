function CS = fillInLine(CS, index1, index2, filling)
    % This part uses a Matlab optmized version of Bresenham line algorithm.
    % (No loops) created by Aaron Wetzler
    % https://www.mathworks.com/matlabcentral/fileexchange/28190-bresenham-optimized-for-matlab
    index1(1)=round(index1(1)); index2(1)=round(index2(1));
    index1(2)=round(index1(2)); index2(2)=round(index2(2));
    dx=abs(index2(1)-index1(1));
    dy=abs(index2(2)-index1(2));
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
        if index1(2)<=index2(2) 
            y=[index1(2):index2(2)]'; 
        else
            y=[index1(2):-1:index2(2)]';
        end
        if index1(1)<=index2(1) 
            x=index1(1)+cumsum(q);
        else
            x=index1(1)-cumsum(q); 
        end
    else
        if index1(1)<=index2(1) 
            x=[index1(1):index2(1)]'; 
        else
            x=[index1(1):-1:index2(1)]'; 
        end
        if index1(2)<=index2(2) 
            y=index1(2)+cumsum(q);
        else
            y=index1(2)-cumsum(q);
        end
    end
    
    %fill in every point discovered
    for i = 1:length(x)
        CS(x(i),y(i)) = filling;
    end

return;
end