clc;
clear
c=1;
n=0;
for kp =0.1:0.1:5
    for ka= kp:0.1:5
        for kb =-5:0.1:-0.1
            A=[-kp     0      0;...
                0    kp-ka   -kb;...
                0     -kp     0];
            ei=eig(A);
            if all(real(ei)<0)
            val(c,:)= [kp, ka, kb];
            
            
            poles(c,:)=ei';
            c=c+1;
            n=n+1;
            end
        end
    end
end




