function [Uatt,Urep]=APF(columns, rows, x, y, Katt, Krep, obsi,rlim,i)

Uatt=zeros(length(y),length(x));
Urep=zeros(length(y),length(x));

xt=x(2)-x(1);
yt=y(2)-y(1);

xd= x(length(x));
yd= y(length(y));

n=length(obsi);

for col = 1 : columns+1
    for row = 1 : rows+1
        Uatt(columns+2-col,row)=Katt*((x(row)-xd)^2+((y(columns+2-col)-yd)^2))^0.5;
        
        repmax=-inf;
        for k=1:n-1
            %rp=((row-obsi(k,2))^2+(columns+2-col-obsi(k,1))^2)^0.5; %%distance in pixels
            r=(((row-obsi(k,2))*xt)^2+((columns+2-col-obsi(k,1))*yt)^2)^0.5;
            
            if r<rlim % 5 cm
                rep= Krep*((1/r)-(1/rlim))^2;
                
                if rep>repmax && rep~=inf
                    repmax=rep;
                end
                
                Urep(columns+2-col,row)= repmax;
            end
            
        end
        
    end
end

if xt<0 && i==1
    Uatt=fliplr(Uatt);
end

if yt<0 && i==1
    Uatt=flip(Uatt);
end

end