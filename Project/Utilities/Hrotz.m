function H = Hrotz(theta)
%     if isa(theta,'sym')
%         H = 
%     elseif
%         H = zeros(4);
%     end
	H(1:3, 1:3) = [ cos(theta) -sin(theta) 0            ;
                    sin(theta) cos(theta)  0            ;
                    0           0           1          ];

	H(4, 1:3) = [0 0 0];
	H(1:3, 4) = [0 0 0]';
	H(4, 4) = 1;
end
