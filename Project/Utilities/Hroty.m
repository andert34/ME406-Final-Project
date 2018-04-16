function H = Hroty(theta)
	H = zeros(4);
	H(1:3, 1:3) = [ cos(theta) 0           sin(theta);
                    0          1          0          ;
                   -sin(theta) 0          cos(theta);];
	H(1:3, 4) = [0 0 0]';
	H(4, 1:3) = [0 0 0];
	H(4, 4) = 1;
end
