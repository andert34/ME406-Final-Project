function H = Hrotx(theta)
	H = zeros(4);
	H(1:3, 1:3) = [ 1          0           0;
                    0          cos(theta) -sin(theta);
                    0          sin(theta)  cos(theta);];
	H(1:3, 4) = [0 0 0]';
	H(4, 1:3) = [0 0 0];
	H(4, 4) = 1;
end