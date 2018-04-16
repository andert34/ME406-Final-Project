function H = Htrans(r1, r2, r3)
% 	H = zeros(4);
	H(1:3, 1:3) = eye(3);
	H(1:3, 4) = [r1, r2, r3]';
	H(4, 1:3) = [0 0 0];
	H(4, 4) = 1;
end