function [new_matrix] = shift_matrix(A, row, col)
%SHIFT_MAP Summary of this function goes here
%   Detailed explanation goes here
	new_matrix = zeros(size(A));
	
	if((row > 0) && (col > 0))
		new_matrix = [A(row:end, col:end), A(row:end, 1:col);
					  A(1:row, col:end), A()];
	end
end

