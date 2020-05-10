%===================================%
%                                   %
% gaussSolver                       %
%                                   %
% A: coefficient matrix, A(row,col) %
% b: right side vector,  b(row)     %
%                                   %
% return: solution vector x         %
%===================================%
function [x] = gaussSolver(A,b)
    [n, ~] = size(A);

    for i = 1:(n - 1)
        % i ��ڂ̂����C�ő�̂��̂�T��
        [~, id] = max(abs(A(i:n, i)));

        if id ~= 1
            pivot = id + i - 1;
            % �s�̓���ւ�
            tmpVec = A(i, :);
            A(i, :) = A(pivot, :);
            A(pivot, :) = tmpVec;

            % �E�Ӄx�N�g���̓���ւ�
            tmpScalar = b(i);
            b(i) = b(pivot);
            b(pivot) = tmpScalar;
        end
        diagVal = A(i, i);

        % �Ίp������ eps �����������ꍇ�́C�G���[����
%         if (abs(diagVal) < eps)
%             error('Error: diagonal component is less than eps=%.15e\n', eps);
%         end

        % ========
        % �O�i����
        % ========
        for row = (i + 1):n
            idList = i:n;
            weight = A(row, i) / diagVal;
            A(row, idList) = A(row, idList) - weight * A(i, idList);
            b(row) = b(row) - weight * b(i);
        end
    end

    % ========
    % ��ޑ��
    % ========
    b(n) = b(n) / A(n, n);
    for row = (n - 1):(-1):1
        idList = (row + 1):n;
        b(row) = (b(row) - A(row, idList) * b(idList)) / A(row, row);
    end
    x = b;
end