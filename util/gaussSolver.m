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
        % i 列目のうち，最大のものを探す
        [~, id] = max(abs(A(i:n, i)));

        if id ~= 1
            pivot = id + i - 1;
            % 行の入れ替え
            tmpVec = A(i, :);
            A(i, :) = A(pivot, :);
            A(pivot, :) = tmpVec;

            % 右辺ベクトルの入れ替え
            tmpScalar = b(i);
            b(i) = b(pivot);
            b(pivot) = tmpScalar;
        end
        diagVal = A(i, i);

        % 対角成分が eps よりも小さい場合は，エラー処理
%         if (abs(diagVal) < eps)
%             error('Error: diagonal component is less than eps=%.15e\n', eps);
%         end

        % ========
        % 前進消去
        % ========
        for row = (i + 1):n
            idList = i:n;
            weight = A(row, i) / diagVal;
            A(row, idList) = A(row, idList) - weight * A(i, idList);
            b(row) = b(row) - weight * b(i);
        end
    end

    % ========
    % 後退代入
    % ========
    b(n) = b(n) / A(n, n);
    for row = (n - 1):(-1):1
        idList = (row + 1):n;
        b(row) = (b(row) - A(row, idList) * b(idList)) / A(row, row);
    end
    x = b;
end