syms psi h theta a

table = [0 0 0 psi;
         0 0 573+h 0;
         -pi/2 0 0 -pi/2+theta;
         pi/2 0 508 0];

T = sym('T', [6, 4, 4]);
for i = 1:height(table)
    joint = table(i, :);
    T(i, :, :) = get_T(joint(1), joint(2), joint(3), joint(4));
end

si = height(table) + 1;
T(si, :, :) = reshape(T(1, :, :), [4, 4]);
for i = 2:height(table)
    T(si, :, :) = reshape(T(si, :, :), [4, 4]) * reshape(T(i, :, :), [4, 4]);
    %reshape(T(i, :, :), [4, 4])
end

% for i = 1:length(table')
%     fprintf("%d:\n", i);
%     fprintf("%s\n", latex(reshape(T(i, :, :), [4, 4])))
% end

% Frome camera frame to bicycle frame
fprintf("From camera frame to bicycle frame\n");
mrTc = simplify(reshape(T(si, :, :), [4, 4]))
latex(mrTc)