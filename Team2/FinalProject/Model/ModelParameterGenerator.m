syms x y u theta phi psi delta
syms h Delta_t

%% bicycle to camera DH tables
table = [-90  0   0  phi-pi/2;
         -90  h   0  theta;
          90  0   0  0];

% From frame 0 to bicycle frame
mbT0 = [0, 1, 0, 0;
        1, 0, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];

% From camera frame to frame 3
m3Tc = [0, -1, 0, 0;
        -1, 0, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];

%% Solve for DH table
si = height(table) + 1;
T = sym('T', [si, 4, 4]);

for i = 1:height(table)
    joint = table(i, :);
    T(i, :, :) = get_T(deg2rad(joint(1)), joint(2), joint(3), joint(4));
end

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
mbTc = simplify(mbT0 * reshape(T(si, :, :), [4, 4]) * m3Tc)

fprintf("Latex form:\n%s\n", latex(mbTc));

%% Solve for measurement
Rz = [cos(psi) -sin(psi) 0;
      sin(psi) cos(psi) 0;
      0 0 1];

mmRc = simplify(Rz * mbTc(1:3, 1:3));
fprintf('Solve for measurement:\n%s\n', latex(mmRc))

%% System model - Transition 

syms a L beta g

states = [x, y, u, theta, phi, psi, delta];
fprintf("State:\n%s\n", latex(states.'));

beta = atan2(tan(delta), 2);
F = [x + u * cos(beta + psi) * Delta_t, 
     y + u * sin(beta + psi) * Delta_t, 
     u, 
     theta, 
     phi,
     psi + (u * sin(beta) / (L/2)) * Delta_t, 
     delta]; %atan(((v^2) * tan(delta)) / g / L)
fprintf("Transition:\n%s\n", latex(F));

dFdX = simplify(jacobian(F, states))
fprintf("Latex form:\n%s\n", latex(simplify(dFdX)));

%% System model - Observation
syms zeta
mbRw = [cos(psi) -sin(psi) 0;
        sin(psi)  cos(psi) 0;
        0         0        1];
mwRm = [cos(zeta) -sin(zeta) 0;
        sin(zeta)  cos(zeta) 0;
        0          0         1];
mcTm = simplify((mbTc(1:3, 1:3)).' * simplify(mbRw * mwRm))
fprintf("Latex: \n%s\n", latex(mcTm));

H = [x;
     y;
     theta;
     phi;
     psi];
fprintf("Observeration: \n%s\n", latex(H));
 
dHdX = jacobian(H, states)
fprintf("Latex: \n%s\n", latex(dHdX));

%% Process noise (Q)
syms sigma_x sigma_y sigma_psi sigma_v sigma_delta sigma_theta sigma_phi
Qa = diag([0, 0, sigma_v, sigma_theta, sigma_phi, sigma_psi, sigma_delta]);
out = dFdX * Qa * dFdX.'

