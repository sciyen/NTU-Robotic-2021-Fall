syms s Ki Kb L Ra Jm Bm
syms pid_ks pid_kv 

Pc = (1/(s*L + Ra)) * Ki * (1/(s*Jm + Bm));

cltf_c = simplify(Pc / (1 + Kb * Pc));

C = (pid_ks + s * pid_kv);

cltf_v = simplify(C * cltf_c / (1 + C * cltf_c));
