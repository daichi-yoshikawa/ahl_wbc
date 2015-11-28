% Assume that all frames' rotation matrix is identity matrix.

Ml = 0.68700
% Inertia matrix of gripper link in frame attached to its COM
Il_cl = ...
  [0.0001602         0         0; ...
           0 0.0000689         0; ...
           0         0 0.0001934];
% COM of link5 in frame attached to link5
Pl_l = [-0.000903; 0.000638; 0.006573; 1];

% Add grippers weight to gripper link
Mg = 0.19900 + 0.01 + 0.01
% Inertia matrix of gripper link in frame attached to its COM
Ig_cg = ...
  [0.0003629         0         0; ...
           0 0.0002067         0; ...
           0         0 0.0002324];
% COM of gripper link in frame attached to gripper link
Pg_g = [0.002737; 0; 0.2616; 1];

Tlg = ...
  [1 0 0    0; ...
   0 1 0    0; ...
   0 0 1 0.04; ...
   0 0 0    1];

% Pg_l is COM of gripper in frame attached to link5
Pg_l = Tlg * Pg_g;
Pg_l = Pg_l(1:3);

I = [1 0 0; 0 1 0; 0 0 1];
% Ig_ol is inertia matrix w.r.t frame attached to origin of link5
Ig_ol = Ig_cg + Mg * ((Pg_l.' * Pg_l) * I - Pg_l * Pg_l.');

Pl_l = Pl_l(1:3);
% Ig_cl is inertia matrix w.r.t frame attached to COM of link5
M = Ml + Mg
Pc_l = (Ml / M) * Pl_l + (Mg / M) * Pg_l
Il_cl
Ig_cl = Ig_ol + Mg * ((Pc_l.' * Pc_l) * I - Pc_l * Pc_l.')
I_cl = Il_cl + Ig_cl
