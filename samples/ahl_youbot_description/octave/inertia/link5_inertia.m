% Frame p is the frame attached to p and its orientation is along
% principal axis
% Frame c is the frame attached to com, however it's rotation matrix
% is coincident with link frame
% Frame l is the frame attached to link, however it's rotation matrix
% is coincident with link frame
% Frame o is the frame attached to link, however it's rotation matrix
% is coincident with identity matrix

I = [1 0 0; 0 1 0; 0 0 1];
M = 0.687
% Inertia w.r.t frame p
Ip = [0.0001934         0         0; ...
              0 0.0001602         0; ...
              0         0 0.0000689];
% COM w.r.t frame l
Pl = [0; 0.0012; -0.01648];

rad = 0 / 180 * pi;
Rz = [cos(rad) -sin(rad) 0; ...
      sin(rad)  cos(rad) 0; ...
            0        0 1];
rad = 90 / 180 * pi;
Ry = [ cos(rad) 0 sin(rad); ...
              0 1        0; ...
      -sin(rad) 0 cos(rad)];
rad = 0 / 180 * pi;
Rx = [1        0         0; ...
      0 cos(rad) -sin(rad); ...
      0 sin(rad)  cos(rad)];

% Rotation matrix representing orientation of frame p w.r.t frame c and frame l.
% Both are the same.
Rcp = Rz * Ry * Rx;
Rlp = Rz * Ry * Rx;
Ic = Rcp * Ip * Rcp.';
% Inertia matrix w.r.t frame l
Il = Ic + M * (Pl.' * Pl * I - Pl * Pl.');

rad = 180 / 180 * pi;
Rz = [cos(rad) -sin(rad) 0; ...
      sin(rad)  cos(rad) 0; ...
            0        0 1];
rad = 0 / 180 * pi;
Ry = [ cos(rad) 0 sin(rad); ...
              0 1        0; ...
      -sin(rad) 0 cos(rad)];
rad = 0 / 180 * pi;
Rx = [1        0         0; ...
      0 cos(rad) -sin(rad); ...
      0 sin(rad)  cos(rad)];
% Rotation matrix representing orientation of frame l w.r.t frame o.
Rol = Rz * Ry * Rx;

% Inertia matrix w.r.t frame o
Io = Rol * Il * Rol.';
% Inertia matrix w.r.t frame c
Ic = Rol * Ic * Rol.';

% Frame l's orientation is not coincident with frame o, whose rotation
% matrix is identity.

Ip
Ic
Io
