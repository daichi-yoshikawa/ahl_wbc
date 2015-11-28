% World to base
Pb  = [0 0 0.084];
Pab = [0.143 0 0.0142];
Pl1 = [0.167 0 0.0921];
Pl2 = [0.2 0 0.161];
Pl3 = [0.2 0 0.316];
Pl4 = [0.2 0 0.451];
Pl5 = [0.2 0 0.535];
Pg  = [0.2 0 0.5567];
Pop = [0.2 0 0.6315];

Pop -= Pg;
Pg  -= Pl5;
Pl5 -= Pl4;
Pl4 -= Pl3;
Pl3 -= Pl2;
Pl2 -= Pl1;
Pl1 += Pab;

Pop += Pg;

Pab
Pl1
Pl2
Pl3
Pl4
Pl5
Pg
Pop
