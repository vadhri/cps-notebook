% Find rotation matrix R such that (R-AB)^2 is minimized.

A = [1 1 -1 -1;1 -1 -1 1;1 1 1 1];
B = [
   -1.2131,  0.0851, -1.2334;
   -1.4413, -0.7858,  0.5525;
    0.3470, -1.6594,  0.3550;
    0.5752, -0.7885, -1.4309
];

B = transpose(B);

[u d v] = svd(B*transpose(A));

R = v*transpose(u)

