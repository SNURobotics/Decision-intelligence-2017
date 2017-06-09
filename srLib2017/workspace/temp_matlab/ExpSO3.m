function R = ExpSO3(w)
th = norm(w);
skew_what = skew(w/th);
R = eye(3) + sin(th)*skew_what + (1-cos(th))*skew_what*skew_what;