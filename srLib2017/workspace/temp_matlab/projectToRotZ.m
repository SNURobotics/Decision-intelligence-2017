function R = projectToRotZ(input)

input_z = input(:,3);

w = cross(input_z, [0;0;1]);
th = acos(input_z(3));

R = ExpSO3(w/norm(w)*th) * input;