function [I] = InertiaMatrix(Ixx, Ixy, Ixz, Iyy, Iyz, Izz)
%InertiaMatrix Builds inertia matrix from minimal set of parameters
I = [Ixx, Ixy, Ixz; 
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
end

