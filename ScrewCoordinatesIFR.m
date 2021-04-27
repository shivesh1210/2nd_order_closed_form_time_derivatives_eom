function [Y] = ScrewCoordinatesIFR(y,e,h)
%ScrewCoordinatesIFR Computes the screw coordinates vector in the inertial
%frame of reference(IFR)
%   y: position vector of point on the joint axis in IFR
%   e: unit axis vector in IFR
%   h: pitch of the screw (h = 0 for revolute joint, h = inf for prismatic joint)

if nargin == 2   % if the number of inputs equals 2
    h=0;
    disp('screw pitch value (h) was not provided. Assuming h = 0 i.e. revolute joint');
end

if h==0
    Y = [e, cross(y,e)]';
elseif h == inf
    Y = [e, zeros(1,3)]';
else
    Y = [e, cross(y,e)+h*e]';    
end

end

