function [CInv] = SE3Inv(C)
[CInv] = [C(1,1),C(2,1),C(3,1), -C(1,1)*C(1,4)-C(2,1)*C(2,4)-C(3,1)*C(3,4);
C(1,2),C(2,2),C(3,2) -C(1,2)*C(1,4)-C(2,2)*C(2,4)-C(3,2)*C(3,4);
C(1,3),C(2,3),C(3,3) -C(1,3)*C(1,4)-C(2,3)*C(2,4)-C(3,3)*C(3,4);
0,0,0,1];
end

