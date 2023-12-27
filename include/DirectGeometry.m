function iTj_q = DirectGeometry(qi, iTj, linkType)
% DirectGeometry Function 
% inputs: 
% qi : current joint position;
% iTj is the constant transformation between the base of the link <i>
% and its follower frame <j>; 
% jointType :0 for revolute, 1 for prismatic

% output :
% iTj_q : transformation between the base of the joint <i> and its follower frame taking 
% into account the actual rotation/traslation of the joint

% generic rotation around Z axis, defined as default rotation axis
Rz = [cos(qi), -sin(qi),     0;
      sin(qi),  cos(qi),     0;
        0,        0,         1];
        

if linkType == 0 % if the passed link is rotational
    iri_1 = iTj(1:3, 4); % lenght of link between joint i and i-1
    iRi_1 = iTj(1:3, 1:3) * Rz; % rotation
    
    block1 = [iRi_1; 0 0 0];
    block2 = [iri_1; 1];

    iTj_q = [ block1, block2 ]; % creation of 4 by 4 trasformation matrix

elseif linkType == 1 % if the passed link is prismatic
    iri_1 = iTj(1:3, 4) + [0;0;1] * qi;% traslation along Z axis
    iRi_1 = iTJ(1:3, 1:3); % orientation of the frame
    
    block1 = [iRi_1; 0 0 0];
    block2 = [iri_1, 1];

    iTj_q = [ block1, block2 ];% creation of 4 by 4 trasformation matrix
end

end