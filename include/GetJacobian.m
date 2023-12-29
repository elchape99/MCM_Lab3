function [J] = GetJacobian(bTi, linkType, numberOfLinks)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTi.
%
% Inputs:
% - bTi: trasformation matrix from <base> to <i-frme> 
% - linkType: vector identifying the joint type, 0 = revolute, 1 = prismatic
% - numberOfLinks: the total number of robot links
%
% Output:
% - J: end-effector jacobian matrix


% compute the angular velocity jacobian
Ja = zeros(3, numberOfLinks);
for i = 1:numberOfLinks
    if linkType(i) == 0 % revolute joint
        % extract the k vector for all joint
        Ja(:, i) = bTi (1:3, 3, i); 
    end
    % if the joint is prismatic don't change, it remain zeros
end
% Ja

% compute the linear jacobian matrix
% - first i extract the vrctor bre, vector distance of end-eff. w.r.t. base
bre = bTi(1:3, 4, numberOfLinks);
ire = zeros(3, numberOfLinks);

% calculate ire: distance vector of end-effectors w.r.t i-frame 
for j = 1:numberOfLinks 
    ire(:,j) = bre - bTi(1:3, 4, j);
end

% now can compte the linear jacobian matrix:
% for prismatic is equl to k1
% for revoloute is equal to (ki x eri)
Jl = zeros(3, numberOfLinks);
for z = 1:numberOfLinks 
    if linkType(z) == 0 % if revoloute joint (ki x eri)
        Jl(:, z) = cross(Ja(:, z),ire(:, z));  

    elseif linkType(z) == 1 % if prsmatic joint (ki)
        Jl(:, z) = bTi (1:3, 3, z);
    end    
end
J = [Ja; Jl];
end