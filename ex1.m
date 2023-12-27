%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix
addpath('include')

% The same model of assignment 2
geom_model = BuildTree();
numberOfLinks = size(geom_model,3); % number of manipulator's links.
linkType = zeros(numberOfLinks); % specify two possible link type: Rotational, Prismatic.
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base

% Initial joint configuration 
q(1,:,1) = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];
q(1,:,2) = [ 1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8];
q(1,:,3) = [ 0.3, 1.4, 0.1, 2.0, 0, 1.3, 0];
q(1,:,4) = [ 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.];
q(1,:,5) = [1,1,1,1,1,1,1];

%% Compute direct geometry

% Compute the transformation w.r.t. the base
% biTei vector of matrices containing the transformation matrices from base to the next linkr for the input q. 
for j = 1:size(q,3)
    biTei = GetDirectGeometry(q(1,:,j), geom_model, linkType);
    % bTi vector of matrices from the base and the i-esimo joint
    for i = 1:numberOfLinks
        bTi(:,:,i)= GetTransformationWrtBase(biTei, i);
    end    
    % Computing end effector jacobian 
    [J(:,:,j)] = GetJacobian(bTi, linkType, numberOfLinks);
end




