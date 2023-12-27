function [bTi] = GetTransformationWrtBase(biTei, linkNumber)
%%% GetTransformatioWrtBase function
% inputs :
% biTei vector of matrices containing the transformation matrices from link i to link i +1 for the current joint position q.
% The size of biTei is equal to (4,4,numberOfLinks)
% linkNumber for which computing the transformation matrix
% outputs
% bTi : transformation matrix from the manipulator base to the ith joint in
% the configuration identified by biTei.

% linkNumber is the link to whitch i want to compute the transformation
% matrix
for i = 1:linkNumber
    if i == 1
        % only for the first iteration
        bTi = biTei(:, :, i);
    else
        bTi = bTi * biTei(:, :, i);
    end
end

end