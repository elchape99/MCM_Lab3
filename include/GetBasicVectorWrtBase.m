function [r]=GetBasicVectorWrtBase(bTi, linkNumber)
%%% GetBasicVectorWrtBase function 
% input :
% biTei: (4, 4, numberOflinks) -> ogni foglio è la roba sulla roba;
% linkNumber: link number 
% output
% r : basic vector from frame i to the robot base frame <0>

% extraction of the projected vector to be plotted
r = bTi(1:3, 4, linkNumber); 

end