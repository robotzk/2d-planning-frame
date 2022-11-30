function a0=unit_graph(a)
% Normalization of vectors
% The function gives a unit vector with the same direction as original
% given vector a=[a1 a2 a3].


%%% For example
%% c=-eps^10000;
% c=-.001*eps;
% a=[0 0 c];
% a0=unit_graph(a)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% na=norm(a);
% if na<realmin;
%    na=na+eps;
% end
% a0=a/na;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
na=norm(a);
if na <=eps
    disp('     Warning:  norm(a) = 0 ') 
    a1=nan;
elseif na > eps
    a1=a/na;
end
a0=a1;
end
