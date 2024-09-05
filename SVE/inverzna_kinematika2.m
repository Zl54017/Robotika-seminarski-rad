
targetTform = trvec2tform(targetPos);

[configSol, solInfo] = ik('endeffector', targetTform, weights, initialguess);

configSol = rad2deg(configSol);
%rad2deg(configSol)