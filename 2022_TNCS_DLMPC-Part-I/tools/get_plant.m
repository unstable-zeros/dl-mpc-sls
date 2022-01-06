function [sys, adjMtx, nodeCoords] = get_plant(gridSize, seed, connectThresh)

numNodes       = gridSize * gridSize;

% Actuation
actDens        = 1;
numActs        = round(actDens*numNodes);
actuatedNodes  = randsample(numNodes, numActs);

% Sampling time
Ts = 0.2; 

[adjMtx, nodeCoords, susceptMtx, inertiasInv, dampings] = generate_grid_topology(gridSize, connectThresh, seed);
sys = generate_grid_plant(actuatedNodes, adjMtx, susceptMtx, inertiasInv, dampings, Ts);

end