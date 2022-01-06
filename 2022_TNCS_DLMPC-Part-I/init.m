% add current directory and all subdirectories
addpath(genpath(pwd));

disp('The SLS-MATLAB directory')
disp(['    ' pwd])
disp('has been added to the MATLAB path.')
disp('To use SLS-MATLAB regularly, save the new path definition.');
disp('You can do this by typing the command' );
disp('    savepath' );