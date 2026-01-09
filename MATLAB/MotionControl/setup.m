function setup(action)
%SETUP Initialize project paths and dependencies
%   Run this once when opening the project, or call it from main.m
%
%   Usage:
%       setup()         % Add paths
%       setup('remove') % Remove paths (cleanup)

    arguments
        action (1,1) string {mustBeMember(action, ["add", "remove"])} = "add"
    end

    projectRoot = fileparts(mfilename('fullpath'));
    
    subfolders = {
        'MotionControl'
        'RemoteAPI'
        'UnitSimulation'
        'Navigation'
    };
    
    for i = 1:length(subfolders)
        folderPath = fullfile(projectRoot, subfolders{i});
        if action == "add"
            addpath(folderPath);
        else
            rmpath(folderPath);
        end
    end
    
    if action == "add"
        fprintf('Project paths added successfully.\n');
    else
        fprintf('Project paths removed.\n');
    end
end
