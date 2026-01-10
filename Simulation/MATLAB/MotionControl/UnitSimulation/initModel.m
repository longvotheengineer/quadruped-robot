function leg_model = initModel(robot_config)
    L  = robot_config.robot_length.base_length;
    W  = robot_config.robot_length.base_width;
    L1 = robot_config.robot_length.L1;
    L2 = robot_config.robot_length.L2;
    L3 = robot_config.robot_length.L3;
    
    switch robot_config.leg_type
        case {"left-front", "left-behind"}
            link1 = Link([0  0  0     0], 'modified');
            link2 = Link([0 L1  0  pi/2], 'modified');
            link3 = Link([0  0 L2     0], 'modified');
        case {"right-front", "right-behind"}
            link1 = Link([0  0  0     0], 'modified');
            link2 = Link([0 L1  0 -pi/2], 'modified');
            link3 = Link([0  0 L2     0], 'modified');
        otherwise
    end
        
    leg      = SerialLink([link1 link2 link3]);
    leg.tool = transl(L3, 0, 0);
    leg.base = trotz(-pi) * troty(-pi/2) * trotx(0);
    switch robot_config.leg_type
        case "left-front"
            leg.base = transl( L/2,  W/2, 0) * leg.base;
        case "left-behind"
            leg.base = transl(-L/2,  W/2, 0) * leg.base;
        case "right-front"
            leg.base = transl( L/2, -W/2, 0) * leg.base;
        case "right-behind"
            leg.base = transl(-L/2, -W/2, 0) * leg.base;
        otherwise
    end
    
    leg_model = leg;
end