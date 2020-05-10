function initMaskPortKF(enableP,enableG,kindKF)

% Mask Display
if strcmp(kindKF,'KF')
    MaskDisplay = ['fprintf(''Kamlman Filter'')' char(10) ...
         'port_label(''Output'', ' num2str(1) ', ''xhat'')' char(10) ...
         'port_label(''Input'', ' num2str(1) ', ''Input'')' char(10) ...
         'port_label(''Input'', ' num2str(2) ', ''Output'')'];
elseif strcmp(kindKF,'EKF')
     MaskDisplay = ['fprintf(''Extended\n Kalman Filter'')' char(10) ...
         'port_label(''Output'', ' num2str(1) ', ''xhat'')' char(10) ...
         'port_label(''Input'', ' num2str(1) ', ''Input'')' char(10) ...
         'port_label(''Input'', ' num2str(2) ', ''Output'')'];
end

% For P Output port
if ~enableP
    % Disable the port
    try
        lh = get_param([gcb, '/P'],'LineHandles');
        position = get_param([gcb, '/P'],'Position');
        delete_line(lh.Inport(1));
        delete_block([gcb,'/P']);
        h = add_block('built-in/Terminator',[gcb,'/Terminator_P']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb, '/Signal Specification P'],'PortHandles');
        add_line(gcb,ph2.Outport(1),ph1.Inport(1));
    catch
    end
else
    % Enable the port
    try
        lh = get_param([gcb '/Terminator_P'],'LineHandles');
        position = get_param([gcb, '/Terminator_P'],'Position');
        delete_line(lh.Inport(1));
        delete_block([gcb, '/Terminator_P']);
        h = add_block('built-in/Outport',[gcb, '/P']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb,'/Signal Specification P'],'PortHandles');
        add_line(gcb,ph2.Outport(1),ph1.Inport(1));
    catch
    end
end

% For G Output port
if ~enableG
    % Disable the port
    try
        lh = get_param([gcb,'/G'],'LineHandles');
        position = get_param([gcb, '/G'],'Position');
        delete_line(lh.Inport(1));
        delete_block([gcb, '/G']);
        h = add_block('built-in/Terminator',[gcb,'/Terminator_G']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb,'/Signal Specification G'],'PortHandles');
        add_line(gcb,ph2.Outport(1),ph1.Inport(1));
    catch
    end
else
    % Enable the port
    try
        lh = get_param([gcb,'/Terminator_G'],'LineHandles');
        position = get_param([gcb, '/Terminator_G'],'Position');
        delete_line(lh.Inport(1));
        delete_block([gcb,'/Terminator_G']);
        h = add_block('built-in/Outport',[gcb,'/G']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb,'/Signal Specification G'],'PortHandles');
        add_line(gcb,ph2.Outport(1),ph1.Inport(1));
    catch
    end
end

% Sort order of Outport
if enableP
    set_param([gcb,'/P'],'Port','2');
    MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(2) ', ''P'')'];
    if enableG
        set_param([gcb,'/G'],'Port','3');
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(3) ', ''G'')'];
    end
elseif enableG
    set_param([gcb,'/G'],'Port','2');
    MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(2) ', ''G'')'];
end

% Update MaskDisplay
set_param(gcb,'MaskDisplay',MaskDisplay);

end