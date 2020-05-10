function numDist = initMaskPortMPC(numDist,enableDist,enableSlack,enableStatus,enableMVseq,kindMPC)

% Mask Display
if strcmp(kindMPC,'MPC')
    MaskDisplay = ['fprintf(''MPC'')' char(10) ...
         'port_label(''Output'', ' num2str(1) ', ''mv'')' char(10) ...
         'port_label(''Input'', ' num2str(1) ', ''state'')' char(10) ...
         'port_label(''Input'', ' num2str(2) ', ''ref'')'];
elseif strcmp(kindMPC,'Adaptive MPC')
     MaskDisplay = ['fprintf(''Adaptive MPC'')' char(10) ...
         'port_label(''Output'', ' num2str(1) ', ''mv'')' char(10) ...
         'port_label(''Input'', ' num2str(1) ', ''state'')' char(10) ...
         'port_label(''Input'', ' num2str(2) ', ''ref'')'];
end

% For Disturbance Input port
if ~enableDist
    % Disable the port
    % Number of disutrbance input
    numDist = 1; % Default value
    try
        lh = get_param([gcb, '/md'],'LineHandles'); % Inport block
        position = get_param([gcb, '/md'],'Position');
        delete_line(lh.Outport(1));
        delete_block([gcb, '/md']);
        h = add_block('built-in/Ground',[gcb,'/Ground_md']);    % Add ground block
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb,'/Signal Specification md'],'PortHandles');
        add_line(gcb,ph1.Outport(1),ph2.Inport(1));
    catch
    end
    
    % Update MaskDisplay
    if strcmp(kindMPC,'Adaptive MPC')
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Input'', ' num2str(3) ', ''sysm'')'];
    end
    
else
    % Enable the port
    try
        lh = get_param([gcb, '/Ground_md'],'LineHandles');
        position = get_param([gcb, '/Ground_md'],'Position');
        delete_line(lh.Outport(1));
        delete_block([gcb,'/Ground_md']);
        h = add_block('built-in/Inport',[gcb, '/md']);
        set_param(h,'position',[position(1) position(2) position(3) position(4)]);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb ,'/Signal Specification md'],'PortHandles');
        add_line(gcb,ph1.Outport(1),ph2.Inport(1));
    catch
    end
    
    % Update MaskDisplay
    MaskDisplay = [MaskDisplay char(10) 'port_label(''Input'', ' num2str(3) ', ''md'')'];
    if strcmp(kindMPC,'Adaptive MPC')
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Input'', ' num2str(4) ', ''sysm'')'];
    end
    
    % Update order of inport
    set_param([gcb,'/md'],'Port','3');
end

% For Slack Output port
if ~enableSlack
    % Disable the port
    try
        lh = get_param([gcb, '/Slack'],'LineHandles');
        position = get_param([gcb, '/Slack'],'Position');
        delete_line(lh.Inport(1));
        delete_block([gcb,'/Slack']);
        h = add_block('built-in/Terminator',[gcb,'/Terminator_Slack']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb, '/Signal Specification slack'],'PortHandles');
        add_line(gcb,ph2.Outport(1),ph1.Inport(1));
    catch
    end
else
    % Enable the port
    try
        lh = get_param([gcb '/Terminator_Slack'],'LineHandles');
        position = get_param([gcb, '/Terminator_Slack'],'Position');
        delete_line(lh.Inport(1));
        delete_block([gcb, '/Terminator_Slack']);
        h = add_block('built-in/Outport',[gcb, '/Slack']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb,'/Signal Specification slack'],'PortHandles');
        add_line(gcb,ph2.Outport(1),ph1.Inport(1));
    catch
    end
end

% For Status Output port
if ~enableStatus
    % Disable the port
    try
        lh = get_param([gcb,'/Status'],'LineHandles');
        position = get_param([gcb, '/Status'],'Position');
        delete_line(lh.Inport(1));
        delete_block([gcb, '/Status']);
        h = add_block('built-in/Terminator',[gcb,'/Terminator_Status']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb,'/Signal Specification status'],'PortHandles');
        add_line(gcb,ph2.Outport(1),ph1.Inport(1));
    catch
    end
else
    % Enable the port
    try
        lh = get_param([gcb,'/Terminator_Status'],'LineHandles');
        position = get_param([gcb, '/Terminator_Status'],'Position');
        delete_line(lh.Inport(1));
        delete_block([gcb,'/Terminator_Status']);
        h = add_block('built-in/Outport',[gcb,'/Status']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb,'/Signal Specification status'],'PortHandles');
        add_line(gcb,ph2.Outport(1),ph1.Inport(1));
    catch
    end
end

% For MVseq Output port
if ~enableMVseq
    % Disable the port
    try
        lh = get_param([gcb,'/MVseq'],'LineHandles');
        position = get_param([gcb, '/MVseq'],'Position');
        delete_line(lh.Inport(1));
        delete_block([gcb,'/MVseq']);
        h = add_block('built-in/Terminator',[gcb,'/Terminator_MVseq']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb,'/Signal Specification MVseq out'],'PortHandles');
        add_line(gcb,ph2.Outport(1),ph1.Inport(1));
    catch
    end
else
    % Enable the port
    try
        lh = get_param([gcb,'/Terminator_MVseq'],'LineHandles');
        position = get_param([gcb, '/Terminator_MVseq'],'Position');
        delete_line(lh.Inport(1));
        delete_block([gcb,'/Terminator_MVseq']);
        h = add_block('built-in/Outport',[gcb,'/MVseq']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb,'/Signal Specification MVseq out'],'PortHandles');
        add_line(gcb,ph2.Outport(1),ph1.Inport(1));
    catch
    end
end

% Sort order of Outport
if enableSlack
    set_param([gcb,'/Slack'],'Port','2');
    MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(2) ', ''slack'')'];
    if enableStatus
        set_param([gcb,'/Status'],'Port','3');
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(3) ', ''status'')'];
        if enableMVseq
            set_param([gcb,'/MVseq'],'Port','4');
            MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(4) ', ''mvseq'')'];
        end
    elseif enableMVseq
        set_param([gcb,'/MVseq'],'Port','3');
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(3) ', ''mvseq'')'];
    end
elseif enableStatus
    set_param([gcb,'/Status'],'Port','2');
    MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(2) ', ''Status'')'];
    if enableMVseq
        set_param([gcb,'/MVseq'],'Port','3');
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(3) ', ''mvseq'')'];
    end
elseif enableMVseq
    set_param([gcb,'/MVseq'],'Port','2');
    MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(2) ', ''mvseq'')'];
end

% Update MaskDisplay
set_param(gcb,'MaskDisplay',MaskDisplay);

end