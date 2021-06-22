function nd = initMaskPortNMPC(nd,enableDist,enableSlack,enableStatus,enableYseq,enableFnorm,kindNMPC)

% Mask Display
if strcmp(kindNMPC,'SQP')
    MaskDisplay = ['fprintf(''Nonlinear MPC\nSQP'')' char(10) ...
         'port_label(''Output'', ' num2str(1) ', ''mv'')' char(10) ...
         'port_label(''Input'', ' num2str(1) ', ''state'')' char(10) ...
         'port_label(''Input'', ' num2str(2) ', ''ref'')'];
elseif strcmp(kindNMPC,'CGMRES')
    MaskDisplay = ['fprintf(''Nonlinear MPC\nC/GMRES'')' char(10) ...
         'port_label(''Output'', ' num2str(1) ', ''mv'')' char(10) ...
         'port_label(''Input'', ' num2str(1) ', ''state'')' char(10) ...
         'port_label(''Input'', ' num2str(2) ', ''ref'')'];
else
    MaskDisplay = ['fprintf(''Nonlinear MPC\nN/GMRES'')' char(10) ...
         'port_label(''Output'', ' num2str(1) ', ''mv'')' char(10) ...
         'port_label(''Input'', ' num2str(1) ', ''state'')' char(10) ...
         'port_label(''Input'', ' num2str(2) ', ''ref'')'];
end

% For Disturbance Input port
if ~enableDist
    % Disable the port
    % Number of disutrbance input
    nd = 1; % Default value
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
else
    % Enable the port
    try
        lh = get_param([gcb, '/Ground_md'],'LineHandles');
        position = get_param([gcb, '/Ground_md'],'Position');
        delete_line(lh.Outport(1));
        delete_block([gcb,'/Ground_md']);
        h = add_block('built-in/Inport',[gcb, '/md']);
        set_param(h,'position',position);
        ph1 = get_param(h,'PortHandles');
        ph2 = get_param([gcb ,'/Signal Specification md'],'PortHandles');
        add_line(gcb,ph1.Outport(1),ph2.Inport(1));
    catch
    end
    
    % Update MaskDisplay
    MaskDisplay = [MaskDisplay char(10) 'port_label(''Input'', ' num2str(3) ', ''md'')'];
    
    % Update order of inport
    set_param([gcb,'/md'],'Port','3');
end

if strcmp(kindNMPC,'SQP')
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

    % Sort order of Outport
    if enableSlack
        set_param([gcb,'/Slack'],'Port','2');
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(2) ', ''Slack'')'];
        if enableStatus
            set_param([gcb,'/Status'],'Port','3');
            MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(3) ', ''Status'')'];
        end
    elseif enableStatus
        set_param([gcb,'/Status'],'Port','2');
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(2) ', ''Status'')'];
    end
else
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
    
    % For Sequence Output
    if ~enableYseq
        % Disable the port
        try
            lh = get_param([gcb,'/yseq'],'LineHandles');
            position = get_param([gcb, '/yseq'],'Position');
            delete_line(lh.Inport(1));
            delete_block([gcb, '/yseq']);
            h = add_block('built-in/Terminator',[gcb,'/Terminator_yseq']);
            set_param(h,'position',position);
            ph1 = get_param(h,'PortHandles');
            ph2 = get_param([gcb,'/Signal Specification yseq'],'PortHandles');
            add_line(gcb,ph2.Outport(1),ph1.Inport(1));
        catch
        end
    else
        % Enable the port
        try
            lh = get_param([gcb,'/Terminator_yseq'],'LineHandles');
            position = get_param([gcb, '/Terminator_yseq'],'Position');
            delete_line(lh.Inport(1));
            delete_block([gcb,'/Terminator_yseq']);
            h = add_block('built-in/Outport',[gcb,'/yseq']);
            set_param(h,'position',position);
            ph1 = get_param(h,'PortHandles');
            ph2 = get_param([gcb,'/Signal Specification yseq'],'PortHandles');
            add_line(gcb,ph2.Outport(1),ph1.Inport(1));
        catch
        end
    end

    % For Fnorm Output
    if ~enableFnorm
        % Disable the port
        try
            lh = get_param([gcb,'/F'],'LineHandles');
            position = get_param([gcb, '/F'],'Position');
            delete_line(lh.Inport(1));
            delete_block([gcb, '/F']);
            h = add_block('built-in/Terminator',[gcb,'/Terminator_F']);
            set_param(h,'position',position);
            ph1 = get_param(h,'PortHandles');
            ph2 = get_param([gcb,'/Signal Specification F'],'PortHandles');
            add_line(gcb,ph2.Outport(1),ph1.Inport(1));
        catch
        end
    else
        % Enable the port
        try
            lh = get_param([gcb,'/Terminator_F'],'LineHandles');
            position = get_param([gcb, '/Terminator_F'],'Position');
            delete_line(lh.Inport(1));
            delete_block([gcb,'/Terminator_F']);
            h = add_block('built-in/Outport',[gcb,'/F']);
            set_param(h,'position',position);
            ph1 = get_param(h,'PortHandles');
            ph2 = get_param([gcb,'/Signal Specification F'],'PortHandles');
            add_line(gcb,ph2.Outport(1),ph1.Inport(1));
        catch
        end
    end
    
    % Sort order of Outport
    if enableStatus
        set_param([gcb,'/Status'],'Port','2');
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(2) ', ''Status'')'];
        if enableYseq
            set_param([gcb,'/yseq'],'Port','3');
            MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(3) ', ''yseq'')'];
            if enableFnorm
                set_param([gcb,'/F'],'Port','4');
                MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(4) ', ''F'')'];
            end
        elseif enableFnorm
                set_param([gcb,'/F'],'Port','3');
                MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(3) ', ''F'')'];
        end
    elseif  enableYseq
        set_param([gcb,'/yseq'],'Port','2');
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(2) ', ''yseq'')'];
        if enableFnorm
            set_param([gcb,'/F'],'Port','3');
            MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(3) ', ''F'')'];
        end
    elseif enableFnorm
        set_param([gcb,'/F'],'Port','2');
        MaskDisplay = [MaskDisplay char(10) 'port_label(''Output'', ' num2str(2) ', ''F'')'];
    end
end

% Update MaskDisplay
set_param(gcb,'MaskDisplay',MaskDisplay);

end