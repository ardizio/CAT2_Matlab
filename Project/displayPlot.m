

%{

plot_f_Request = ["Request", "function", "plot_name"];
plot_f_Options = ["Grid_on", "Box_on", "edit_xlabel", "edit_ylabel", "edit_legend"];
displayPlot(plot_Request, function, plot_Options)

functions supported:
- "rlocus"
- "step"
- "bode"
%}
function displayPlot(Request, Input, Options)

    %fprintf('START displayPlot \n');



    % GET from Workspace
    plot_enable_Display = evalin('base', 'plot_enable_Display');

     if plot_enable_Display == 1 
        plot_style_fontSize = evalin('base', 'plot_style_fontSize');
        plot_position_from_left = evalin('base', 'plot_position_from_left'); %#ok<NASGU>
        plot_position_from_bottom = evalin('base', 'plot_position_from_bottom'); %#ok<NASGU>
        plot_window_Width = evalin('base', 'plot_window_Width');
        plot_window_Height = evalin('base', 'plot_window_Height');
    
        plot_device_maxColumns = evalin('base', 'plot_device_maxColumns');
        plot_device_maxRows = evalin('base', 'plot_device_maxRows');
        plot_current_Column = evalin('base', 'plot_current_Column');
        plot_current_Row = evalin('base', 'plot_current_Row');
    
    
        % GET figureIndex from Workspace
        figureIndex = evalin('base', 'plot_figure_Index');
    
        % CREATE new figure
        fig = figure(figureIndex);

   
         %Rollin' && PLOTTIN'
        if Request(1) == "Request"
           if Request(2) == "rlocus"
               rlocus(Input);
           elseif Request(2) == "rlocusplot"
               rlocusplot(Input);
           elseif Request(2) == "step"
               step(Input);
           elseif Request(2) == "stepplot"
               stepplot(Input);
           elseif Request(2) == "impulse"
               impulse(Input);
           elseif Request(2) == "margin"
               margin(Input);
           elseif Request(2) == "bode"
               bodeplot(Input);
           end
        end
       
    
        % SET GRID
        if Options(1) == "Grid_on"
            grid on
        end
        % SET BOX
        if Options(2) == "Box_on"
            box on
        end
        % SET xlabel
        if Options(3) ~= "edit_xlabel"
            xlabel(Options(3), FontSize=plot_style_fontSize, Interpreter='latex')
        end
        % SET ylabel
        if Options(4) ~= "edit_ylabel"
            xlabel(Options(4), FontSize=plot_style_fontSize, Interpreter='latex')
        end
        % SET legend
        if Options(5) ~= "edit_legend"
            xlabel(Options(5), FontSize=plot_style_fontSize, Interpreter='latex')
        end
    
        % Desktop Position
        %fprintf("col: %d\n",plot_current_Column);
        %fprintf("row: %d\n",plot_current_Row);
    
        if plot_current_Row == 1
            plot_position_from_bottom = 1200;
        else
            plot_position_from_bottom = 0;
        end
    
        index_minus_one = plot_current_Column - 1;
        plot_position_from_left = index_minus_one * plot_window_Width;
        %disp(plot_position_from_left)
    
        set(gcf,'position',[plot_position_from_left,plot_position_from_bottom,plot_window_Width,plot_window_Height])
    
        if plot_current_Column >= plot_device_maxColumns
            plot_current_Column = 1;
            if plot_current_Row < plot_device_maxRows
                plot_current_Row = plot_current_Row + 1;
            else
                plot_current_Row = 1;
            end
        else
            plot_current_Column = plot_current_Column + 1;
        end
    
        % Set visible title name
        % title(Request(3), FontSize=plot_style_fontSize, Interpreter='none')
    
    
        % Set window name
        set(fig, 'Name', Request(3));
    
    
    
        % RETURN to Workspace plot_figure_Index
        figureIndex = figureIndex + 1;
        assignin('base', 'plot_figure_Index', figureIndex);
    
        assignin('base', 'plot_current_Column', plot_current_Column);
        assignin('base', 'plot_current_Row', plot_current_Row);       
        assignin('base', 'plot_position_from_left', plot_position_from_left);       
        assignin('base', 'plot_position_from_bottom', plot_position_from_bottom);       
    
        %fprintf('END displayPlot\n');
        hold off
    end
    
   

end