

%{

Request = ["Request", "function", "plot_name"];
Options = ["Grid_on", "Box_on", "edit_xlabel", "edit_ylabel", "edit_legend"];
plotWithCustomOptions(Request, function, Options)

%}

function plotWithCustomOptions(Request, Input, Options)

    %fprintf('START plotWithCustomOptions \n');



    % GET from Workspace
    plot_font_size = evalin('base', 'plot_font_size');
    plot_from_left = evalin('base', 'plot_from_left'); %#ok<NASGU>
    plot_from_bottom = evalin('base', 'plot_from_bottom'); %#ok<NASGU>
    plot_width = evalin('base', 'plot_width');
    plot_height = evalin('base', 'plot_height');

    plot_MaxColumns = evalin('base', 'plot_MaxColumns');
    plot_max_rows = evalin('base', 'plot_max_rows');
    plot_CurrentColumn = evalin('base', 'plot_CurrentColumn');
    plot_CurrentRow = evalin('base', 'plot_CurrentRow');


    % GET figureIndex from Workspace
    figureIndex = evalin('base', 'plot_figureIndex');

    % CREATE new figure
    fig = figure(figureIndex);

   

    
    %Rolloin' && PLOTTIN'
    if Request(1) == "Request"
       if Request(2) == "rlocus"
           rlocus(Input);
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
        xlabel(Options(3), FontSize=plot_font_size, Interpreter='latex')
    end
    % SET ylabel
    if Options(4) ~= "edit_ylabel"
        xlabel(Options(4), FontSize=plot_font_size, Interpreter='latex')
    end
    % SET legend
    if Options(5) ~= "edit_legend"
        xlabel(Options(5), FontSize=plot_font_size, Interpreter='latex')
    end

    % Desktop Position
    %fprintf("col: %d\n",plot_CurrentColumn);
    %fprintf("row: %d\n",plot_CurrentRow);

    if plot_CurrentRow == 1
        plot_from_bottom = 1200;
    else
        plot_from_bottom = 0;
    end

    index_minus_one = plot_CurrentColumn - 1;
    plot_from_left = index_minus_one * plot_width;
    %disp(plot_from_left)

    set(gcf,'position',[plot_from_left,plot_from_bottom,plot_width,plot_height])

    if plot_CurrentColumn >= plot_MaxColumns
        plot_CurrentColumn = 1;
        if plot_CurrentRow < plot_max_rows
            plot_CurrentRow = plot_CurrentRow + 1;
        else
            plot_CurrentRow = 1;
        end
    else
        plot_CurrentColumn = plot_CurrentColumn + 1;
    end

    

    set(fig, 'Name', Request(3));


    % RETURN to Workspace plot_figureIndex
    figureIndex = figureIndex + 1;
    assignin('base', 'plot_figureIndex', figureIndex);

    assignin('base', 'plot_CurrentColumn', plot_CurrentColumn);
    assignin('base', 'plot_CurrentRow', plot_CurrentRow);       
    assignin('base', 'plot_from_left', plot_from_left);       
    assignin('base', 'plot_from_bottom', plot_from_bottom);       

    %fprintf('END plotWithCustomOptions\n');
end



 %{
% Check if line can accept one more plot


if plot_CurrentRow > plot_max_rows
        plot_CurrentRow = 1
end
plot_CurrentRow = plot_CurrentRow + 1
    %}