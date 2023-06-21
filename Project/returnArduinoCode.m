function returnArduinoCode(Numerator, Denominator, StringToBeAdded ,CopyOption)

    % arduinoString_init = "";
    % arduinoString_wait = "";
    % arduinoString_control = "";
    arduinoSpaces = "      ";

    arduinoString_result = "u"+StringToBeAdded+"[0]=";

    e_counter = 0;
    u_counter = 1;

    % Num_Contains_e = 0;
    % Den_Contains_e = 1;

    count_cells_num = numel(Numerator);
    count_cells_den = numel(Denominator);  

    multiplier = -1;

    if count_cells_num == 1
        invertedArray_Num = Numerator{1} * multiplier;
        % disp(invertedArray_Num);
        % disp(numel(invertedArray_Num));
        for i = 1:numel(invertedArray_Num)
            % disp(invertedArray_Num(i))
            needPlus = "";
             if(invertedArray_Num(i) > 0 && i > 1)
                 needPlus = "+";
             end
            arduinoString_result = arduinoString_result + needPlus + num2str(invertedArray_Num(i)) + "*u"+StringToBeAdded+"["+num2str(u_counter)+"]";
            u_counter = u_counter + 1;
        end

    end
    if count_cells_den == 1
        invertedArray_Den = Denominator{1} * multiplier;
        % disp(invertedArray_Den);
        % disp(numel(invertedArray_Den));
         for i = 1:numel(invertedArray_Den)
             % disp(invertedArray_Den(i))
             needPlus = "";
             if(invertedArray_Den(i) > 0)
                 needPlus = "+";
             end
             arduinoString_result = arduinoString_result + needPlus + num2str(invertedArray_Den(i)) + "*e"+StringToBeAdded+"["+num2str(e_counter)+"]";
             e_counter = e_counter + 1;
        end
    end
    arduinoString_result = arduinoString_result + ";";

    fprintf(arduinoString_result)
    if(CopyOption == 1)
        fprintf("\n")
        arduinoString_result = arduinoSpaces + arduinoString_result;
        clipboard('copy', arduinoString_result);
    end

    
end