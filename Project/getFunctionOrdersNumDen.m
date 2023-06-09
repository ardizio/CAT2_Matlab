% % check: FISICA REALIZZABILITA'
% [f_G_angle_orderNum, f_G_angle_orderDen] = getFunctionOrdersNumDen(f_G_angle);
% % Display the orders
% if f_G_angle_orderNum <= f_G_angle_orderDen
%     disp('f_G_angle è FISICAMENTE REALIZZABILE');
% end
% % check: STABILITA'
% f_G_angle_isStable = isstable(f_G_angle);
% if f_G_angle_isStable
%     disp('f_G_angle Stabilità: STABILE');
% else
%     disp('f_G_angle Stabilità: INSTABILE');
% end


function [order_num, order_den] = getFunctionOrdersNumDen(G)

    % Extract Numeratore coeffs
    Numeratore_coeffs = cell2mat(G.num);
    % NUMERATORE
    MaxIndex = 0;
    maxElements = numel(Numeratore_coeffs);
    i = maxElements;
    count = 1;
    while i > 0
        element = Numeratore_coeffs(i);
        if(element ~= 0)
            MaxIndex = i - count;
            %fprintf('\ngrado Numeratore: %d\n\n',MaxIndex);
            break
        end
        count = count + 1;
        i = i - 1;
    end
    order_num = MaxIndex;


    % Extract Denominatore coeffs
    Denominatore_coeffs = cell2mat(G.den);
    % DENOMINATORE
    MaxIndex = 0;
    maxElements = numel(Denominatore_coeffs);
    i = maxElements;
    count = 1;
    while i > 0
        element = Denominatore_coeffs(i);
        if(element ~= 0)
            MaxIndex = i - count;
            %fprintf('\ngrado Denominatore: %d\n\n',MaxIndex);
            break
        end
        count = count + 1;
        i = i - 1;
    end
    order_den = MaxIndex;

end