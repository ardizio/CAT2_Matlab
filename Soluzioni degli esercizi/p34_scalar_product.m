function [o] = p34_scalar_product(vec1,vec2)
%SCALAR_PRODUCT
%   given 2 vectors of same lenght, output their slalar product
    o = 0;
    if length(vec1) == length(vec2)
        for x=1:length(vec1)
            o = o + vec1(1,x) * vec2(1,x);
        end
    else
        o = NaN;
    end
end

