%find what binary pairs are in a number e.g. 1101 has three binary pairs
%1001, 0101 and 1100
function bi_pairs = findBiPairs(num, num_of_bits)
    bi_pairs = [];
    for i = 1:num_of_bits-1
        for j = i+1:num_of_bits
            %check if both bits being checked are on
            if (bitget(num,i) + bitget(num,j) == 2)
                %bi_pairs = [bi_pairs; 2^(i-1) + 2^(j-1)];
                bi_pairs = [bi_pairs; (j-2)*(j-1)/2+i]; %returns the indexes
            end
        end
    end
return;
end