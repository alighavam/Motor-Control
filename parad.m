function paradigm = parad(trlNum,Z)

paradigm = zeros(1,trlNum);
tmp = 0;
for i = 1:trlNum
    paradigm(i) = tmp;
    if (rand() < 1-Z)
        tmp = 1 - tmp;
    end
end