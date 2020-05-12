clear all
close all
clc

col = 7;

for ii = 1:col
    if ii < col
        str{ii} = '%f ';
    else
        str{ii} = '%f';
    end
end
str = cell2mat(str);
    