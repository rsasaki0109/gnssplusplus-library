clear;
clc;
close all;

%% Generate color cube dae file
C = readcell("cube_org.dae", FileType="text", TextType="string"); % original cube dae file
col = jet(256);

for i=1:256
    C{67} = sprintf("<color>%.3f %.3f %.3f 1</color>", col(i,1), col(i,2), col(i,3));
    writecell(C, "cube"+num2str(i,"%04d")+".dae", FileType="text", QuoteStrings="none");
end
