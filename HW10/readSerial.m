%% reads COM4 for the imu data and plots it
s = serial('COM4');
fopen(s)
flushinput(s);
printMax = 100;
raw = zeros(printMax,1);
iir = zeros(printMax,1);
maf = zeros(printMax,1);
fir = zeros(printMax,1);

fprintf(s,'r'); % tell it to start
pause(6*printMax/100)
for i = 1:printMax
    tline = '';
    while(isempty(tline))
        tline = fgetl(s);
    end
    
    % parse based on spaces
    C = strsplit(tline,' ');
    raw(i,1) = str2num(C{2});
    maf(i,1) = str2num(C{3});
    iir(i,1) = str2num(C{4});
    fir(i,1) = str2num(C{5});
end

fclose(s)


%% plot data

figure;
hold on
plot(raw);
plot(fir);
plot(iir);
plot(maf);
legend('raw','fir','iir','maf')
