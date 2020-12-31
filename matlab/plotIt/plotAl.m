function plotAl(paths,maxnNames,labels)
first = true; % initialize variables
for i = 1:size(paths)
    path = paths(i); % get the path
    maxnName = maxnNames(i); % get the file information name
    [areaM, overlapM, areaN, t, variance] = coveredArea(path,maxnName,0); % evaluate algorithm
    
    area = areaM/areaN*100; % get percentage
    
    if first % if it's the first initialize everithing
        minN = size(areaM,1);
        first = false;
        areas = area;
        areasM = areaM;
        overlaps = overlapM;
    else % complement
        minN = min(size(areaM,1),minN);
        areas = [areas(1:minN,:) area(1:minN,:)];
        areasM = [areasM(1:minN,:) areaM(1:minN,:)];
        overlaps = [overlaps(1:minN,:) overlapM(1:minN,:)];
    end
    disp(join([labels(i),": ",variance],""))
end
t = t(1:minN,:); % trim the time array

for i = 1:size(labels,2)
    slope = (areasM(size(t,1),i) - areasM(59750,i))/(t(size(t,1)) - t(59750));
    disp(join([labels(i)," slope: ",slope],""))
end

lines = ['o','*','^','x']; % diferent types of lines to be able to differentiate them even with a copy in gray scale

%%[Plot percentages]
figG = figure;
hold on
for i = 1:size(areas,2)
    plot(t(1:11000:end),areas(1:11000:end,i),lines(i)) % take very few points to make it possible to distinguish the patterns
end
hold off
ylim([0,50])
xlabel('time [min]')
ylabel('percentage [%]')
title('Comparison of the efficiency of each algorithm')
legend(labels)
saveas(figG,"alinoneG.fig")
saveas(figG,"alinoneG.png")
%%![Plot percentages]

%%[Plot accummulated area]
figA = figure;
hold on
for i = 1:size(areas,2)
    plot(t(1:11000:end),areasM(1:11000:end,i),lines(i))
end
hold off
xlabel('time [min]')
ylabel('area [m^2]')
title('Comparison of the covered area over time')
legend(labels)
saveas(figA,"oneArea.fig")
saveas(figA,"oneArea.png")
%%![Plot accummulated area]

%%[Plot overlapment]
figO = figure;
hold on
for i = 1:size(areas,2)
    plot(t(1:11000:end),overlaps(1:11000:end,i),lines(i))
end
hold off
xlabel('time [min]')
title('Comparison of the overlapment')
ylabel('area [m^2]')
legend(labels)
saveas(figO,"oneOverlap.fig")
saveas(figO,"oneOverlap.png")
%%![Plot overlapment]