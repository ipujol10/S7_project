function [areaM, overlapM, areaN, t, variance] = coveredArea(path,maxnName,plotIt)
    warning('off','all')
    maxn = importdata(maxnName); % import data from the information file
    iterations = size(maxn,1); % see how many iteration it has
    first = true; % its the first sample
    for i = 1:iterations % go through all iteration
        for n = maxn(i,2:size(maxn,2)) % go through all samples from the iteration
            if ~isnan(n) % if n is not NaN (because they can have different lenghts)
                fileName1 = join([path,"\",i,"\","visited_point",n,".txt"],""); % create the file name of the data sample
                fileData = importdata(fileName1); % import the data sample
                data = fileData.data; % get the actual data
                data = data(3:size(data,1),:); % with the important rows

                if (first) % if  its the first sample initialize everything
                    minN = size(data,1); % get the length of the data
                    first = false; % set the first as false
                    area = data(:,14); % start area
                    overlap = data(:,15); % start overlap
                else
                    minN = min(size(data,1),minN); % see which is the shortest data file
                    area = [area(1:minN,:) data(1:minN,14)]; % add the new column
                    overlap = [overlap(1:minN,:) data(1:minN,15)];
                end

                t = data(1:minN,4); % set the time array
            end
        end
    end
    
    t = t/60; % convert time into minutes
    
    areaM = mean(area,2); % get the mean area of all samples
    overlapM = mean(overlap,2); % get the mean overlap of all samples
    
    %%[Calculate variance]
    variance = area(size(area,1),:) - areaM(size(area,1)); 
    variance = variance.^2;
    variance = sqrt(sum(variance)/size(variance,2));
    %%![Calculate variance]

    areaN = maxn(1,1); % get the area to compare
    
    leg = []; % create the legend
    for i = 1:size(maxn,1)
        for j = maxn(i,2:size(maxn,2))
            if ~isnan(j)
                leg = [leg join(["test",i,j],"")];
            end
        end
    end
    
    if plotIt == 1 % if you want to plot
        absolute = figure;
        plot(t,areaM,'LineWidth',3)
        title('Average coverage over time')
        ylim([0,(max(max(areaM))+.25)])
        xlabel('time [min]')
        ylabel('Area [m^2]')
        saveas(absolute,"accummulated.fig")
        saveas(absolute,"accummulated.png")

        ovrl = figure;
        plot(t,overlapM,'LineWidth',3)
        title('Overlapment')
        ylim([0,(max(max(overlapM))+.25)])
        xlabel('time [min]')
        ylabel('Area [m^2]')
        saveas(ovrl,"overlap.fig")
        saveas(ovrl,"overlap.png")

        per = figure;
        plot(t,areaM/areaN*100,'LineWidth',3)
        title('Average coverage percentage')
        ylim([0,100])
        xlabel('time [min]')
        ylabel('percentage [%]')
        saveas(per,"percentage.fig")
        saveas(per,"percentage.png")

        indi = figure;
        plot(t,area,'LineWidth',3)
        title('Coverage over time')
        ylim([0,(max(max(area))+.25)])
        xlabel('time [min]')
        ylabel('Area [m^2]')
        legend(leg)
        saveas(indi,"individuals.fig")
        saveas(indi,"individuals.png")

        indiper = figure;
        plot(t,area/areaN*100,'LineWidth',3)
        title('Coverage percentage')
        ylim([0,100])
        xlabel('time [min]')
        ylabel('percentage [%]')
        legend(leg)
        saveas(indiper,"individual_percentage.fig")
        saveas(indiper,"individual_percentage.png")
    end