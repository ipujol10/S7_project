clear;clc;close all;
warning("off","all");
path = ["C:\Data\Aalborg\Project\FILES\FINAL\vertical\1\visited_point3.txt","C:\Data\Aalborg\Project\FILES\FINAL\hope_1_real\1\visited_point8.txt","C:\Data\Aalborg\Project\FILES\FINAL\beacon-based\1\visited_point8.txt"];
names = ["Slicer","Random Walk","BBA"];
% path = ["C:\Data\Aalborg\Project\FILES\FINAL\vertical\1\visited_point3.txt","C:\Data\Aalborg\Project\FILES\FINAL\beacon-based\1\visited_point8.txt"];
% plotPath("C:\Data\Aalborg\Project\FILES\FINAL\vertical\1\visited_point3.txt")
% plotPath("C:\Data\Aalborg\Project\FILES\FINAL\hope_1_real\1\visited_point8.txt")
% plotPath("C:\Data\Aalborg\Project\FILES\FINAL\beacon-based\1\visited_point8.txt")
% plotPath("C:\Data\Aalborg\Project\FILES\FINAL\24h\mark\1\visited_point2.txt")
% for i = 1:8
%     plotPath(join(["C:\Data\Aalborg\Project\FILES\FINAL\vertical\1\visited_point",i,".txt"],""))
% end
% fig = figure;
% hold on
for i = 1:3
    fig = figure;
    plotPath(path(i))
    title(join(["Path ",names(i)],""))
    xlabel('x')
    ylabel('y')
    saveas(fig,join(["path",names(i),".fig"],""))
    saveas(fig,join(["path",names(i),".png"],""))
end
% hold off
% title('Path comparison')
% xlabel('x')
% ylabel('y')
% legend(["Slicer","BBA"])
% saveas(fig,"path.fig")
% saveas(fig,"path.png")

function plotPath(name)
% figure;
% line = ["k","g"];
dataFile = importdata(name);
data = dataFile.data;
data = data(3:size(data,1),16:17);
plot(data(:,1),data(:,2));
end