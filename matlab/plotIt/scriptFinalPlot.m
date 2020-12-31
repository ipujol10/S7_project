clear;clc;close all;
warning('off','all')
tic
paths = ["C:\Data\Aalborg\Project\FILES\FINAL\beacon-based","C:\Data\Aalborg\Project\FILES\FINAL\hope_1_real","C:\Data\Aalborg\Project\FILES\FINAL\vertical"]';
labels = ["BBA","Random Walk","Slicer"];
maxns = ["beacon-based.txt","random.txt","vertical.txt"];
plotAl(paths,maxns,labels)
toc