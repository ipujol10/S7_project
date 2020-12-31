%% Path generation
% The main file is slice_stl_create.m 
% The supporting functions are triangle_plane_intersection.m, read_binary_stl_file.m, orient_stl.m, rotate_stl.m and plot_slices.m. 
% The script stl_slice_and_plot.m is an example using the functions.

%%   
clear; close all;
str = ["Stupid_Hull_Base_without_1_1"; "Stupid_Hull_Base_without_1_2" ; "Stupid_Hull_Base_without_2_1"; "Stupid_Hull_Base_without_2_2"; "Stupid_Hull_Base_without_3_1"; "Stupid_Hull_Base_without_3_2";
       "Stupid_Hull_Base_1_1"; "Stupid_Hull_Base_1_2"; "Stupid_Hull_Base_2_1"; "Stupid_Hull_Base_2_2"; "Stupid_Hull_Base_3_1"; "Stupid_Hull_Base_3_2"];


for i = 1:size(str)
   
    s = str(i);
    
    stl = strcat(str(i) , '.STL');
    
    triangles = read_binary_stl_file(stl);

    trianglesRotX = rotate_stl(triangles,'x',90);

    trianglesRotY = rotate_stl(trianglesRotX,'y',180);
    height = 5;

    tic;[moveList,z_slices]=slice_stl_create_path(trianglesRotY,height);toc;

    fprintf("Slice done");

    tic;list_ref(moveList,z_slices,s);toc;

    fprintf("List done");
end
