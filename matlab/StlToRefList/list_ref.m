function [] = list_ref(movelist_all, z_slices, name)
s = strcat( './' , name , '.txt');

fid = fopen(s, 'wt' );
for i = 1: size(movelist_all,2)
    mlst_all = movelist_all{i};
    if ~isempty(mlst_all)
        for j = 1:size(mlst_all,1)-1
            fprintf( fid, '%f,%f,%f\n',mlst_all(j,1),mlst_all(j,2),ones(1,1)*z_slices(i));
        end
    end  
end   
fclose(fid);
end

