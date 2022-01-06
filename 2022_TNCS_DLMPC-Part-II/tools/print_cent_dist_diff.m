function print_cent_dist_diff(data, name)
    diff = abs(data{1}{4} - data{2}{4});
    fprintf('Cost diff for %s: %.3f\n', name, diff);
end