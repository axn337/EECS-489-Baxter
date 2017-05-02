function g_temp = custom_cum_prod(input_mat,num_start, num_end)
    g_temp = eye(4,4);
    for i = num_start:num_end
        g_temp = g_temp*input_mat{i};
    end
end