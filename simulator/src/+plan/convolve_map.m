function conv_map = convolve_map(map, inflate_radius, convolve_sigma)
    conv_map = occupancyMap(map);
    inflate(conv_map, inflate_radius)
    conv_map_mat = imgaussfilt(conv_map.occupancyMatrix, convolve_sigma);
    setOccupancy(conv_map, conv_map_mat);
end
