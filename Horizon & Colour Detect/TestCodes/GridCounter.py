def grid_counter(object_arrays, n_rows, n_columns):

    total = np.copy(object_arrays)

    if len(total) != 240:
        total = np.zeros((240, 520))
        for object in range(len(object_arrays)):
            total = np.add(total, object_arrays[object])

    grid_pixels = np.zeros((n_rows,n_columns))
    grid_width = 520/n_columns
    grid_heigth = 240/n_rows

    for i in range(n_rows):
        for j in range(n_columns):
            grid_pixels[i,j] = np.ndarray.sum(total[int(i*grid_heigth):int((i+1)*grid_heigth - 1), int(j*grid_width):int((j+1)*grid_width - 1)])

    grid_percentage = grid_pixels/grid_width/grid_heigth*100

    return grid_percentage