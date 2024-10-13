x_max = 5
y_max = 5
z_max = 5

x_dir = True
y_dir = True
z_dir = True

ctr = 0

for i in range(x_max + 1):
    for j in range(y_max + 1):
        for k in range(z_max + 1):
            if x_dir:
                if y_dir:
                    print(f'{ctr}: {i}, {j}, {k}')
                else:
                    print(f'{ctr}: {i}, {y_max - j}, {k}')
            else:
                if y_dir:
                    print(f'{ctr}: {i}, {j}, {z_max - k}')
                else:
                    print(f'{ctr}: {i}, {y_max - j}, {z_max - k}')
            ctr += 1
            print(k)
        x_dir = not x_dir
    y_dir = not y_dir

            