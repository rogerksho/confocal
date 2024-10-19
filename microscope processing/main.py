import time
import serial
from funcs import *
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from scipy import signal
import csv

# plotting
colormap = matplotlib.cm.spring

# stage params
STEPS_PER_REV = 200
TRAVEL_PER_STEP = 0.003175

X_Y_MICROSTEP_MULTIPLIER = 1
Z_MICROSTEP_MULTIPLIER = 2

RESOLUTION_REDUCTION_FACTOR = 1

# serial setup
ser = serial.Serial()
ser.port='/dev/cu.usbserial-0001'
ser.baudrate=115200
ser.bytesize = serial.EIGHTBITS
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE

ser.timeout = 0.1

try: 
    ser.open()
except Exception as e:
    print("error opening serial port: " + str(e))
    
if ser.isOpen():
    ser.flushInput()  # flush input buffer, discarding all its contents
    ser.flushOutput() # flush output buffer, aborting current output 
                      # and discard all that is in buffer

    time.sleep(1)  # give the serial port sometime to receive the data

    # read prompt 
    clean_serial_read(ser.readlines())
    
    # get user input for scan mode
    keyboard_input_str = get_input_and_send_int(ser)
    
    # give the serial port sometime to receive the data
    time.sleep(0.1)
    
    # read confirmation of scan mode
    clean_serial_read(ser.readlines())
    ser.timeout = 0.1
        
    # if scan mode is direct
    if keyboard_input_str == "2":
        time.sleep(0.1) # need to wait for scan to start
        while True:
            # read prompt for coords
            clean_serial_read(ser.readlines())
            clean_serial_read(ser.readlines())
            clean_serial_read(ser.readlines())
            
            coord_sent = get_input_and_send_coords(ser)
            time.sleep(3)
            
    # if circle
    elif keyboard_input_str == "1":
        time.sleep(0.1) # need to wait for scan to start
        print("circle mode:")
        while True:
            # read prompt for coords
            ser.timeout = 2
            clean_serial_read_until(ser.read_until())

    elif keyboard_input_str == "3":
        time.sleep(0.1) # need to wait for scan to start
        # read prompt 
        clean_serial_read(ser.readlines())
        
        # get user input for scan mode
        keyboard_input_str = get_input_and_send_coords(ser)
        keyboard_input_str = keyboard_input_str.replace(' ', "")
        
        input_list = keyboard_input_str.split(',')
        
        assert(len(input_list) == 5)
        
        scan_height = float(input_list[4])
        
        scan_height_step = int(round(scan_height*Z_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))

        # split keyboard input str into scan parameters
        scan_z_len = scan_height_step + 1
        
        assert(scan_z_len)
        
        time.sleep(1)
        
        # read response
        clean_serial_read(ser.readlines())

        # init 3d map 
        z_list = []

        ctr = 1
        
        ser.timeout = 1
        time.sleep(3)
        clean_serial_read(ser.readlines())
        
        print("starting scan...")
        
        start_time = time.time()
        
        for j in range(scan_z_len):
            # read confirmation of scan mode 2 bytes
            ser.timeout = 1
            ser.flush()
            data = ser.read(4)
            #print(data)
            data_int = int.from_bytes(data, 'little', signed=True)
            #print(data_int)

            # save data
            print(f'{ctr}: {data_int}')
            z_list.append(data_int)
            ctr += 1
            
        print("scanning done")
        end_time = time.time()
        
        print(f"total scan time: {end_time - start_time} seconds")

        print("plotting vertical data...")
        x = [i for i in range(0, scan_z_len)]
        y = z_list
        
        y_filt = signal.medfilt(y)
        
        print(f"y max: {x[np.argmax(y)]} y_filt max: {x[np.argmax(y_filt)]}")
        print(len(y))
        print(len(y_filt))
        
        fig, ax = plt.subplots()
        ax.plot(x, y)
        ax.plot(x, y_filt)
        ax.legend(["og", "medfilt"])
        plt.show()
        
    # if scan mode is rectangle
    elif keyboard_input_str == "0":
        time.sleep(0.1) # need to wait for scan to start
        # read prompt 
        clean_serial_read(ser.readlines())
        
        # get user input for scan mode
        keyboard_input_str = get_input_and_send_coords(ser)
        keyboard_input_str = keyboard_input_str.replace(' ', "")
        
        input_list = keyboard_input_str.split(',')
        
        assert(len(input_list) == 5)
        
        x1 = float(input_list[0])
        y1 = float(input_list[1])
        x2 = float(input_list[2])
        y2 = float(input_list[3])
        scan_height = float(input_list[4])
        
        x1_step = int(round(x1*X_Y_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))
        y1_step = int(round(y1*X_Y_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))
        x2_step = int(round(x2*X_Y_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))
        y2_step = int(round(y2*X_Y_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))
        scan_height_step = int(round(scan_height*Z_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))

        # split keyboard input str into scan parameters
        scan_x_len = int((x2_step - x1_step)/RESOLUTION_REDUCTION_FACTOR) + 1
        scan_y_len = int((y2_step - y1_step)/RESOLUTION_REDUCTION_FACTOR) + 1
        scan_z_len = scan_height_step + 1
        
        assert(scan_x_len == scan_y_len)
        
        assert(scan_x_len)
        assert(scan_y_len)
        assert(scan_z_len)
        
        # strings for saving in data file
        scan_coords_xy = f'XY travel [mm]: ({x1}, {y1}) -> ({x2}, {y2})'
        scan_coords_z = f'Z travel [mm]: (0) -> ({scan_height})'
        
        scan_steps_xy = f'XY travel [step]: ({x1_step}, {y1_step}) -> ({x2_step}, {y2_step})'
        scan_steps_z = f'Z travel [step]: (0) -> ({scan_height_step})'
        
        scan_dims_steps = f'({scan_x_len} x {scan_y_len} x {scan_z_len})'
        
        preamble = [scan_coords_xy, scan_coords_z, scan_steps_xy, scan_steps_z, scan_dims_steps]
        
        time.sleep(1)
        
        # read response
        clean_serial_read(ser.readlines())

        # init 3d map 
        height_map = [[23 for j in range(scan_y_len)] for i in range(scan_x_len)]
        magnitude_map = [[0 for j in range(scan_y_len)] for i in range(scan_x_len)]
        
        # plot data
        figure, (axis, axis2) = plt.subplots(1, 2)
        im = axis.imshow(height_map, interpolation='none', origin='lower', cmap='Greys')
        im2 = axis2.imshow(magnitude_map, interpolation='none', origin='lower', cmap='Greys')
        
        axis.set_title("height map")
        axis2.set_title("magnitude map")
        
        plt.pause(0.001)
        plt.show(block=False)

        ctr = 1
        
        ser.timeout = 1
        time.sleep(3)
        clean_serial_read(ser.readlines())
        
        temp_data_list = []
        
        x_dir = True
        y_dir = True
        z_dir = True
        
        print("starting scan...")
        
        start_time = time.time()
        
        for i in range(scan_x_len):
            for j in range(scan_y_len):
                # read confirmation of scan mode 2 bytes
                ser.timeout = None
                ser.flush()
                data = ser.read(4)
                data_magnitude = ser.read(4)
                #print(data)
                data_int = int.from_bytes(data, 'little', signed=True)
                data_mag_int = int.from_bytes(data_magnitude, 'little', signed=True)
                #print(data_int)

                # save data
                print(f'({i},{j}): {data_int} || {data_mag_int}')
                temp_data_list.append(data_int)
                
                if (y_dir):
                    height_map[i][j] = data_int
                    magnitude_map[i][j] = data_mag_int

                else:
                    height_map[i][(scan_y_len - 1) - j] = data_int
                    magnitude_map[i][(scan_y_len - 1) - j] = data_mag_int

                # update plot after every new data point
                im.set_data(height_map)
                im.autoscale()
                
                im2.set_data(magnitude_map)
                im2.autoscale()
                
                plt.pause(0.001)
                ctr += 1
            y_dir = not y_dir
            
        print("scanning done")
        end_time = time.time()
        
        print(f"total scan time: {end_time - start_time} seconds")
        print("processing data now")
        
        #print(z_map)
        
        # process and filter data
        #for i in range(scan_x_len):
        #    for j in range(scan_y_len):
        #        pixel_data = z_map[i][j]
        #        pixel_data_filt = signal.medfilt(pixel_data)
        #        height_map[i][j] = np.argmax(pixel_data_filt)
        
        print(f"max height: {np.amax(height_map)}")
        
        filename = int(time.time())
        
        # height map
        with open(f"./scans/{filename}.csv","w+") as my_csv:
            csvWriter = csv.writer(my_csv,delimiter=',')
            preWriter = csv.writer(my_csv, delimiter='\t',
                lineterminator='\r\n',
                quoting = csv.QUOTE_NONE
                )
            for s in preamble:
                preWriter.writerow([s])
            csvWriter.writerows(height_map)
        
        # magnitude map 
        with open(f"./scans/{filename}_magnitude.csv","w+") as my_csv:
            csvWriter = csv.writer(my_csv,delimiter=',')
            preWriter = csv.writer(my_csv, delimiter='\t',
                lineterminator='\r\n',
                quoting = csv.QUOTE_NONE
                )
            for s in preamble:
                preWriter.writerow([s])
            csvWriter.writerows(magnitude_map)
        

        plt.show()
                
        time.sleep(10)
        
    elif keyboard_input_str == "9":
        time.sleep(0.1) # need to wait for scan to start
        # read prompt 
        clean_serial_read(ser.readlines())
        
        # get user input for scan mode
        keyboard_input_str = get_input_and_send_coords(ser)
        keyboard_input_str = keyboard_input_str.replace(' ', "")
        
        input_list = keyboard_input_str.split(',')
        
        assert(len(input_list) == 5)
        
        x1 = float(input_list[0])
        y1 = float(input_list[1])
        x2 = float(input_list[2])
        y2 = float(input_list[3])
        scan_height = float(input_list[4])
        
        x1_step = int(round(x1*X_Y_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))
        y1_step = int(round(y1*X_Y_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))
        x2_step = int(round(x2*X_Y_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))
        y2_step = int(round(y2*X_Y_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))
        scan_height_step = int(round(scan_height*Z_MICROSTEP_MULTIPLIER/(TRAVEL_PER_STEP)))

        # split keyboard input str into scan parameters
        scan_x_len = x2_step - x1_step + 1
        scan_y_len = y2_step - y1_step + 1
        scan_z_len = scan_height_step + 1
        
        assert(scan_x_len)
        assert(scan_y_len)
        assert(scan_z_len)
        
        time.sleep(1)
        
        # read response
        clean_serial_read(ser.readlines())

        # init 3d map 
        z_map = [[[0 for k in range(scan_z_len)] for j in range(scan_y_len)] for i in range(scan_x_len)]

        ctr = 1
        
        ser.timeout = 1
        time.sleep(3)
        clean_serial_read(ser.readlines())
        
        temp_data_list = []
        
        x_dir = True
        y_dir = True
        z_dir = True
        
        print("starting scan...")
        
        start_time = time.time()
        
        for i in range(scan_x_len):
            for j in range(scan_y_len):
                for k in range(scan_z_len):
                    # read confirmation of scan mode 2 bytes
                    ser.timeout = 1
                    ser.flush()
                    data = ser.read(4)
                    #print(data)
                    data_int = int.from_bytes(data, 'little', signed=True)
                    #print(data_int)

                    # save data
                    print(f'{ctr}: {data_int}')
                    temp_data_list.append(data_int)
                    
                    if (z_dir):
                        if (y_dir):
                            z_map[i][j][k] = data_int

                        else:
                            z_map[i][(scan_y_len - 1) - j][k] = data_int
                    else:
                        if (y_dir):
                            z_map[i][j][scan_z_len - 1 - k] = data_int

                        else:
                            z_map[i][(scan_y_len - 1) - j][scan_z_len - 1 - k] = data_int

                #print(time.time())
                    ctr += 1
                z_dir = not z_dir
            y_dir = not y_dir
            
        print("scanning done")
        end_time = time.time()
        
        print(f"total scan time: {end_time - start_time} seconds")
        print("processing data now")
        
        x = [i for i in range(0, scan_z_len)]
        
        # process and filter data
        for i in range(scan_x_len):
            for j in range(scan_y_len):
                pixel_data = z_map[i][j]
                pixel_data_filt = signal.medfilt(pixel_data)
                plt.figure()
                plt.plot(x, pixel_data)
                plt.plot(x, pixel_data_filt)
                plt.legend(["og", "medfilt"])
        
        plt.show()
        
        filename = int(time.time())
                
        time.sleep(2)

    elif keyboard_input_str == "4":
        time.sleep(0.1)
        while True:
            ser.timeout = None
            ser.flush()
            data = ser.read(4)
            #print(data)
            data_int = int.from_bytes(data, 'little', signed=True)
            #print(data_int)

            # save data
            print(f'{data_int}')
    
    # if anything else
    else:
        pass     
else:
    print("cannot open serial port ")