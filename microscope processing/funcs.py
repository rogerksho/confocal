# serial input cleanup
def clean_serial_read(input_list):
    output_list = []
    
    for input in input_list:
        output_list.append(clean_serial(input))
        #print(clean_serial(input))
        
    print_list(output_list)
    
def clean_serial_read_until(input):
    print(input.decode())

def clean_serial(input_in):
    input = str(input_in)
    try:
        input = input.rsplit('>', 1)[1]
    except:
        #print(input)
        pass
    #print(input)
    #input = input.replace('\r\n', "#")
    
    input = input[:-5]
    return input

def print_list(list):
    for i in list:
        print(i)

# prompt user and send input
def get_input_and_send_int(ser):
    # get scan mode from user input
    keyboard_input_str = input(">>> ")
    try:
        data_to_send = int(keyboard_input_str)
    except Exception as e:
        print("invalid input" + str(e))
        
    # write scan mode to esp32
    ser.write(data_to_send.to_bytes(length=1, byteorder='big'))
    
    return keyboard_input_str

# prompt user and send input
def get_input_and_send_coords(ser):
    # get scan mode from user input
    keyboard_input_str = input(">>> ")
        
    # write scan mode to esp32
    ser.write(keyboard_input_str.encode())
    
    return keyboard_input_str