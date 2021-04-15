import struct


# Reads [chunk] bytes from the file at once, caching them
# May or may not be faster
def next_byte(b, file, chunk):
    if len(b) == 0:
        b = file.read(chunk)
    byte = None if len(b) == 0 else b[0]
    b = b[1:]
    return [byte, b]


path = "./../code/data/thetacalc1hzlowsen_td.dat"
chunk = 1024

f = open(path, 'rb')

try:
    b = []

    [byte, b] = next_byte(b, f, chunk)

    hit_new_line = True
    end_of_header = False
    do_skip_one = False

    current_comment = ""

    current_data = []

    while byte != None:

        if do_skip_one:
            do_skip_one = False
            [byte, b] = next_byte(b, f, chunk)
            continue

        # Do something
        if not end_of_header:
            # Process header
            if hit_new_line:
                hit_new_line = False

                if chr(byte) == "%":
                    print("Comment found")
                else:
                    print("End of header")
                    end_of_header = True
                    do_skip_one = True
            if not end_of_header:
                if chr(byte) == "\n":
                    print(current_comment)
                    current_comment = ""
                    hit_new_line = True
                else:
                    current_comment += chr(byte)
                    hit_new_line = False
        else:
            # Process data
            current_data.append(byte)
            if len(current_data) == 8:

                # Extract data from 8 bytes
                ts, dat = struct.unpack("<LL", bytearray(current_data))
                x = (dat & 0x00003FFF)
                y = (dat & 0x0FFFC000) >> 14
                p = 1 if ((dat & 0x10000000) >> 28) else -1
                
                #print(f"ts {ts}\tx {x}\ty {y}\tp {p}")
                print("ts {}\tx {}\ty {}\tp {}".format(ts, x, y, p))
                
                current_data = []
        
        [byte, b] = next_byte(b, f, chunk)
    
finally:
    f.close()


