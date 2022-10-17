import serial
import serial.tools.list_ports
# import time

# to intelligently find the right port
# for p in serial.tools.list_ports.comports():
#     print(p.hwid)

def get_port_from_ser(ser):
    '''Use the serial number for the port to ensure connection to the correct device
    the serial number can be found using:
    python3 -m serial.tools.list_ports -v
    '''
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if p.serial_number == ser:
            return p.device
    return ""


class RelayBoard():
    def __init__(self, port, num_relays):
        self.num_relays = num_relays
        self.port = port
        self.baud = 115200
        self.ser = 0
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        print(self.ser.name)

        self.status_base_idx = 16
        self.set_on_base_idx = 8
        self.set_off_base_idx = 0
        self.prefix = 254

    def connect_serial(self):
        #connect
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)

    def close_serial(self):
        try:
            self.ser.close()
        except:
            print('Serial Port Not Open')

    def is_communicating(self):
        '''Checks that serial communication is working'''
        self.ser.write(bytes([self.prefix, 33]))
        response = self.ser.read(1)
        if response == b"U":
            return True
        else:
            return False
        # else:
        #     print("Unexpected response from relay: {}".format(ord(response)))
        #     return False

    def read_serial(self):
        not_read = True
        while not_read:

            try:
                resp = self.ser.read(1)
                not_read = False
            except serial.serialutil.SerialException:
                print('ERROR \n ------')
                self.close_serial()
                self.connect_serial()
                continue
            # break
        return resp



    def set_relay(self, idx, status):
        if idx < 0 or idx > self.num_relays:
            return False

        status = int(status)
        if not (status == 0 or status == 1):
            return False

        if status:
            self.ser.write(bytes([self.prefix, self.set_on_base_idx + idx]))
        else:
            self.ser.write(bytes([self.prefix, self.set_off_base_idx + idx]))

        #check response
        # if self.ser.read() == b'U':
        resp = self.read_serial()
        # print(resp)
        if resp == b'U':
            return True
        else:
            return False

    def get_status(self):
        status = []
        compare_bits = [0,1,2,4]
        self.ser.write(bytes([self.prefix, 24]))
        status_byte = int.from_bytes(self.read_serial(),"big")
        for i in compare_bits:
            status.append((status_byte >> i) & 1)
        return status

if __name__ == "__main__":
    #testing script
    # print(get_port_from_ser("AB0MSGBD"))

    control = RelayBoard(get_port_from_ser("AB0MSGBD"), 4)
    print(control.is_communicating())

    # print(control.set_relay(0, 0))
    # print(control.set_relay(1, 0))
    # print(control.set_relay(2, 1))
    # print(control.set_relay(3, 1))
    # print(control.get_status())
