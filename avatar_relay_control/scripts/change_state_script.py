import relay
import time

#arm_gripper = "AB0MSGBD"
camera = "AB0MSPTK"
board = relay.RelayBoard(relay.get_port_from_ser(camera), 4)
board.set_relay(0, True)
board.set_relay(1, True)
board.set_relay(2, False)
board.set_relay(3, False)
# time.sleep(1)
# # print(board.get_status())
# # time.sleep(1)
# # board.ser.write(bytes([board.prefix, 131, 0]))
# # board.ser.write(bytes([board.prefix, 34]))
# board.ser.write(bytes([board.prefix, 42, 0]))
# # time.sleep(1)
# print(board.read_serial())
# board.ser.write(bytes([board.prefix, 43, 1]))
# # time.sleep(1)
# print(board.read_serial())
