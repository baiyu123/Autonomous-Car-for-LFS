from inputs import get_gamepad

# left trigger ABS_Z 0-255
# right trigger ABS_RZ 0-255
# joy ABS_X ABS_Y ABS_RX ABS_RY 0-32767
# while 1:
#     events = get_gamepad()
#     for event in events:
#         if event.code == "ABS_X":
#             print(event.ev_type, event.code, event.state)

class game_pad_lisener:
    def get_data(self, event_code):
        events = get_gamepad()
        for event in events:
            if event.code == event_code:
                return event.state

    def get_left_trigger(self):
        return self.get_data("ABS_Z")

    def get_right_trigger(self):
        return self.get_data("ABS_RZ")

    def get_left_joy_X(self):
        return self.get_data("ABS_X")

    def get_left_joy_Y(self):
        return self.get_data("ABS_Y")

    def get_right_joy_X(self):
        return  self.get_data("ABS_RX")

    def get_right_joy_Y(self):
        return self.get_data("ABS_RY")

gp = game_pad_lisener()
while True:
    rt_val = '0'
    lt_val = '0'
    rt = gp.get_right_trigger()
    lt = gp.get_left_trigger()
    if rt is not None :
        rt_val = str(rt)

    if lt is not None:
        lt_val = str(lt)
    print("left trigger" + lt_val)
    print("right trigger" + rt_val)
