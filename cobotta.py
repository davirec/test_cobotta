from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import time


class Cobotta:

    def __init__(self):
        # smartphone
        self.resolution = [1920, 1080]
        self.smartphone = [165, 76, 10]
        # reference points
        self.ref_touch_center = [-54.000000, 0.000000, 210.000000]
        self.joints_ref = [-50.000000, 25.000000, 125.000000, -55.000000, -72.000000, 25.000000]

        # smartphone points
        self.gap = None
        self.smartphone_sup_left = None
        self.smartphone_sup_right = None
        self.smartphone_inf_left = None
        self.smartphone_inf_right = None
        self.borda_scroll_up_down = None
        self.borda_scroll_left_right = None
        self.borda_touch = None
        self.distance_screen = None
        self.config_coord_device()

        # Connection to the simulator
        self.RDK = Robolink()
        self.robot = self.RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
        if not self.robot.Valid():
            raise Exception('No robot selected or available')

        # current position of the TCP with respect to the reference frame
        target_ref = self.robot.Pose()
        # pos_ref = target_ref.Pos()
        # move the robot to the first point:
        self.robot.MoveJ(target_ref)
        self.robot.MoveJ(self.joints_ref)

        # It is important to provide the reference frame and the tool frames when generating programs offline
        self.robot.setPoseFrame(self.robot.PoseFrame())
        self.robot.setPoseTool(self.robot.PoseTool())

        # Set the rounding parameter (Also known as: CNT, APO/C_DIS, ZoneData, Blending radius, cornering, ...)
        self.robot.setRounding(10)
        self.robot.setSpeed(200)  # Set linear speed in mm/s

    def config_coord_device(self):
        self.gap = 15
        self.borda_scroll_up_down = 40
        self.borda_scroll_left_right = 10
        self.borda_touch = 0
        self.distance_screen = 10

        self.smartphone_sup_left = [self.ref_touch_center[0] - self.gap,
                                    self.ref_touch_center[1] + (self.smartphone[1] / 2),
                                    self.ref_touch_center[2] - (self.smartphone[0] / 2)]
        self.smartphone_sup_right = [self.ref_touch_center[0] - self.gap,
                                     self.ref_touch_center[1] - (self.smartphone[1] / 2),
                                     self.ref_touch_center[2] - (self.smartphone[0] / 2)]
        self.smartphone_inf_left = [self.ref_touch_center[0] - self.gap,
                                    self.ref_touch_center[1] + (self.smartphone[1] / 2),
                                    self.ref_touch_center[2] + (self.smartphone[0] / 2)]
        self.smartphone_inf_right = [self.ref_touch_center[0] - self.gap,
                                     self.ref_touch_center[1] - (self.smartphone[1] / 2),
                                     self.ref_touch_center[2] + (self.smartphone[0] / 2)]

    def move_robot(self, pos_i):
        target_i = self.robot.Pose()
        target_i.setPos(pos_i)
        try:
            self.robot.MoveL(target_i)
        except TargetReachError as erro:
            print("\033[1;31m", erro, "\033[0;0m")
            self.move_robot([-64.000000, 0.000000, 210.000000])
            self.robot.MoveL(target_i)
        time.sleep(1)

    def touch(self, x, y):
        print('\033[41m' + "Touch " + str(x) + ", " + str(y)+'\033[0m')

        # bloqueia o x e y para as dimensoes do smartphone
        if x > self.smartphone[1]:
            x = self.smartphone[1]
        if y > self.smartphone[0]:
            y = self.smartphone[0]

        if x > self.smartphone[1] / 2:
            x0_y0 = [self.ref_touch_center[1] + self.smartphone[1] / 2 + self.borda_touch,
                     self.ref_touch_center[2] - self.smartphone[0] / 2 + self.borda_touch]
            start = [self.ref_touch_center[0] - self.distance_screen, x0_y0[0] - x, x0_y0[1] + y]
            touch_smartphone = [self.ref_touch_center[0], x0_y0[0] - x, x0_y0[1] + y]
            end = [self.ref_touch_center[0] - self.distance_screen, x0_y0[0] - x, x0_y0[1] + y]
            # print("aaaaaa")
        else:
            x0_y0 = [self.ref_touch_center[1] + self.smartphone[1] / 2 - self.borda_touch,
                     self.ref_touch_center[2] - self.smartphone[0] / 2 + self.borda_touch]
            start = [self.ref_touch_center[0] - self.distance_screen, x0_y0[0] - x, x0_y0[1] + y]
            touch_smartphone = [self.ref_touch_center[0], x0_y0[0] - x, x0_y0[1] + y]
            end = [self.ref_touch_center[0] - self.distance_screen, x0_y0[0] - x, x0_y0[1] + y]

        self.move_robot(start)
        self.move_robot(touch_smartphone)
        self.move_robot(end)

    def scroll_left(self):
        touch_left = self.ref_touch_center[1] + self.smartphone[1] / 2 - self.borda_scroll_left_right
        touch_right = self.ref_touch_center[1] - self.smartphone[1] / 2 + self.borda_scroll_left_right

        start = [self.ref_touch_center[0] - self.distance_screen, touch_right, self.ref_touch_center[2]]
        self.move_robot(start)

        touch_right_smartphone = [self.ref_touch_center[0], touch_right, self.ref_touch_center[2]]
        self.move_robot(touch_right_smartphone)

        touch_left_smartphone = [self.ref_touch_center[0], touch_left, self.ref_touch_center[2]]
        self.move_robot(touch_left_smartphone)

        end = [self.ref_touch_center[0] - self.distance_screen, touch_left, self.ref_touch_center[2]]
        self.move_robot(end)

    def scroll_right(self):
        touch_left = self.ref_touch_center[1] + self.smartphone[1] / 2 - self.borda_scroll_left_right
        touch_right = self.ref_touch_center[1] - self.smartphone[1] / 2 + self.borda_scroll_left_right

        start = [self.ref_touch_center[0] - self.distance_screen, touch_left, self.ref_touch_center[2]]
        self.move_robot(start)

        touch_right_smartphone = [self.ref_touch_center[0], touch_left, self.ref_touch_center[2]]
        self.move_robot(touch_right_smartphone)

        touch_left_smartphone = [self.ref_touch_center[0], touch_right, self.ref_touch_center[2]]
        self.move_robot(touch_left_smartphone)

        end = [self.ref_touch_center[0] - self.distance_screen, touch_right, self.ref_touch_center[2]]
        self.move_robot(end)

    def move_touch_home(self):
        self.robot.MoveJ(self.joints_ref)

    def move_touch_center(self):
        self.move_robot(self.ref_touch_center)

    def scroll_up(self):
        print('\033[41m' + "Scroll up" + '\033[0m')

        touch_down = self.ref_touch_center[2] + self.smartphone[0] / 2 - self.borda_scroll_up_down
        touch_up = self.ref_touch_center[2] - self.smartphone[0] / 2 + self.borda_scroll_up_down
        start = [self.ref_touch_center[0] - self.distance_screen, self.ref_touch_center[1], touch_down]
        self.move_robot(start)

        touch_down_smartphone = [self.ref_touch_center[0], self.ref_touch_center[1], touch_down]
        self.move_robot(touch_down_smartphone)

        touch_up_smartphone = [self.ref_touch_center[0], self.ref_touch_center[1], touch_up]
        self.move_robot(touch_up_smartphone)

        end = [self.ref_touch_center[0] - self.distance_screen, self.ref_touch_center[1], touch_up]
        self.move_robot(end)

    def scroll_down(self):
        print('\033[41m' + "Scroll down" + '\033[0m')

        touch_down = self.ref_touch_center[2] + self.smartphone[0] / 2 - self.borda_scroll_up_down
        touch_up = self.ref_touch_center[2] - self.smartphone[0] / 2 + self.borda_scroll_up_down
        start = [self.ref_touch_center[0] - self.distance_screen, self.ref_touch_center[1], touch_up]
        self.move_robot(start)

        touch_down_smartphone = [self.ref_touch_center[0], self.ref_touch_center[1], touch_up]
        self.move_robot(touch_down_smartphone)

        touch_up_smartphone = [self.ref_touch_center[0], self.ref_touch_center[1], touch_down]
        self.move_robot(touch_up_smartphone)

        end = [self.ref_touch_center[0] - self.distance_screen, self.ref_touch_center[1], touch_down]
        self.move_robot(end)

    def double_rotation(self, cod):
        print('\033[41m' + "Double rotation " + str(cod) + '\033[0m' )

        self.robot.MoveJ(self.joints_ref)
        portrait = [-50.000000, 25.000000, 125.000000, -55.000000, -72.000000, 25.000000]
        landscape = [-50.000000, 25.000000, 125.000000, -55.000000, -72.000000, -65.000000]

        if cod == 1:
            self.robot.MoveJ(landscape)
        else:
            self.robot.MoveJ(portrait)
            time.sleep(1)
            self.move_robot([-64.000000, 0.000000, 210.000000])


if __name__ == '__main__':
    robot_cobotta = Cobotta()
    robot_cobotta.touch(0, 165)
    robot_cobotta.double_rotation(0)
    time.sleep(1)
    robot_cobotta.double_rotation(1)
    time.sleep(1)
    robot_cobotta.double_rotation(0)
    time.sleep(1)
    robot_cobotta.scroll_down()
    robot_cobotta.scroll_up()
    robot_cobotta.move_touch_home()
