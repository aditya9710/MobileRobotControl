"""
challenge.py
"""
import time
from typing import List, Tuple, Union

import numpy as np
import pygame
import unittest
import roboticstoolbox as rtb


class Robot:
    JOINT_LIMITS = [-6.28, 6.28]
    MAX_VELOCITY = 15
    MAX_ACCELERATION = 50
    DT = 0.033

    link_1: float = 65.  # pixels
    link_2: float = 50.  # pixels
    link_3: float = 20.  # pixels
    _theta_0: float      # radians
    _theta_1: float      # radians
    _theta_2: float      # radians
    _y_position_error_term = 7.32

    def __init__(self) -> None:
        # internal variables
        self.all_theta_0: List[float] = []
        self.all_theta_1: List[float] = []
        self.all_theta_2: List[float] = []

        self.theta_0 = 0.
        self.theta_1 = 0.
        self.theta_2 = 0.

    # Getters/Setters
    @property
    def theta_0(self) -> float:
        return self._theta_0

    @theta_0.setter
    def theta_0(self, value: float) -> None:
        self.all_theta_0.append(value)
        self._theta_0 = value
        # Check limits
        assert self.check_angle_limits(value), \
            f'Joint 0 value {value} exceeds joint limits'
        assert self.max_velocity(self.all_theta_0) < self.MAX_VELOCITY, \
            f'Joint 0 Velocity {self.max_velocity(self.all_theta_0)} exceeds velocity limit'
        assert self.max_acceleration(self.all_theta_0) < self.MAX_ACCELERATION, \
            f'Joint 0 Accel {self.max_acceleration(self.all_theta_0)} exceeds acceleration limit'

    @property
    def theta_1(self) -> float:
        return self._theta_1

    @theta_1.setter
    def theta_1(self, value: float) -> None:
        self.all_theta_1.append(value)
        self._theta_1 = value
        assert self.check_angle_limits(value), \
            f'Joint 1 value {value} exceeds joint limits'
        assert self.max_velocity(self.all_theta_1) < self.MAX_VELOCITY, \
            f'Joint 1 Velocity {self.max_velocity(self.all_theta_1)} exceeds velocity limit'
        assert self.max_acceleration(self.all_theta_1) < self.MAX_ACCELERATION, \
            f'Joint 1 Accel {self.max_acceleration(self.all_theta_1)} exceeds acceleration limit'

    @property
    def theta_2(self) -> float:
        return self._theta_2

    @theta_2.setter
    def theta_2(self, value: float) -> None:
        self.all_theta_2.append(value)
        self._theta_2 = value
        assert self.check_angle_limits(value), \
            f'Joint 2 value {value} exceeds joint limits'
        assert self.max_velocity(self.all_theta_2) < self.MAX_VELOCITY, \
            f'Joint 2 Velocity {self.max_velocity(self.all_theta_2)} exceeds velocity limit'
        assert self.max_acceleration(self.all_theta_2) < self.MAX_ACCELERATION, \
            f'Joint 2 Accel {self.max_acceleration(self.all_theta_2)} exceeds acceleration limit'

    # Kinematics
    def joint_1_pos(self) -> Tuple[float, float]:
        """
        Compute the x, y position of joint 1
        """
        return self.link_1 * np.cos(self.theta_0), self.link_1 * np.sin(self.theta_0), self.theta_0

        # PT = [[self.theta_0, 0, self.link_2, self.link_1],
        #     [self.theta_1, 0, 0, 0],
        #     [self.theta_2, 0, self.link_3, 0]]

        # i = 0
        # self.H0_1 = [[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])],
        #         [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
        #         [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[i][3]],
        #         [0, 0, 0, 1]]

        # return self.H0_1[0][3], self.H0_1[1][3], self.theta_0

    def joint_2_pos(self) -> Tuple[float, float]:
        """
        Compute the x, y position of joint 2
        """
        x = self.link_1 * np.cos(self.theta_0) + self.link_2 * np.cos(self.theta_0 + self.theta_1)
        y = self.link_1 * np.sin(self.theta_0) + self.link_2 * np.sin(self.theta_0 + self.theta_1)

        return x, y, self.theta_0 + self.theta_1
        # PT = [[self.theta_0, 0, self.link_2, self.link_1],
        #     [self.theta_1, 0, 0, 0],
        #     [self.theta_2, 0, self.link_3, 0]]

        # i = 1
        # self.H1_2 = [[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])],
        #         [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
        #         [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[i][3]],
        #         [0, 0, 0, 1]]

        # self.H0_2 = np.dot(self.H0_1, self.H1_2)
        # return self.H0_2[0][3], self.H0_2[1][3], self.theta_0 + self.theta_1

    def joint_3_pos(self) -> Tuple[float, float, float]:
        """
        Compute the x, y position of joint 3
        """

        print(f'FK: {self.forward(self.theta_0, self.theta_1, self.theta_2)}')
        return self.forward(self.theta_0, self.theta_1, self.theta_2)

        # PT = [[self.theta_0, 0, self.link_2, self.link_1],
        #     [self.theta_1, 0, 0, 0],
        #     [self.theta_2, 0, self.link_3, 0]]

        # i = 2
        # self.H2_3 = [[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])],
        #         [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
        #         [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[i][3]],
        #         [0, 0, 0, 1]]

        # self.H0_3 = np.dot(self.H0_2,self.H2_3)
        # return self.H2_3[0][3], self.H2_3[1][3], self.theta_0 + self.theta_1 + self.theta_2


    @classmethod
    def forward(cls, theta_0: float, theta_1: float, theta_2: float) -> Tuple[float, float, float]:
        """
        Compute the x, y position of the end of the links from the joint angles
        """
        x = cls.link_1 * np.cos(theta_0) + cls.link_2 * np.cos(theta_0 + theta_1) + \
            cls.link_3 * np.cos(theta_0 + theta_1 + theta_2)
        y = cls.link_1 * np.sin(theta_0) + cls.link_2 * np.sin(theta_0 + theta_1) + \
            cls.link_3 * np.cos(theta_0 + theta_1 + theta_2) - cls._y_position_error_term

        print(f'Y Position: {y}')
        phi = theta_0 + theta_1 + theta_2
        return x, y, phi

    @classmethod
    def inverse(cls, px: float, py: float, phi: float) -> Tuple[float, float]:
        """
        Compute the joint angles from the position of the end of the links
        """
        wx = px - cls.link_3 * np.cos(phi)
        wy = py - cls.link_3 * np.sin(phi)

        delta = wx**2 + wy**2
        c2 = (delta - cls.link_1**2 - cls.link_2**2)/(2*cls.link_1*cls.link_2)
        s2 = np.sqrt(1 - c2**2)  # elbow down
        theta_1 = np.arctan2(s2, c2)

        s1 = ((cls.link_1 + cls.link_2 * c2) * wy - cls.link_2 * s2 * wx)/delta
        c1 = ((cls.link_1 + cls.link_2 * c2) * wx + cls.link_2 * s2 * wy)/delta
        
        theta_0 = np.arctan2(s1,c1)
        theta_2 = phi - theta_0 - theta_1
        
        print(f'Theta 0: {theta_0}')
        print(f'Theta 1: {theta_1}')
        print(f'Theta 2: {theta_2}')
        # theta_1 = np.arccos((x ** 2 + y ** 2 - cls.link_1 ** 2 - cls.link_2 ** 2)
        #                     / (2 * cls.link_1 * cls.link_2))
        # theta_0 = np.arctan2(y, x) - \
        #     np.arctan((cls.link_2 * np.sin(theta_1)) /
        #               (cls.link_1 + cls.link_2 * np.cos(theta_1)))

        return theta_0, theta_1, theta_2

    @classmethod
    def check_angle_limits(cls, theta: float) -> bool:
        return cls.JOINT_LIMITS[0] < theta < cls.JOINT_LIMITS[1]

    @classmethod
    def max_velocity(cls, all_theta: List[float]) -> float:
        return float(max(abs(np.diff(all_theta) / cls.DT), default=0.))

    @classmethod
    def max_acceleration(cls, all_theta: List[float]) -> float:
        return float(max(abs(np.diff(np.diff(all_theta)) / cls.DT / cls.DT), default=0.))

    @classmethod
    def min_reachable_radius(cls) -> float:
        return max(cls.link_1 - cls.link_2 - cls.link_3, 0)

    @classmethod
    def max_reachable_radius(cls) -> float:
        return cls.link_1 + cls.link_2 + cls.link_3


class World:
    def __init__(
        self,
        width: int,
        height: int,
        robot_origin: Tuple[int, int],
        goal: Tuple[int, int, float]
    ) -> None:
        self.width = width
        self.height = height
        self.robot_origin = robot_origin
        self.goal = goal
        self.goal_color = (255, 20, 20)
        self.goal_speed = (-0.1, .1)
        self.wall_width = 5

        print(f'Goal in world: {goal}')

    def convert_to_display(
            self, point: Tuple[Union[int, float], Union[int, float]]) -> Tuple[int, int]:
        """
        Convert a point from the robot coordinate system to the display coordinate system
        """
        robot_x, robot_y, robot_pose = point
        offset_x, offset_y = self.robot_origin

        return int(offset_x + robot_x), int(offset_y - robot_y)


class Visualizer:
    BLACK: Tuple[int, int, int] = (0, 0, 0)
    RED: Tuple[int, int, int] = (255, 0, 0)
    GREEN: Tuple[int, int, int] = (0, 255, 0)
    WHITE: Tuple[int, int, int] = (255, 255, 255)
    BROWN: Tuple[int, int, int] = (165, 42, 52)

    screen_extension = 130 # pixels

    def __init__(self, world: World) -> None:
        """
        Note: while the Robot and World have the origin in the center of the
        visualization, rendering places (0, 0) in the top left corner.
        """
        pygame.init()
        pygame.font.init()
        self.world = world
        self.screen = pygame.display.set_mode((world.width, world.height + self.screen_extension))
        pygame.display.set_caption('3R Planar Robot Manipulator')
        self.font = pygame.font.SysFont('freesansbolf.tff', 30)
        self.wall = pygame.draw.rect(self.screen, self.WHITE, pygame.Rect(100, 20, 20, 40))
        self.arm_link_1 = pygame.draw.rect(self.screen, self.WHITE, pygame.Rect(0, 0, 0, 0))
        self.arm_link_2 = pygame.draw.rect(self.screen, self.WHITE, pygame.Rect(0, 0, 0, 0))
        self.arm_link_3 = pygame.draw.rect(self.screen, self.WHITE, pygame.Rect(0, 0, 0, 0))
        
    def display_world(self) -> None:
        """
        Display the world
        """
        goal = self.world.convert_to_display((self.world.goal[0], self.world.goal[1], 90))
        self.goal_circ = pygame.draw.circle(self.screen, self.world.goal_color, goal, 6)
        Border1 = pygame.draw.rect(self.screen, self.BLACK, pygame.Rect(0, 0, self.world.wall_width, self.world.height))
        Border2 = pygame.draw.rect(self.screen, self.BLACK, pygame.Rect(0, self.world.width - self.world.wall_width, self.world.width, self.world.wall_width))
        Border3 = pygame.draw.rect(self.screen, self.BLACK, pygame.Rect(0, 0, self.world.width, self.world.wall_width))
        Border4 = pygame.draw.rect(self.screen, self.BLACK, pygame.Rect(self.world.width - self.world.wall_width, 0, self.world.wall_width, self.world.height))
        self.wall = pygame.draw.rect(self.screen, self.BROWN, pygame.Rect(100, 20, 20, 40))
        self.screen.blit(self.font.render('Wall', True, self.BROWN), (55, 20))

    def display_robot(self, robot: Robot) -> None:
        """
        Display the robot
        """
        j0 = self.world.robot_origin
        j1 = self.world.convert_to_display(robot.joint_1_pos())
        j2 = self.world.convert_to_display(robot.joint_2_pos())
        j3 = self.world.convert_to_display(robot.joint_3_pos())
        # Draw joint 0
        pygame.draw.circle(self.screen, self.BLACK, j0, 4)
        # Draw link 1
        self.arm_link_1 = pygame.draw.line(self.screen, self.BLACK, j0, j1, 2)
        # Draw joint 1
        pygame.draw.circle(self.screen, self.BLACK, j1, 4)
        # Draw link 2
        self.arm_link_2 = pygame.draw.line(self.screen, self.BLACK, j1, j2, 2)
        # Draw joint 2
        pygame.draw.circle(self.screen, self.BLACK, j2, 4)
        # Draw link 3
        self.arm_link_3 = pygame.draw.line(self.screen, self.BLACK, j2, j3, 2)
        # Draw joint 3
        pygame.draw.circle(self.screen, self.BLACK, j3, 4)

    def display_velocity_accleration(self, robot: Robot):
        j0_velocity_track = abs(np.diff(robot.all_theta_0))/robot.DT
        j0_velocity = j0_velocity_track[-1]
        j0_accl_track = np.diff(np.diff(robot.all_theta_0)/robot.DT)/robot.DT
        if len(j0_accl_track):
            j0_accl = j0_accl_track[-1]
        else:
            j0_accl = 0

        j1_velocity_track = abs(np.diff(robot.all_theta_1))/robot.DT
        j1_velocity = j1_velocity_track[-1]
        j1_accl_track = np.diff(np.diff(robot.all_theta_1)/robot.DT)/robot.DT
        if len(j1_accl_track):
            j1_accl = j1_accl_track[-1]
        else:
            j1_accl = 0

        j2_velocity_track = abs(np.diff(robot.all_theta_2))/robot.DT
        j2_velocity = j2_velocity_track[-1]
        j2_accl_track = np.diff(np.diff(robot.all_theta_2)/robot.DT)/robot.DT
        if len(j2_accl_track):
            j2_accl = j2_accl_track[-1]
        else:
            j2_accl = 0
        
        self.screen.blit(self.font.render('  Joint 0 velocity: ' + str("{:.4f}".format(j0_velocity)), 
            True, self.BLACK), (0, 305))
        self.screen.blit(self.font.render('  Joint 0 acceleration: ' + str("{:.4f}".format(j0_accl)), 
            True, self.BLACK), (0, 325))
        self.screen.blit(self.font.render('  Joint 1 velocity: ' + str("{:.4f}".format(j1_velocity)), 
            True, self.BLACK), (0, 345))
        self.screen.blit(self.font.render('  Joint 1 acceleration: ' + str("{:.4f}".format(j1_accl)), 
            True, self.BLACK), (0, 365))
        self.screen.blit(self.font.render('  Joint 2 velocity: ' + str("{:.4f}".format(j2_velocity)), 
            True, self.BLACK), (0, 385))
        self.screen.blit(self.font.render('  Joint 2 acceleration: ' + str("{:.4f}".format(j2_accl)), 
            True, self.BLACK), (0, 405))

    def update_display(self, robot: Robot, success: bool) -> bool:
        for event in pygame.event.get():
            # Keypress
            if event.type == pygame.KEYDOWN:
                # Escape key
                if event.key == pygame.K_ESCAPE:
                    return False
            # Window Close Button Clicked
            if event.type == pygame.QUIT:
                return False

        self.screen.fill(self.WHITE)
        self.display_world()
        self.display_robot(robot)
        self.display_velocity_accleration(robot)

        if success:
            text = self.font.render('Success!', True, (20,180,20))
            self.screen.blit(text, (self.world.width/2-45, 65))
            self.world.goal_speed = (0,0)
            self.world.goal_color = (30, 255, 30)

        pygame.display.flip()

        return True

    def cleanup(self) -> None:
        pygame.quit()


class Controller:
    def __init__(self, goal: Tuple[int, int]) -> None:
        self.goal = goal

        self.goal_theta_0, self.goal_theta_1, self.goal_theta_2 = \
            Robot.inverse(self.goal[0], self.goal[1], self.goal[2])

    def step(self, robot: Robot) -> Robot:
        """
        Simple P controller
        """
        theta_0_error = self.goal_theta_0 - robot.theta_0
        theta_1_error = self.goal_theta_1 - robot.theta_1
        theta_2_error = self.goal_theta_2 - robot.theta_2

        robot.theta_0 += theta_0_error / 10
        robot.theta_1 += theta_1_error / 10
        robot.theta_2 += theta_2_error / 10

        return robot


class Runner:
    def __init__(
        self,
        robot: Robot,
        # controller: Controller,
        world: World,
        vis: Visualizer
    ) -> None:
        self.robot = robot
        
        self.world = world
        self.vis = vis

    def run(self) -> None:
        running = True

        while running:
            self.controller = Controller(self.world.goal)
            # Step the controller
            self.robot = self.controller.step(self.robot)

            # Check collisions
            assert not self.vis.wall.colliderect(self.vis.arm_link_1), f'Collision Detected'

            assert not self.vis.wall.colliderect(self.vis.arm_link_2), f'Collision Detected'

            assert not self.vis.wall.colliderect(self.vis.arm_link_3), f'Collision Detected'

            # Moving Goal
            self.world.goal = (self.world.goal[0] + self.world.goal_speed[0], 
                self.world.goal[1] + self.world.goal_speed[1], self.world.goal[2])

            # Check success
            goal_pos = (self.world.goal[0], self.world.goal[1], self.world.goal[2])
            success = self.check_success(self.robot, goal_pos)


            # Update the display
            running = self.vis.update_display(self.robot, success)
            
            # sleep for Robot DT seconds, to force update rate
            time.sleep(self.robot.DT)

    @staticmethod
    def check_success(robot: Robot, goal: Tuple[int, int]) -> bool:
        """
        Check that robot's joint 2 is very close to the goal.
        Don't not use exact comparision, to be robust to floating point calculations.
        """
        print(f'Reached goal ?: {goal}')
        return np.allclose(robot.joint_3_pos(), goal, atol=1.1)

    def cleanup(self) -> None:
        self.vis.cleanup()


def generate_random_goal(min_radius: float, max_radius: float) -> Tuple[int, int]:
    """
    Generate a random goal that is reachable by the robot arm
    """
    # Ensure theta is not 0
    theta = (np.random.random() + np.finfo(float).eps) * 2 * np.pi
    # Ensure point is reachable
    r = np.random.uniform(low=min_radius, high=max_radius)

    x = int(r * np.cos(theta))
    y = int(r * np.sin(theta))

    return x, y


def main() -> None:
    height = 300
    width = 300

    robot_origin = (int(width / 2), int(height / 2))
    goal = generate_random_goal(Robot.min_reachable_radius(), Robot.max_reachable_radius())

    print (f'Goal: {goal}')

    # goal = (30, 80, 30 * np.pi/180)

    goal = (goal[0], goal[1], np.random.randint(-45, high=45) * np.pi/180)

    robot = Robot()
    controller = Controller(goal)
    world = World(width, height, robot_origin, goal)
    vis = Visualizer(world)
    runner = Runner(robot, world, vis)

    try:
        runner.run()
    except AssertionError as e:
        print(f'ERROR: {e}, Aborting.')
    except KeyboardInterrupt:
        pass
    finally:
        runner.cleanup()


if __name__ == '__main__':
    main()
