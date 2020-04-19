import numpy as np
from narrow_window import Window
from Quad import Quad_Disc

class Enviroment():
    #Class defining the environment for planning 
    def __init__(self):

        #Initiate Quad_disc model
        self.Quad = Quad_Disc(0.3, 0.1)

        #initiate narrow windows for the environment 
        
        p1 = np.array([0,-0.35,-0.075])
        p2 = np.array([0,0.35,-0.075])
        p3 = np.array([0,0.35,0.075])
        p4 = np.array([0,-0.35,0.075])
        pts = [p1, p2, p3, p4]

        roll = -0.3
        T_roll = np.array([[1, 0, 0],
                     [0, np.cos(roll), -np.sin(roll)],
                     [0, np.sin(roll), np.cos(roll)]])
        pts1 = []

        for point in pts:
            pts1.append(T_roll.dot(point))
        T1= np.identity(4)
        T1[0:3,3] = np.array([4,-1,2])
        win1 = Window(1,1,1,T1, pts1)

        roll = 0.3
        T_roll = np.array([[1, 0, 0],
                     [0, np.cos(roll), -np.sin(roll)],
                     [0, np.sin(roll), np.cos(roll)]])
        pts2 = []
        for point in pts:
            pts2.append(T_roll.dot(point))

        T2= np.identity(4)
        T2[0:3,3] = np.array([4,3,2])
        win2 = Window(1,1,1,T2, pts2)

        roll = -0.2
        T_roll = np.array([[1, 0, 0],
                     [0, np.cos(roll), -np.sin(roll)],
                     [0, np.sin(roll), np.cos(roll)]])
        pts3 = []
        for point in pts:
            pts3.append(T_roll.dot(point))

        T3= np.identity(4)
        T3[0:3,3] = np.array([8,1,2])
        win3 = Window(1,1,1,T3, pts3)

        roll = 0.25
        T_roll = np.array([[1, 0, 0],
                     [0, np.cos(roll), -np.sin(roll)],
                     [0, np.sin(roll), np.cos(roll)]])
        pts4 = []
        for point in pts:
            pts4.append(T_roll.dot(point))

        T4= np.identity(4)
        T4[0:3,3] = np.array([8,3,2])
        win4 = Window(1,1,1,T4, pts4)

        self.windows = [win1, win2, win3, win4]

        #define circular obstacles
        self.obs_rad = 0.5
        self.obs_locs = [np.array([2,0.5]), np.array([6,2]), np.array([10,1.5])]

    def sample_parabolas(self, win, it, dt_des):
        ay = np.random.uniform(-3.5,3.5)
        vx = np.random.uniform(0,1)

        roll = np.arctan(-ay/9.81)
        T = np.array([[1, 0, 0, 0],
                     [0, np.cos(roll), -np.sin(roll),0],
                     [0, np.sin(roll), np.cos(roll),0],
                     [0,0,0,1]])
        collision = self.Quad.check_collision_with_window(T, win.polygon)

        if collision:
            return False, None, None
        else:
            state_init, node = win.generate_parabolic_nodes(it, dt_des, vx, ay)
            return True, state_init, node


    def check_in_window(self, p_rand):
        """Function that checks whether a random sample (p_rand) lies
        within the vicinity of any of the narrow windows

        Arg's:
            p_rand: random sample in Cartesian space 

        Returns: 
            True or false if sample is in window, along the window object 
        """

        #check if random point lies within window range
        for win in self.windows:
            if win.check_in_window(p_rand):
                return True, win

        return False, None

    def check_path_collision(self, X):
        """Function that checks the collision along the polynomial path
        generated 

        Arg's:
            X: 9xN matrix containing N states forming the trajectory 

        Return:
            True if collision exists, False otherwise
        """

        #check collision with circular obstacles
        for i in range(X.shape[1]):
            p = X[0:3,i].flatten()

            for obs_loc in self.obs_locs:
                if np.linalg.norm(p[0:2] - obs_loc) < self.obs_rad:
                    return True
            if i>0:
                p2 = X[0:3,i-1].flatten()
                #check collision with walls
                for win in self.windows:
                    if win.check_collision(p, p2):
                        return True

        return False
