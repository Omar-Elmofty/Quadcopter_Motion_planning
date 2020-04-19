import numpy as np
from numpy.linalg import norm

class ConvexPolygon:
    def __init__(self,point_list):
        self.point_list = point_list
        self.normal_vec = self.get_normal_vec()
        self.check_on_same_plane()
        self.check_convexity()
        
    def get_normal_vec(self):
        v1 = self.point_list[0] - self.point_list[1]
        v2 = self.point_list[1] - self.point_list[2]
        n = np.cross(v1, v2)
        n = n / float(norm(n))
        return n
        
    def check_on_same_plane(self):
        in_plane = True
        for i in range(len(self.point_list)):
            v = self.point_list[i-1] - self.point_list[i]
            if abs(np.dot(v,self.normal_vec)) >0.01:
                in_plane = False
        if not(in_plane):
            warnings.warn("Polygon Points don't lie in a single plane")
    
    def check_convexity(self):
        convex = True
        for i in range(len(self.point_list)):
            v1 = self.point_list[i-2] - self.point_list[i-1]
            v2 = self.point_list[i-1] - self.point_list[i]
            v_c = np.cross(v1, v2)
            if np.dot(v_c.T, self.normal_vec) < 0:
                convex = False
        if not(convex):
            warnings.warn("Polygon not convex")
            
    def check_in_polygon(self, p):
        in_polygon = True
        #project point onto the plane of the polygon
        p1 = self.point_list[0]
        v = p - p1
        y = np.cross(v, self.normal_vec)
        x = np.cross(self.normal_vec, y)
        x = x/np.linalg.norm(x)
        v_proj = np.dot(v.T, x)*x
        p_proj = p1 + v_proj

        #Create planes for checking inside polygon 
        for i in range(len(self.point_list)):
            v = self.point_list[i] - self.point_list[i-1]
            n = np.cross(self.normal_vec, v)
            vec = p_proj - self.point_list[i]
            if np.dot(vec.T, n) < 0:
                in_polygon = False   
                
        return in_polygon



class Window():
    def __init__(self, x_w, y_w, z_w, T, pts):
        #Window parameters
        self.T_w_i = T
        self.T_i_w = np.linalg.inv(T)
        self.x_width = x_w
        self.y_width = y_w
        self.z_width = z_w

        #Create window polygon
        self.polygon = ConvexPolygon(pts)

  
    def check_in_window(self, position):
        #put p in homogeneous form
        p = np.ones(4)
        p[0:3] = position
        p = p.reshape(-1,1)
        
        #express position in local coordinates
        p_local = self.T_i_w.dot(p)
        
        if abs(p_local[0,0]) < self.x_width/2. and  \
            abs(p_local[1,0]) < self.y_width/2. and  \
            abs(p_local[2,0]) < self.z_width/2. :
            return True
        else:
            return False
        projector
    def generate_parabolic_nodes(self, node_key, dt_des, vx, y_acc):
        
        #generate the nodes at x= x_width /2. and - x_width /2.
        x1 = self.x_width /4.
        x2 = - self.x_width /4.
        y1 = y_acc / 2.0 * x1**2 / vx**2
        y2 = y_acc / 2.0 * x2**2 / vx**2
        vy1 = y_acc * x1 / vx
        vy2 = y_acc * x2 / vx
        point1 = np.array([x1, y1, 0, 1]).reshape(-1,1)
        point2 = np.array([x2, y2, 0, 1]).reshape(-1,1)
        vel1 = np.array([vx, vy1, 0])
        vel2 = np.array([vx, vy2, 0])
        acc = np.array([0, y_acc, 0])
        
        #convert to global coordinates
        point1_global = self.T_w_i.dot(point1).flatten()
        point2_global = self.T_w_i.dot(point2).flatten()
        vel1_global = self.T_w_i[0:3,0:3].dot(vel1).flatten()
        vel2_global = self.T_w_i[0:3,0:3].dot(vel2).flatten()
        acc_global = self.T_w_i[0:3,0:3].dot(acc).flatten()
        
        #put everything in state form
        state1 = np.concatenate((point1_global[0:3], vel1_global, acc_global))
        state2 = np.concatenate((point2_global[0:3], vel2_global, acc_global))
        
        #calculate total points needed for path
        t_total = self.x_width / 2. / float(vx)
        N = int(t_total / float(dt_des))

        #calculate positions
        x = np.linspace(x1, x2, N)
        y = y_acc / 2.0 * x**2 / vx**2
        z = np.zeros(N)

        #calculate velocities 
        vx_list = vx*np.ones(N)
        vy_list = y_acc * x / vx
        vz_list = np.zeros(N)

        #calculate accelerations
        ax_list = np.zeros(N)
        ay_list = y_acc * np.ones(N)
        az_list = np.zeros(N)
        
        #calculate cost
        dx = x[1:N] - x[0:N-1]
        dy = y[1:N] - y[0:N-1]

        ds2 = dx**2 + dy**2 
        ds = np.sqrt(ds2)
        s = np.sum(ds)

        v2 = vx_list**2 + vy_list**2
        v = np.sum(np.sqrt(v2))

        #Combine distance and input to cost
        cost = 1.0 * s + 100 * v 
        
        #generate node randomly
        states = [state1, state2]
        [i1, i2] = np.random.choice([0,1], 2, replace=False)
        state_init = states[i1]
        state_final = states[i2]
        
        #pack everything in state form
        X = np.ones((10,N))
        X[0,:] = x.flatten()
        X[1,:] = y.flatten()
        X[2,:] = z.flatten()
        #row 3 is for homogeneous form
        X[4,:] = vx_list.flatten()
        X[5,:] = vy_list.flatten()
        X[6,:] = vz_list.flatten()
        X[7,:] = ax_list.flatten()
        X[8,:] = ay_list.flatten()
        X[9,:] = az_list.flatten()

        #Convert to global coordinates
        M_T = np.zeros((10,10))
        M_T[0:4,0:4] = self.T_w_i
        M_T[4:7,4:7] = self.T_w_i[0:3,0:3]
        M_T[7:10,7:10] = self.T_w_i[0:3,0:3]

        X_global = M_T.dot(X)

        X_final = np.zeros((9,N))
        X_final[0:3,:] = X_global[0:3, :]
        X_final[3:, :] = X_global[4:, :]

        #reverse the directions of points if applicable
        if state_final[0] == state1[0]:
            for i in range(9):
                x = X_final[i,:]
                X_final[i,:] = x[::-1]

        #Remove last node to avoid repetition of points
        X_final = X_final[:,0:N-1]
        
        node = {'parent':node_key,'state':state_final, 'cost': cost, 'path': X_final, 'free':False}
        
        
        return state_init, node

    def check_collision(self, state1, state2):

        p = self.T_w_i[0:3,3]
        n = self.T_w_i[0:3,0]

        dp1 = state1[0:3] - p.flatten()
        dp2 = state2[0:3] - p.flatten()
        
        a1 = dp1.dot(n)
        a2 = dp2.dot(n)
        
        if a1/float(a2) > 0:
            return False
        return True
