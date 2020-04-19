
# Import libraries
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from helper_functions import nearby_nodes, get_total_cost
from min_snap import min_snap_constrained, min_snap_unconstrained
from steer import steer
from enviroment import Enviroment




class TrajectoryPlanner(object):
    #Define a class for overall trajectory planner
    def __init__(self):

        #Initiate Graph
        self.G = {0:{'parent':None,'state':np.array([0, 0, 2, 0, 0, 0, 0, 0, 0]), 
                    'cost': 0, 'path':None, 'free':True}}

        #define Goal state
        self.state_goal = np.array([12, 3, 2, 0, 0, 0, 0, 0, 0])

        #Define some global parameters 
        self.r_search = 2
        self.max_iter = 1000
        self.dt_des = 1/10.0 #this is the mpc controller's delta time

        #Select RRT* steer function order
        self.steer_order = 1
        self.optimization_type = 'constrained'

        #define search range for RRT*
        self.x_range = [0,13]
        self.y_range = [-2,4]
        self.z_range = [0,6]

        #Create environment 
        self.env = Enviroment()


    def rrt_plan(self):
        """Function that Runs RRT* with parabolic sampling in the vicinity
        of narrow windows

        Returns:
            best path to the goal
        """

        #Iterate to get converges state
        for it in range(1,self.max_iter):
            if it%100 == 0:
                print('iteration = ', it)

            #Random sample random point
            state_rand = np.zeros(9)
            state_rand[0] = np.random.uniform(self.x_range[0],self.x_range[1])
            state_rand[1] = np.random.uniform(self.y_range[0],self.y_range[1])
            state_rand[2] = np.random.uniform(self.z_range[0],self.z_range[1])
            #sample velocity 
            angle_rand = np.random.uniform(-np.pi/2., np.pi/2.)
            state_rand[3:5] = np.array([np.cos(angle_rand), np.sin(angle_rand)])

            #randomly sample a window every 10 iterations, else random rest of space
            if it%10 == 0:
                win = np.random.choice(self.env.windows)
                #state_rand, next_node = win.generate_parabolic_nodes(it, self.dt_des)
                found_path, state_rand, next_node = self.env.sample_parabolas(win, it, self.dt_des)
                in_win = True
                if not(found_path):
                    continue
            else:
                #check if in window
                in_win, win = self.env.check_in_window(state_rand[0:3])
                if in_win:
                    #state_rand, next_node = win.generate_parabolic_nodes(it, self.dt_des)
                    found_path, state_rand, next_node = self.env.sample_parabolas(win, it, self.dt_des)
                    if not(found_path):
                        continue
             
            #look for nearby nodes
            Near_nodes, Nearest_node, key_Nearest_node = nearby_nodes(state_rand,self.G,self.r_search)
            
            #Connect to best node
            min_cost = float('inf')
            best_node = None
            for key in Near_nodes.keys():
                node = self.G[key]
                
                stage_cost, X = steer(node['state'], state_rand, self.dt_des, self.steer_order)

                if self.env.check_path_collision(X):
                    continue

                total_cost = stage_cost + get_total_cost(self.G, key)
                
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_node = key
                    best_path = X
                    best_cost = stage_cost
            
            #Continue if node is not found
            if best_node == None:
                continue

            #Wire new node
            self.G[it] = {'parent':best_node,'state':state_rand, 'cost':best_cost, 'path':best_path, 'free': True}  

            #Wire next node if in window
            if in_win:
                self.G[-it] = next_node

            #rewire close nodes to reduce cost
            for key in Near_nodes.keys():
                node = self.G[key]
                    
                stage_cost, X = steer(state_rand,node['state'], self.dt_des, self.steer_order)

                if self.env.check_path_collision( X):
                    continue

                total_cost = stage_cost + get_total_cost(self.G, it)
                
                if total_cost < node['cost']:
                    self.G[key]['parent'] = it
                    self.G[key]['cost'] = stage_cost
                    self.G[key]['path'] = X
        

        #find best node to connect to goal
        min_cost = float('inf')
        best_node = None
        Near_nodes,Nearest_node, key_Nearest_node = nearby_nodes(self.state_goal,self.G,self.r_search)
        for key in Near_nodes.keys():
            node = self.G[key]
                    
            stage_cost, X = steer(node['state'],self.state_goal, self.dt_des, self.steer_order)

            if self.env.check_path_collision(X):
                continue

            total_cost = stage_cost + get_total_cost(self.G, key)
            
            if total_cost < min_cost:
                min_cost = total_cost
                best_node = key
                best_path = X
                    
        #wire goal state
        self.G['goal'] = {'parent':best_node,'state':self.state_goal, 'cost':min_cost, 'path':best_path, 'free':True}  

        #generate best path
        best_path = [self.G['goal']]
        parent = best_node
        while parent != None:
            best_path.append(self.G[parent])
            parent = self.G[parent]['parent']

        return best_path
    
    def plot_path(self, best_path, traj):
        """Function for plotting the results
        """

        #Plotting Results:
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        plt.title('Graph')
        for obs in self.env.obs_locs:
            circle = plt.Circle((obs[0], obs[1]), self.env.obs_rad, color='r')
            ax.add_artist(circle)
        for key in self.G.keys():
            pos = self.G[key]['state'][0:3]
            ax.plot([pos[0]],[pos[1]],[pos[2]],'ro')
            parent_key = self.G[key]['parent']
            if parent_key != None:
                parent_pos = self.G[parent_key]['state'][0:3]
                ax.plot([pos[0],parent_pos[0]],[pos[1],parent_pos[1]],[pos[2], parent_pos[2]],'b')

        plt.xlim(self.x_range)
        plt.ylim(self.y_range)

        #plot the shortest path
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        
        for i in range(len(best_path)-1):
            x = best_path[i]['path'][0,:]
            y = best_path[i]['path'][1,:]
            z = best_path[i]['path'][2,:]
            ax.plot(x,y,z,'b')

       
        ax.plot(traj[0,:], traj[1,:],traj[2,:], 'b.')
        plt.title('Overall Trajectory 3D')

        fig, ax = plt.subplots()
        for obs in self.env.obs_locs:
            circle = plt.Circle((obs[0], obs[1]), self.env.obs_rad, color='r')
            ax.add_artist(circle)
        ax.plot(traj[0,:], traj[1,:], 'b.')
        plt.title('Overall Trajectory 2D')

        #additional plots
        # plt.figure()
        # plt.title('x-pos')
        # plt.plot(traj[0,:])
        # plt.figure()
        # plt.title('y-pos')
        # plt.plot(traj[1,:])
        # plt.figure()
        # plt.title('x-vel')
        # plt.plot(traj[3,:])
        # plt.figure()
        # plt.title('y-vel')
        # plt.plot(traj[4,:])
        # plt.figure()
        # plt.title('x-acc')
        # plt.plot(traj[6,:])
        # plt.figure()
        # plt.title('y-acc')
        # plt.plot(traj[7,:])
        
        plt.show()


    def lazy_states_contraction(self, best_path):
        """Implementation of lazy states contraction algorithm, prunes 
        the path by removing any lazy states

        Arg's: 
            best_path: list of nodes forming the best path
        Returns:
            best_path: pruned best_path

        """
        #lazy states contraction
        curr_idx = 0
        mid_idx = 1
        next_idx = 2
        while next_idx < len(best_path):
            node1 = best_path[curr_idx]
            node2 = best_path[next_idx]
            
            _, X = steer(node2['state'],node1['state'], self.dt_des, self.steer_order)
            
            if self.env.check_path_collision( X):
                curr_idx += 1
                mid_idx = curr_idx + 1
                next_idx = curr_idx + 2
                continue
            
            best_path.pop(mid_idx)
            best_path[curr_idx]['path'] = X

    

        return best_path


    def min_snap_trajectory(self,best_path):
        """Function that generates the minimum snap trajectory

        Arg's: 
            best_path: list of nodes forming the best path

        Returns:
            traj: a 9xN matrix forming the min snap trajectory
            solution_found: true if a solution was found, False otherwise
            s: total distance of trajectory
        """
        print('Generating minimum snap trajectory')
        traj = None
        i = 0
        solution_found = True
        while i < (len(best_path)-1):

            #if node[i] has no path
            if best_path[i]['free']:
                state_final = best_path[i]['state'] 
                int_points = [] 
                int_nodes = []
                for j in range(i+1, len(best_path)):
                    if best_path[j]['free']:
                        if j+1 == len(best_path):
                            state_init = best_path[j]['state'] 
                            break
                        int_points.append(best_path[j]['state'][0:3])
                        int_nodes.append(best_path[j])
                        continue
                    else:
                        state_init = best_path[j]['state'] 
                        break
                n_int = len(int_points)

                if self.optimization_type == 'constrained':
                    s, X = min_snap_constrained(state_init, state_final, int_points, self.dt_des)
                elif self.optimization_type == 'unconstrained':
                    s, X = min_snap_trajectory(state_init, state_final, int_points, self.dt_des)
                else:
                    raise Exception('optimization_type not defined')

                #Check min snap trajectory collision
                div = 2
                while self.env.check_path_collision(X) and div < 10:
                    print('Collision Detected, adding midpoints')

                    #add intermediate points
                    int_points = []
                    N = len(int_nodes)
                    for j in range(N-1,-1,-1):
                        for k in range(1,div):
                            factor = k/float(div)
                            int_idx = int(int_nodes[j]['path'].shape[1] * factor)
                            p_mid = int_nodes[j]['path'][0:3,int_idx]
                            int_points.append(p_mid)
                        int_points.append(int_nodes[j]['state'][0:3])

                    for k in range(1,div):
                        factor = k/float(div)
                        int_idx = int(best_path[i]['path'].shape[1] * factor)
                        p_mid = best_path[i]['path'][0:3,int_idx]
                        int_points.append(p_mid)
                    
                    #recalculate path using intermediate points
                    if self.optimization_type == 'constrained':
                        s, X = min_snap_constrained(state_init, state_final, int_points, self.dt_des)
                    elif self.optimization_type == 'unconstrained':
                        s, X = min_snap_trajectory(state_init, state_final, int_points, self.dt_des)
                    else:
                        raise Exception('optimization_type not defined')

                    div += 1

                if div == 10:
                    solution_found = False

                i+=1 + n_int
                if traj is None:
                    traj = X
                else:
                    traj = np.concatenate((X, traj), axis=1)
            
            else:
                X =  best_path[i]['path'] 
                i+=1
                if traj is None:
                    traj = X
                else:
                    traj = np.concatenate((X, traj), axis=1)

        #Calculate total path length
        if traj is not None:
            N = traj.shape[1]
            x  = traj[0,:]
            y  = traj[1,:]
            z  = traj[2,:]

            dx = x[1:N] - x[0:N-1]
            dy = y[1:N] - y[0:N-1]
            dz = z[1:N] - z[0:N-1]
            
            ds2 = dx**2 + dy**2 + dz**2
            ds = np.sqrt(ds2)
            s = np.sum(ds)
        else:
            s = None

        return traj, solution_found, s

    # def publish_path(self, traj):
    """Function for publishing path to ROS"""
    #     rate = rospy.Rate(10)

    #     for i in range(traj.shape[1]):
    #         rrt_point = traj[:,i].flatten()
    #         self.rrt_msg.flat_state = list(rrt_point)
    #         self.pub_rrt.publish(self.rrt_msg)
    #         rate.sleep()


    #     print('RRT_star_done')
    #     comp_rrt = 1
    #     self.pub_send_comp.publish(comp_rrt)
