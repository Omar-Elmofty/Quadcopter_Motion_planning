from Trajectory_Planner import TrajectoryPlanner
import time
import argparse

def parse_args(planner):
    """Function that sets global variables for planner
    """

    parser = argparse.ArgumentParser()
    parser.add_argument('--steer_order', help='polynomial order of steer function',
                            type = int, default=1)
    parser.add_argument('--search_radius', help='search radius for RRT*',
                            type=float, default=2.0)
    parser.add_argument('--max_iter', help='max iterations for the planner',
                            type=int, default=1000)
    parser.add_argument('--optim_type', help='max iterations for the planner',
                            type=str, default='constrained')
    parser.add_argument('--dt_des', help='desired delta t for controller',
                            type=float, default=1/10.0)

    args = parser.parse_args()

    planner.steer_order = args.steer_order
    planner.r_search = args.search_radius
    planner.max_iter = args.max_iter
    planner.optimization_type = args.optim_type
    planner.dt_des = args.dt_des

    return planner



if __name__ == '__main__':
    
    start = time.time()
    planner = TrajectoryPlanner()
    planner = parse_args(planner)

    best_path = planner.rrt_plan()
    best_path = planner.lazy_states_contraction(best_path)
    traj, solution_found, length = planner.min_snap_trajectory(best_path)
    end = time.time()
    if solution_found or (traj is not None):
        print('Solution Found :D')
        print('Total_time = ', end - start)
        print('Total_length = ', length)
        planner.plot_path(best_path, traj)
    else:
        print('Solution not found :(')
    
    