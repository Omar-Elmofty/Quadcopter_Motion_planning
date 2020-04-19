import numpy as np
from numpy.linalg import norm


def line_plane_intersection(Q, n, p1, p2):
    """Function that returns the intersection point
    of a line with a plane
    
    Arg's:
        Q: point on plane
        n: normal vector to plane
        p1: point 1 defining the line
        p2: point 2 defining the line
        
    Returns:
        p_int: intersection point of line with plane    
    """
    k = (p2 - p1) / np.linalg.norm(p2 - p1)
    f = (Q - p1).dot(n)/float(k.dot(n))
    p_int = p1 + f * k
    
    return p_int

def distance_point_to_line(C, p1, p2):
    """Function that computes the closest distance from
    a point to a line segment
    
    Arg's:
        C: point in space
        p1: point 1 defining the line segment
        p2: point 2 defining the line segment
    
    Returns:
        The minimum distance between the line segment and point        
    """
    n = np.cross(p2 - C, p1 - C)
    k = np.cross(n, p2 - p1)
    v1 = np.cross(k, p2 - C)
    v2 = np.cross(k, p1 - C)
    if np.dot(v1, v2) > 0:
        return min(np.linalg.norm(p1 - C),np.linalg.norm(p2 - C))
    else:
        D = norm(p1 - C)
        B = norm(p2 - C)
        A = norm(p2 - p1)
        
        eta = 0.5 - (B**2 - D**2)/(2.0*A**2)
        
        d = np.sqrt(D**2 - eta**2 * A**2)
        return d


class Quad_Disc:
    def __init__(self, radius, height):
        self.radius = radius
        self.height = height
        
    def check_collision_with_line(self,T, p1, p2):
        """Function that checks if the disc model of 
        quad collides with a line segment in space
        
        Arg's:
            T: transformation matrix of quad in space
            p1: point 1 of the line segment
            p2: point 2 of the line segment
            
        Returns:
            True or False indicating if collision exists"""
        #Define normal vectors of plane
        Qn = T[0:3,2]
        Qpos = T[0:3,3]
        Qp_up = Qpos + self.height/2.0 * Qn
        Qp_down = Qpos - self.height/2.0 * Qn
        
        p1_up = np.dot(p1 - Qp_up, Qn) > 0
        p1_down = np.dot(p1 - Qp_down, -Qn) > 0
        
        p2_up = np.dot(p2 - Qp_up, Qn) > 0
        p2_down = np.dot(p2 - Qp_down, -Qn) > 0
        
        p1_mid = not(p1_up) and not(p1_down)
        p2_mid = not(p2_up) and not(p2_down)
        
        #Case 1: No iteresections
        if (p1_up and p2_up) or (p1_down and p2_down):
            return False
        
        elif p1_mid and p2_mid:
            if distance_point_to_line(Qpos, p1, p2) < self.radius:
                return True
            else:
                return False
            
        #Case 2: Intersecting both planes
        elif (p1_up or p2_up) and (p1_down or p2_down):
            p_int_up = line_plane_intersection(Qp_up, Qn, p1, p2)
            p_int_down = line_plane_intersection(Qp_down, -Qn, p1, p2)
            if distance_point_to_line(Qpos, p_int_up, p_int_down) < self.radius:
                return True
            else:
                return False                        
        
        #Case 3: intersection with upper plane
        elif (p1_up or p2_up) and (p1_mid or p2_mid):
            p_int = line_plane_intersection(Qp_up, Qn, p1, p2)
            if p1_mid and distance_point_to_line(Qpos, p1, p_int) < self.radius:
                return True
            elif p2_mid and distance_point_to_line(Qpos, p2, p_int) < self.radius:
                return True
            else:
                return False
            
        #Case 4: intersection with lower plane
        elif (p1_down or p2_down) and (p1_mid or p2_mid):
            p_int = line_plane_intersection(Qp_down, -Qn, p1, p2)
            if p1_mid and distance_point_to_line(Qpos, p1, p_int) < self.radius:
                return True
            elif p2_mid and distance_point_to_line(Qpos, p2, p_int) < self.radius:
                return True
            else:
                return False

    def check_collision_with_window(self, T, window):
        """Function that checks if quad collides with wall having
        a window (modelled as a convex polygon).
        
        Arg's:
            T: Transformation matrix of the drone
        Returns:
            True if collision exists, false otherwise
        """
        
        Qpos = T[0:3,3]
        #check collision with wall
        ps_win = window.point_list
        n_win = window.normal_vec
        
        if abs((Qpos - ps_win[0]).dot(n_win)) > self.radius:
            return False
        else:
            if not(window.check_in_polygon(Qpos)):
                return True
            else:
                for i in range(len(ps_win)):
                    p1 = ps_win[i]
                    p2 = ps_win[i-1]
                    if self.check_collision_with_line(T, p1, p2):
                        return True
                return False    