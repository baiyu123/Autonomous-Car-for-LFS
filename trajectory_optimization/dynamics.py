import numpy as np
import copy
import matplotlib.pyplot as plt


class parameter_class:
    def __init__(self):
        # vehicle mass
        self.m = 1000
        self.g = 9.8
        # vehicle axis
        self.l_f = 1.5
        self.l_r = 1.5
        # tire load
        self.F_N_front = (self.l_f/(self.l_f + self.l_r))*self.m*self.g
        self.F_N_rear = (self.l_r/(self.l_f + self.l_r))*self.m*self.g
        self.engine_max_force = 5000
        self.drag_coefficient = 0.25

        # tire lateral force
        # pacejka tire model coeficcient link: https://www.mathworks.com/help/physmod/sdl/ref/tireroadinteractionmagicformula.html
        self.B = 10
        self.C = 1.9
        self.D = 1
        self.F_y_front = 0
        self.F_y_rear = 0
        self.I = self.m * self.l_r**2

    def calculate_front_rear_tire_lateral(self, v_y, v_x, r, steering):
        # possible bug
        # slip_angle_front = -steering + np.arctan((v_y + self.l_f * r)/ (v_x+0.001))
        slip_angle_rear = np.arctan((v_y + self.l_f * r) / (v_x+0.001))
        slip_angle_front = -steering
        slip_angle_rear = 0
        self.F_y_front = -self.F_N_front * self.D * np.sin(self.C * np.arctan(self.B * slip_angle_front))
        self.F_y_rear = -self.F_N_rear * self.D * np.sin(self.C * np.arctan(self.B * slip_angle_rear))
        return

    def calculate_drive_force(self, throttle, v_x):
        self.F_x = self.engine_max_force*throttle - self.drag_coefficient*v_x**2

    def update(self, last_state):
        self.calculate_front_rear_tire_lateral(last_state.v_y, last_state.v_x, last_state.r, last_state.delta)
        self.calculate_drive_force(last_state.T, last_state.v_x)


# for reference trajectory
class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class state:
    def __init__(self, track_length, cross_track, relative_heading, v_x, v_y, yaw, steering, throttle):
        self.s = track_length
        self.n = cross_track
        self.miu = relative_heading
        self.v_x = v_x
        self.v_y = v_y
        self.r = yaw
        self.delta = steering
        self.T = throttle

    def to_list(self):
        return [self.s, self.n, self.miu, self.v_x, self.v_y, self.r, self.delta, self.T]

    def __str__(self):
        return 's:' + str(self.s) + ' n:' + str(self.n) + " v_x:" + str(self.v_x) + " v_y:" + str(self.v_y) + " steer:" + str(self.delta) + ' yaw rate:' + str(self.r) + " throttle:" + str(self.T) + " heading:" + str(self.miu)

class dynamics:

    # initial state
    def __init__(self, init_state, reference_points, ds, dt):
        self.current_state = init_state
        self.reference_points = reference_points
        self.dt = dt
        self.ds = ds

    # def curve_to_world(self, s, reference_points):
    #     temp_s = copy.deepcopy(s)
    #     pos = None
    #     for i in range(len(reference_points)-1):
    #         curr_point = np.array([reference_points[i].x, reference_points[i].y])
    #         next_point = np.array([reference_points[i+1].x, reference_points[i+1].y])
    #         vec = next_point - curr_point
    #         length = np.linalg.norm(vec)
    #         if length < temp_s:
    #             temp_s -= length
    #         else:
    #             vec = vec/np.linalg.norm(vec)
    #             pos = vec*temp_s
    #
    #     return pos

    # given s track process return closest point index that passed
    def curve_to_world_index(self, s):
        temp_s = copy.deepcopy(s)
        s_pos = None
        index = 0
        for i in range(len(self.reference_points) - 1):
            curr_point = np.array([self.reference_points[i].x, self.reference_points[i].y])
            next_point = np.array([self.reference_points[i + 1].x, self.reference_points[i + 1].y])
            vec = next_point - curr_point
            length = np.linalg.norm(vec)
            if length < temp_s:
                temp_s -= length
            else:
                vec = vec / np.linalg.norm(vec)
                s_pos = vec * temp_s + curr_point
                index = i
                break
        return index

    def calculate_dist(self, x1, y1, x2, y2):
        dist_vec = np.array([x1-x2, y1-y2])
        dist = np.linalg.norm(dist_vec)
        return dist

    def find_closes_point_index(self, reference_points, current_x, current_y):
        min_dist = 100000
        min_index = 0
        for i in range(len(reference_points)):
            ref_point = reference_points[i]
            dist = self.calculate_dist(ref_point.x, ref_point.y, current_x, current_y)
            if dist < min_dist:
                min_index = i
        return min_index

    def cal_curvature(self, index, reference_points):
        if index == 0:
            index = 1
        if index == len(reference_points) - 1:
            index = len(reference_points) - 2
        p1 = np.array([reference_points[index - 1].x, reference_points[index - 1].y])
        p2 = np.array([reference_points[index].x, reference_points[index].y])
        p3 = np.array([reference_points[index + 1].x, reference_points[index + 1].y])
        x = [p1, p2, p3]
        v1 = x[1] - x[0]
        v2 = x[2] - x[0]
        area = np.linalg.norm(np.cross(v1, v2))/2
        x_y = np.linalg.norm(x[0]-x[1])
        y_z = np.linalg.norm(x[1]-x[2])
        z_x = np.linalg.norm(x[2]-x[0])
        curvature = 4*area/(x_y*y_z*z_x)
        return curvature

    def get_curvature_with_s(self, s):
        closest_index = self.curve_to_world_index(s)
        k = self.cal_curvature(closest_index, self.reference_points)
        return k
    # state
    def dx_dt(self, current_state, steering_dt, throttle_dt):
        k = self.get_curvature_with_s(current_state.s)
        # print('k:' + str(k))
        parameters = parameter_class()
        parameters.update(last_state=current_state)

        s_dt = (current_state.v_x*np.cos(current_state.miu) - current_state.v_y * np.sin(current_state.miu))/(1-current_state.n*k)
        n_dt = current_state.v_x*np.sin(current_state.miu) + current_state.v_y*np.cos(current_state.miu)
        miu_dt = current_state.r - k*s_dt
        v_x_dt = (1/parameters.m)*(parameters.F_x - parameters.F_y_front*np.sin(current_state.delta) + parameters.m*current_state.v_y*current_state.r)
        v_y_dt = (1/parameters.m)*(parameters.F_y_rear + parameters.F_y_front*np.cos(current_state.delta) - parameters.m*current_state.v_x*current_state.r)
        r_dt = (1/parameters.I)*(parameters.F_y_front*parameters.l_f*np.cos(current_state.delta) - parameters.F_y_rear*parameters.l_r)
        delta_dt = steering_dt
        throttle_dt = throttle_dt
        new_state = np.array([s_dt, n_dt, miu_dt, v_x_dt, v_y_dt, r_dt, delta_dt, throttle_dt])
        return new_state



    def dynamics_update_dt(self, steering_dt, throttle_dt):
        state_dt = self.dx_dt(self.current_state, steering_dt, throttle_dt)
        s = self.current_state.s + state_dt[0]*self.dt
        n = self.current_state.n + state_dt[1]*self.dt
        miu = self.current_state.miu + state_dt[2]*self.dt
        v_x = self.current_state.v_x + state_dt[3]*self.dt
        v_y = self.current_state.v_y + state_dt[4]*self.dt
        r = self.current_state.r + state_dt[5]*self.dt
        delta = self.current_state.delta + state_dt[6]*self.dt
        T = self.current_state.T + state_dt[7]*self.dt
        self.current_state = state(track_length=s, cross_track=n, relative_heading=miu, v_x=v_x, v_y=v_y, yaw=r, steering=delta, throttle=T)

    def dx_ds(self, current_state, steering_ds, throttle_ds):
        state_dt = self.dx_dt(current_state, steering_ds, throttle_ds)
        state_st = [1]
        for i in range(1,len(state_dt),1):
            state_st.append(state_dt[i]/state_dt[0])
        return state_st

    def dynamics_update_ds(self, steering_ds, throttle_ds):
        state_ds = self.dx_ds(self.current_state, steering_ds, throttle_ds)
        s = self.current_state.s + self.ds
        n = self.current_state.n + state_ds[1] * self.ds
        miu = self.current_state.miu + state_ds[2] * self.ds
        v_x = self.current_state.v_x + state_ds[3] * self.ds
        v_y = self.current_state.v_y + state_ds[4] * self.ds
        r = self.current_state.r + state_ds[5] * self.ds
        delta = self.current_state.delta + state_ds[6] * self.ds
        T = self.current_state.T + state_ds[7] * self.ds
        self.current_state = state(track_length=s, cross_track=n, relative_heading=miu, v_x=v_x, v_y=v_y, yaw=r, steering=delta, throttle=T)

    def dynamics_update_ds_with_state(self, current_state, steering_ds, throttle_ds):
        state_ds = self.dx_ds(current_state, steering_ds, throttle_ds)
        s = current_state.s + self.ds
        n = current_state.n + state_ds[1] * self.ds
        miu = current_state.miu + state_ds[2] * self.ds
        v_x = current_state.v_x + state_ds[3] * self.ds
        v_y = current_state.v_y + state_ds[4] * self.ds
        r = current_state.r + state_ds[5] * self.ds
        delta = current_state.delta + state_ds[6] * self.ds
        T = current_state.T + state_ds[7] * self.ds
        next_state = state(track_length=s, cross_track=n, relative_heading=miu, v_x=v_x, v_y=v_y, yaw=r, steering=delta, throttle=T)
        return next_state

    def get_world_coord(self):
        return self.get_world_coord_with_s_n(self.current_state.s, self.current_state.n)

    def get_world_coord_with_s_n(self, s, n):
        temp_s = copy.deepcopy(s)
        s_pos = None
        index = 0
        for i in range(len(self.reference_points) - 1):
            curr_point = np.array([self.reference_points[i].x, self.reference_points[i].y])
            next_point = np.array([self.reference_points[i + 1].x, self.reference_points[i + 1].y])
            vec = next_point - curr_point
            length = np.linalg.norm(vec)
            if length < temp_s:
                temp_s -= length
            else:
                vec = vec / np.linalg.norm(vec)
                s_pos = vec * temp_s + curr_point
                index = i
                break
        # exceed reference
        if s_pos is None:
            return None
        tangent_vec = np.array([self.reference_points[index + 1].x - self.reference_points[index].x, self.reference_points[index + 1].y - self.reference_points[index].y])
        tangent_vec = tangent_vec/np.linalg.norm(tangent_vec)
        theta = np.pi/2

        rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                            [np.sin(theta), np.cos(theta)]])
        pos_vec = n * np.dot(rot_mat, tangent_vec) + s_pos
        return pos_vec

