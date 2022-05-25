from dynamics import *
from scipy.optimize import minimize
from numba import jit, float64, int64
import time
start_time = time.time()
calculate_length = 15
reference_x = 6
state_dict = {}
plt.axis([-1, 10, -1, 10])


def traject_function(x):
    # return 1/3*(x-3)**2
    # return 1*np.sin(0.5*x)
    return 1/(x+0.17)

# parametric equation of a*t^2 + b*t + c, point at t= 0, 0.25, 0.5, 0.75, 1
def parametric_trajectory_coefficient_3points(p1, p2, p3, p4, p5):
    t_mat = np.array([[0,0,1,0,0,0],
                      [0,0,0,0,0,1],
                      [0.125,0.25,1,0,0,0],
                      [0,0,0,0.125,0.25,1],
                      [0.25,0.5,1,0,0,0],
                      [0,0,0,0.25,0.5,1],
                      [0.5625,0.75,1,0,0,0],
                      [0,0,0,0.5625,0.75,1],
                      [1,1,1,0,0,0],
                      [0,0,0,1,1,1]])
    b_mat = np.array([p1[0],p1[1],p2[0],p2[1],p3[0],p3[1],p4[0],p4[1],p5[0],p5[1]])
    coefficient = np.linalg.pinv(t_mat)@b_mat
    return coefficient

def generate_trajectory_polynomial(step_size, p1, p2, p3, p4, p5):
    coeff = parametric_trajectory_coefficient_3points(p1,p2,p3,p4,p5)
    coeff_mat = np.array([[coeff[0],coeff[1],coeff[2]],
                          [coeff[3],coeff[4],coeff[5]]])
    traject = []
    count = int(0.5/step_size)
    for i in range(count):
        t = 0.25 + float(i/count)
        pos = coeff_mat@np.array([t*t, t, 1]).transpose()
        traject.append([pos[0], pos[1]])
    return traject

def generate_catmull_rom(step_size,p1,p2,p3,p4):
    s = 0.5
    basis = np.array([[-s, 2-s, s-2, s],
                      [2*s, s-3, 3-2*s, -s],
                      [-s, 0, s, 0],
                      [0, 1, 0, 0]])
    control_mat = np.array([[p1[0],p1[1],0],
                            [p2[0],p2[1],0],
                            [p3[0],p3[1],0],
                            [p4[0],p4[1],0],])
    traject = []
    count = int(1/step_size)
    for i in range(count):
        t = float(i/count)
        param_mat = np.array([t**3, t**2, t, 1])
        pos = param_mat@basis@control_mat
        traject.append([pos[0], pos[1]])
    return traject

def generate_trajectory_multiple_point(step_size, point_list):
    traject = []
    for i in range(0, len(point_list)-3, 1):
        temp_traject = generate_catmull_rom(step_size, point_list[i], point_list[i+1], point_list[i+2], point_list[i+3])
        # plt.plot([i[0] for i in temp_traject], [i[1] for i in temp_traject], marker='o', color='r')
        # plt.show()
        traject.extend(temp_traject)
    # plt.plot([i[0] for i in traject], [i[1] for i in traject], marker='o', color='r')
    # plt.show()
    return traject



def init_reference_trajectory():
    reference_trajectory = []
    x = 0
    traject_x = []
    traject_y = []

    # reference trajectory
    # while x < reference_x:
    #     reference_trajectory.append([x, traject_function(x)])
    #     traject_x.append(x)
    #     traject_y.append(traject_function(x))
    #     x += 0.1

    points = [[10,0],[3,0],[2,0],[0.5,0.5],[0,2],[0,3],[0,7],[0,8],[0.5,9.5],[2,10],[3,10],[8,10],[9.5,9.5],[10, 8]]
    reference_trajectory = generate_trajectory_multiple_point(0.1,points)

    reference_points = []
    for elem in reference_trajectory:
        temp = point(elem[0], elem[1])
        reference_points.append(temp)


    for elem in reference_trajectory:
        traject_x.append(elem[0])
        traject_y.append(elem[1])

    return reference_points, traject_x, traject_y

def cal_curvature(p1, p2, p3):
    x = [p1, p2, p3]
    v1 = x[1] - x[0]
    v2 = x[2] - x[0]
    area = np.linalg.norm(np.cross(v1, v2))/2
    x_y = np.linalg.norm(x[0]-x[1])
    y_z = np.linalg.norm(x[1]-x[2])
    z_x = np.linalg.norm(x[2]-x[0])
    curvature = 4*area/(x_y*y_z*z_x)
    return curvature

# optimize trajectory
def objective_traject(x):
    obj = 0
    for i in range(1,len(x)-1,1):
        s = i*ds
        prev_coord = dyna.get_world_coord_with_s_n(s-ds, x[i-1])
        coord = dyna.get_world_coord_with_s_n(s, x[i])
        coord_next =  dyna.get_world_coord_with_s_n(s+ds, x[i+1])
        dist = np.linalg.norm(coord - prev_coord)
        k = cal_curvature(prev_coord, coord, coord_next)
        k_ds = 0
        if i > 1:
            prev_prev_coord = dyna.get_world_coord_with_s_n(s-2*ds, x[i-2])
            k_prev = cal_curvature(prev_prev_coord, prev_coord, coord)
            k_ds = k-k_prev
        obj += k + 1*k_ds**2 + dist
    return obj


    return total_curv


# def within_track_constrain(x, point, state_var_index, obj_size):


def constrain_generation(num_of_point):
    const_list = []
    track_width = 0.5
    for i in range(0,num_of_point,1):
        point = i
        # constrain in track
        con_n_2 = {'type': 'ineq', 'fun': lambda x, point=point: x[point] + track_width}
        con_n_3 = {'type': 'ineq', 'fun': lambda x, point=point: track_width - x[point]}
        const_list += [con_n_2, con_n_3]
    return const_list


# reference_trajectory = [[0,0],[0,1],[0,2],[0,3],[0,4]]
ds = 0.5
dt = 0.1
track_width = 0.5
reference_points, traject_x, traject_y  = init_reference_trajectory()

init_state = state(track_length=0, cross_track=0, relative_heading=0, v_x=1, v_y=0, yaw=0, steering=1.1, throttle=0.4)
dyna = dynamics(init_state, reference_points, ds, dt)
x = []
y = []

# objective initialization
init_obj_array = []
s = 0
num_of_points = 0
while s < calculate_length:
    s += ds
    n = 0
    init_obj_array.append(n)
    num_of_points += 1

consts = constrain_generation(num_of_points)
options = {"disp": True, "maxiter": 1000}
solution = minimize(objective_traject, init_obj_array, method='SLSQP', constraints=consts, options=options)
print(solution)
s = 0
solution = solution.x
s = 0
left_x = []
left_y = []
right_x = []
right_y = []
for i in range(num_of_points):
    s += ds
    n = solution[i]
    world = dyna.get_world_coord_with_s_n(s, n)
    left_bound = dyna.get_world_coord_with_s_n(s, track_width)
    right_bound = dyna.get_world_coord_with_s_n(s, -track_width)
    left_x.append(left_bound[0])
    left_y.append(left_bound[1])
    right_x.append(right_bound[0])
    right_y.append(right_bound[1])
    x.append(world[0])
    y.append(world[1])

plt.plot(x, y, marker = 'o')
plt.plot(left_x, left_y, marker = 'o', color='g')
plt.plot(right_x, right_y, marker = 'o', color='g')
plt.plot(traject_x, traject_y, marker='o', color='r')
plt.axis([-1, 11, -1, 11])
plt.show()



# testing dynamics
# while True:
#     dyna.dynamics_update_ds(0.00, 0.0)
#     print(dyna.current_state)
#     world_coord = dyna.get_world_coord()
#     if world_coord is None:
#         break
#     print(world_coord)
#     x.append(world_coord[0])
#     y.append(world_coord[1])
#
# plt.plot(x, y, marker = 'o')
# plt.plot(traject_x, traject_y, marker='o', color='r')
# plt.show()

# optimization
# def objective(x):
#     return x[0] + x[1]
#
# def eq_constrain(x):
#     return 2*x[0]**2 - x[1]
#
# def ineq_constrain(x):
#     return x[0] - 1
#
# def cons_func(x, num):
#     return x[0] - num
#
# init = [-14,-100]
# # con1 = {'type':'eq', 'fun': eq_constrain}
# # con2 = {'type':'ineq', 'fun': ineq_constrain}
# con1 = {'type':'eq', 'fun': lambda x: 2*x[0]**2 - x[1]}
# con2 = {'type':'ineq', 'fun': lambda x: cons_func(x, 1)}
# consts = [con1, con2]
# solution1 = minimize(objective, init, method='SLSQP', constraints=consts)
# solution2 = shgo(objective, [(0,10)]*2, constraints=consts, iters=5)
# print(solution1)
# print("---------------")
# print(solution2)

print("--- %s seconds ---" % (time.time() - start_time))