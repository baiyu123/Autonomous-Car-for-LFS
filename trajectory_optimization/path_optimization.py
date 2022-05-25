from dynamics import *
from scipy.optimize import minimize
from numba import jit, float64, int64
import time
start_time = time.time()

state_dict = {}

def traject_function(x):
    return 1/3*(x-2)**2

def init_reference_trajectory(ds, dt):
    reference_trajectory = []
    x = 0
    traject_x = []
    traject_y = []

    # reference trajectory
    while x < 7:
        reference_trajectory.append([x, traject_function(x)])
        traject_x.append(x)
        traject_y.append(traject_function(x))
        x += 0.1

    reference_points = []
    for elem in reference_trajectory:
        temp = point(elem[0], elem[1])
        reference_points.append(temp)
    return reference_points, traject_x, traject_y

# optimize trajectory
# @jit(float64(float64[:]), nopython=False, parallel=True)
def objective_traject(x):
    # ds = 0.5
    # dt = 0.1
    # reference_points, traject_x, traject_y = init_reference_trajectory(ds, dt)
    #
    # init_state = state(track_length=0, cross_track=0, relative_heading=0, v_x=1, v_y=0, yaw=0, steering=1.1,
    #                    throttle=0.4)
    # dyna = dynamics(init_state, reference_points, ds, dt)

    i = 0
    obj_sum = 0
    steer_weight = 0.01
    throttle_weight = 0.01
    while i < len(x):
        s = x[i]
        n = x[i+1]
        miu = x[i+2]
        v_x = x[i+3]
        v_y = x[i+4]
        steering_ds = x[i+8]
        throttle_ds = x[i+9]
        k = dyna.get_curvature_with_s(s)
        s_k_dt = (v_x*np.cos(miu) - v_y*np.sin(miu))/(1-n*k)
        time = ds/s_k_dt
        penalty = steer_weight*steering_ds**2 + throttle_weight*throttle_ds**2
        obj_sum += time + penalty
        i += obj_length
    return obj_sum

def dynamics_constrain(x, point, obj_var_index, obj_size, dyna):
    if point in state_dict:
        next_state = state_dict[point]
    else:
        curr_state = state(x[point - obj_size], x[point - obj_size + 1], x[point - obj_size + 2], x[point - obj_size + 3],
                       x[point - obj_size + 4], x[point - obj_size + 5], x[point - obj_size + 6],x[point - obj_size + 7] )
        next_state = dyna.dynamics_update_ds_with_state(curr_state, x[point - obj_size + 8], x[point-obj_size + 9]).to_list()
        state_dict[point] = next_state
    # reset after one iteration
    if point + obj_var_index > (len(x)-obj_size):
        state_dict.clear()
    return x[point + obj_var_index] - next_state[obj_var_index]

# def within_track_constrain(x, point, state_var_index, obj_size):


def constrain_generation(obj_size, num_of_point, dyna, track_width):
    # con_s = {'type': 'eq', 'fun': dynamics_constrain, 'args': (point, 0, obj_size, dyna)}
    track_with = track_width
    con_n_2 = {'type': 'ineq', 'fun': lambda x, point=0: x[point + 1] + track_with}
    con_n_3 = {'type': 'ineq', 'fun': lambda x, point=0: track_with - x[point + 1]}
    const_list = [con_n_2, con_n_3]
    # for i in range(1,num_of_point,1):
        # point = i*obj_size
        # # dynamics update constrains
        # con_s = {'type': 'eq', 'fun': lambda x, point=point: dynamics_constrain(x, point, 0, obj_size, dyna)}
        # con_n = {'type': 'eq', 'fun': lambda x, point=point: dynamics_constrain(x, point, 1, obj_size, dyna)}
        # con_miu = {'type': 'eq', 'fun': lambda x, point=point: dynamics_constrain(x, point, 2, obj_size, dyna)}
        # con_v_x = {'type': 'eq', 'fun': lambda x, point=point: dynamics_constrain(x, point, 3, obj_size, dyna)}
        # con_v_y = {'type': 'eq', 'fun': lambda x, point=point: dynamics_constrain(x, point, 4, obj_size, dyna)}
        # con_r = {'type': 'eq', 'fun': lambda x, point=point: dynamics_constrain(x, point, 5, obj_size, dyna)}
        # con_steering = {'type': 'eq', 'fun': lambda x, point=point: dynamics_constrain(x, point, 6, obj_size, dyna)}
        # con_throttle = {'type': 'eq', 'fun': lambda x, point=point: dynamics_constrain(x, point, 7, obj_size, dyna)}
        # # constrain in track
        # con_n_2 = {'type': 'ineq', 'fun': lambda x, point=point: x[point + 1] + track_with}
        # con_n_3 = {'type': 'ineq', 'fun': lambda x, point=point: track_with - x[point + 1]}

        # const_list += [con_s, con_n, con_miu, con_v_x, con_v_y, con_r, con_steering, con_throttle, con_n_2, con_n_3]
    return const_list


# reference_trajectory = [[0,0],[0,1],[0,2],[0,3],[0,4]]
ds = 0.5
dt = 0.1
track_width = 0.4
reference_points, traject_x, traject_y  = init_reference_trajectory(ds, dt)

init_state = state(track_length=0, cross_track=0, relative_heading=0, v_x=1, v_y=0, yaw=0, steering=1.1, throttle=0.4)
dyna = dynamics(init_state, reference_points, ds, dt)
x = []
y = []

# objective initialization
init_obj_array = []
s = 0
obj_length = 0
num_of_points = 0
while s < 2:
    s += ds
    n = 0
    miu = 0
    v_x = 1
    v_y = 0
    r = 0
    steering = 0
    throttle = 0
    steering_ds = 0
    throttle_ds = 0
    obj_vars = [s, n, miu, v_x, v_y, r, steering, throttle, steering_ds, throttle_ds]
    obj_length = len(obj_vars)
    init_obj_array += obj_vars
    num_of_points += 1

consts = constrain_generation(obj_length, num_of_points, dyna, track_width)
options = {"disp": True, "maxiter": 500000}
solution = minimize(objective_traject, init_obj_array, method='SLSQP', constraints=consts, options=options)
print(solution)
s = 0
solution = solution.x
# solution = [ 4.99999992e-01, -4.99328718e-01, -8.40805544e-01,  4.07789800e+02,
#         7.10536157e+02,  1.95910468e+03,  1.76920740e+03,  2.80261246e+04,
#        -2.73224590e-06, -9.86873192e-05,  9.99999992e-01, -3.83072633e-01,
#         4.02770358e-01,  1.45582273e+03,  1.64047670e+02,  1.95910437e+03,
#         1.76920740e+03,  2.80261246e+04, -4.44146643e-04, -6.61816752e-04,
#         1.49999999e+00, -7.18822748e-02,  1.11745526e+00,  1.65463572e+03,
#        -1.06599916e+03,  1.95910417e+03,  1.76920740e+03,  2.80261246e+04,
#        -1.69244623e-05, -2.21697529e-04,  1.99999999e+00,  2.39699471e-01,
#         1.52358800e+00,  1.05964605e+03, -2.05561632e+03,  1.95910403e+03,
#         1.76920740e+03,  2.80261246e+04,  8.90828769e-05,  5.89457774e-05,
#         2.49999999e+00,  4.37096615e-01,  1.64143407e+00,  2.61541484e+02,
#        -2.48183101e+03,  1.95910394e+03,  1.76920740e+03,  2.80261246e+04,
#         5.11839115e-04, -2.12246819e-04,  2.99999999e+00,  5.00000000e-01,
#         1.59108282e+00, -4.19641794e+02, -2.55574590e+03,  1.95910388e+03,
#         1.76920740e+03,  2.80261246e+04, -1.01040050e-04,  2.68508359e-05,
#         3.49999999e+00,  4.48679723e-01,  1.58013074e+00, -1.09889607e+03,
#        -2.44100529e+03,  1.95910381e+03,  1.76920740e+03,  2.80261246e+04,
#        -5.28536687e-05, -1.99487184e-04,  3.99999999e+00,  2.70560119e-01,
#         1.69433012e+00, -1.86733998e+03, -2.08464616e+03,  1.95910374e+03,
#         1.76920740e+03,  2.80261246e+04,  2.72380733e-05, -1.27444862e-04,
#         4.49999999e+00, -4.95995433e-02,  1.94343333e+00, -2.65854874e+03,
#        -1.35089437e+03,  1.95910365e+03,  1.76920740e+03,  2.80261246e+04,
#         6.26470446e-06, -3.25513007e-05,  4.99999999e+00, -5.00000000e-01,
#         2.28147417e+00, -3.22786867e+03, -1.68669729e+02,  1.95910355e+03,
#         1.76920740e+03,  2.80261246e+04, -7.63320444e-08, -7.19773556e-08]
s = 0
left_x = []
left_y = []
right_x = []
right_y = []
for i in range(num_of_points):
    s += ds
    point = i*obj_length
    n = solution[point+1]
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