"""
Author: Licheng Wen
Date: 2022-06-14 14:06:50
Description: 
Frenet optimal trajectory generator
中文翻译：
Frenet最优轨迹生成器(一种轨迹生成优化算法)

Ref:
- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

Copyright (c) 2022 by PJLab, All Rights Reserved. 
"""
import numpy as np
import matplotlib.pyplot as plt
import copy

import common.cost as cost
from utils.load_config import load_config
from utils.trajectory import Trajectory, State
from utils.cubic_spline import Spline2D
from trafficManager.planner.frenet_optimal_planner.polynomial_curve import QuarticPolynomial, QuinticPolynomial

# 计算特定路径
def calc_spec_path(current_state, target_state, T, dt):
    lat_qp = QuinticPolynomial(
        current_state.d,
        current_state.d_d,
        current_state.d_dd,
        target_state.d,
        target_state.d_d,
        target_state.d_dd,
        T,
    )
    lon_qp = QuinticPolynomial(
        current_state.s,
        current_state.s_d,
        current_state.s_dd,
        target_state.s,
        target_state.s_d,
        target_state.s_dd,
        T,
    )
    fp = Trajectory()
    for t in np.arange(0.0, T * 1.01, dt):
        fp.states.append(
            State(
                t=t,
                s=lon_qp.calc_point(t),
                d=lat_qp.calc_point(t),
                s_d=lon_qp.calc_first_derivative(t),
                d_d=lat_qp.calc_first_derivative(t),
                s_dd=lon_qp.calc_second_derivative(t),
                d_dd=lat_qp.calc_second_derivative(t),
                s_ddd=lon_qp.calc_third_derivative(t),
                d_ddd=lat_qp.calc_third_derivative(t),
            ))
    return fp

# 计算停止路径
def calc_stop_path(current_state, decel, T, dt, config):
    stop_path = Trajectory()
    t = 0
    s = current_state.s 
    d = current_state.d
    s_d = current_state.s_d
    d_d = current_state.d_d
    
    # 检查是否已经停止或速度分量是否为零，避免除零错误
    velocity_magnitude = np.sqrt(s_d**2 + d_d**2)
    if velocity_magnitude < 1e-6:  # 如果速度几乎为零
        # 车辆已经完全停止，直接生成停止状态序列
        final_s_d = 0.0
        final_d_d = 0.0
        while len(stop_path.states) < int(T / dt):
            stop_path.states.append(State(t=t, s=s, d=d, s_d=final_s_d, d_d=final_d_d, stop_flag=True))
            t += dt
        return stop_path
    
    while True:
        #Decomposition decel along s_d and d_d（三角形相似原理）
        # 添加除零保护
        velocity_magnitude = np.sqrt(s_d**2 + d_d**2)
        if velocity_magnitude < 1e-6:
            # 速度几乎为零，直接设置加速度为零
            s_dd = 0.0
            d_dd = 0.0
        else:
            s_dd = decel * s_d / velocity_magnitude
            d_dd = decel * d_d / velocity_magnitude
            
        stop_path.states.append(
            State(t=t, s=s, d=d, s_d=s_d, d_d=d_d, s_dd=s_dd))
        if s_d <= 1e-1: # 如果横向速度足够小
            # 强制设置最终速度为0，确保车辆完全停止
            final_s_d = 0.0
            final_d_d = 0.0
            while len(stop_path.states) < int(T / dt):
                t += dt
                stop_path.states.append(State(t=t, s=s, d=d, s_d=final_s_d, d_d=final_d_d, stop_flag=True))
            break
        # 更新状态数据
        t += dt
        s += s_d * dt
        d += d_d * dt
        s_d += s_dd * dt
        d_d += d_dd * dt
        if d_d < 0: # 如果纵向速度<0
            d_d = 1e-3
        if s_d < 0: # 如果横向速度<0
            sqrt_sd = np.sqrt(s_d**2 + d_d**2)
            if sqrt_sd < 1e-6:
                s_d = 1e-3
                d_d = 0.0
            else:
                s_d= 1e-3 * s_d / sqrt_sd
                d_d= 1e-3 * d_d / sqrt_sd
    return stop_path


def calc_frenet_paths(current_state, sample_d, sample_t, sample_v, dt, config):
    frenet_paths = []

    # generate path to each offset goal
    for di in sample_d:
        # Lateral motion planning
        for Ti in sample_t:
            fp = Trajectory()
            lat_qp = QuinticPolynomial(current_state.d, current_state.d_d,
                                       current_state.d_dd, di, 0.0, 0.0, Ti)
            for t in np.arange(0.0, Ti * 1.01, dt):
                fp.states.append(
                    State(
                        t=t,
                        d=lat_qp.calc_point(t),
                        d_d=lat_qp.calc_first_derivative(t),
                        d_dd=lat_qp.calc_second_derivative(t),
                        d_ddd=lat_qp.calc_third_derivative(t),
                    ))

            # Longitudinal motion planning (Velocity keeping)
            for tv in sample_v:
                lon_qp = QuarticPolynomial(current_state.s, current_state.s_d,
                                           current_state.s_dd, tv, 0.0, Ti)

                for state in fp.states:
                    state.s = lon_qp.calc_point(state.t)
                    state.s_d = lon_qp.calc_first_derivative(state.t)
                    state.s_dd = lon_qp.calc_second_derivative(state.t)
                    state.s_ddd = lon_qp.calc_third_derivative(state.t)

                frenet_paths.append(fp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    # 检查course_spline是否为空
    if csp is None:
        logging.warning("course_spline is None in calc_global_paths")
        return fplist
        
    for fp in fplist:
        fp.frenet_to_cartesian(csp)

    return fplist


def check_collision(fp, ob, config):
    for i in range(len(ob[:, 0])):
        d = []
        for fpi in range(len(fp.states)):
            d.append((fp.states[fpi].x - ob[i, 0])**2 +
                     (fp.states[fpi].y - ob[i, 1])**2)

        collision = any([di <= config["CAR_RADIUS"]**2 for di in d])

        if collision:
            return False

    return True


def cal_cost(fplist, ob, course_spline, config):
    # 检查course_spline是否为空
    if course_spline is None:
        logging.warning("course_spline is None in cal_cost")
        # 为所有轨迹设置一个默认的高成本，使它们不会被选择
        for path in fplist:
            path.cost = float('inf')
        return fplist
        
    for path in fplist:
        path.cost = 0
        ref_vel_list = [20.0 / 3.6] * len(path.states)
        # print("smooth cost", cost.smoothness(path, course_spline) * DT)
        # print("vel_diff cost", cost.vel_diff(path, ref_vel_list) * DT)
        # print("guidance cost", cost.guidance(path) * DT)
        # print("acc cost", cost.acc(path) * DT)
        # print("jerk cost", cost.jerk(path) * DT)
        path.cost = (
            cost.smoothness(path, course_spline, config["weights"]) * config["DT"] +
            cost.vel_diff(path, ref_vel_list, config["weights"]) * config["DT"] +
            cost.guidance(path, config["weights"]) * config["DT"] +
            cost.acc(path, config["weights"]) * config["DT"] +
            cost.jerk(path, config["weights"]) * config["DT"] +
            cost.time(path, config["weights"]) * config["DT"])
    return fplist


def check_path(path, ob, config):
    for state in path.states:
        if state.vel > config["MAX_SPEED"] / 3.6:  # Max speed check
            return False
        elif abs(state.acc) > config["MAX_ACCEL"]:  # Max accel check
            return False
        elif abs(state.cur) > config["MAX_CURVATURE"]:  # Max curvature check
            return False

    if not check_collision(path, ob, config):
        return False
    else:
        return True


def frenet_optimal_planning(csp, current_state, ob, config):
    # 检查course_spline是否为空
    if csp is None:
        logging.warning("course_spline is None in frenet_optimal_planning")
        return None
        
    target_speed = 30.0 / 3.6  # target speed [m/s]
    sample_d = np.arange(-config["MAX_ROAD_WIDTH"], config["MAX_ROAD_WIDTH"],
                         config["D_ROAD_W"])
    sample_t = np.arange(config["MIN_T"], config["MAX_T"], 0.5)
    sample_v = np.arange(
        target_speed - config["D_T_S"] / 3.6 * config["N_S_SAMPLE"],
        target_speed + config["D_T_S"] / 3.6 * config["N_S_SAMPLE"],
        config["D_T_S"] / 3.6,
    )
    fplist = calc_frenet_paths(current_state, sample_d,
                               sample_t, sample_v, config["DT"], config)
    fplist = calc_global_paths(fplist, csp)
    fplist = cal_cost(fplist, ob, csp, config)
    # 先排序 再从小到大check轨迹
    if fplist != None:
        fplist.sort(key=lambda x: x.cost)
        for path in fplist:
            if check_path(path, ob, config):
                return path

    print("No good path!!")
    return None


def main():
    config_file_path = "trafficManager/config.yaml"
    config = load_config(config_file_path)
    config["MAX_ROAD_WIDTH"] = 7.0

    # way points
    wx = [0.0, 10.0, 20.5, 35.0, 70.5]
    wy = [0.0, -4.0, 5.0, 6.5, 0.0]
    # obstacle lists
    # ob = np.array([[0, 0]])
    ob = np.array([[20.0, 10.0], [30.0, 6.0], [30.0, 8.0], [35.0, 8.0],
                   [50.0, 3.0]])

    def generate_target_course(x, y):
        csp = Spline2D(x, y)
        s = np.arange(0, csp.s[-1], 0.1)

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = csp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(csp.calc_yaw(i_s))
            rk.append(csp.calc_curvature(i_s))

        return rx, ry, ryaw, rk, csp

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)
    # generate left right boundaries
    s = np.arange(0, csp.s[-1], 0.1)
    left_bound, right_bound = [], []
    for si in s:
        xi, yi = csp.frenet_to_cartesian1D(si, -config["MAX_ROAD_WIDTH"] / 2)  # left
        left_bound.append([xi, yi])
        xi, yi = csp.frenet_to_cartesian1D(si, config["MAX_ROAD_WIDTH"] / 2)  # right
        right_bound.append([xi, yi])
    left_bound = np.array(left_bound)
    right_bound = np.array(right_bound)

    # initial state
    c_speed = 10.0 / 3.6  # current speed [m/s]
    c_d = 2.0  # current lateral position [m]
    s0 = 0.0  # current course position

    current_state = State(s=s0, s_d=c_speed, d=c_d)

    for i in range(config["SIM_LOOP"]):
        path = frenet_optimal_planning(
            csp,
            current_state,
            ob,
            config
        )

        current_state = path.states[1]

        if np.hypot(path.states[1].x - tx[-1],
                    path.states[1].y - ty[-1]) <= 1.0:
            print("Goal")
            break

        if config["ANIMATION"]:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )
            plt.plot(tx, ty)
            plt.plot(left_bound[:, 0], left_bound[:, 1], "g")
            plt.plot(right_bound[:, 0], right_bound[:, 1], "g")
            plt.plot(ob[:, 0], ob[:, 1], "xk")
            pathx = [state.x for state in path.states[1:]]
            pathy = [state.y for state in path.states[1:]]
            plt.plot(pathx, pathy, "-or")
            plt.plot(path.states[1].x, path.states[1].y, "vc")
            area = 8
            plt.xlim(path.states[1].x - area, path.states[1].x + area * 3)
            plt.ylim(path.states[1].y - area, path.states[1].y + area * 3)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            # plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

    print("Finish")
    if config["ANIMATION"]:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.01)
        plt.show()


if __name__ == "__main__":
    main()
