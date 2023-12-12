from arm_dynamics_base import ArmDynamicsBase
import numpy as np
from geometry import rot, xaxis, yaxis


class ArmDynamicsStudent(ArmDynamicsBase):

    def index_f(self, i):
        return 2 * i

    def index_a(self, i):
        return 2 * self.num_links + 2 * i

    def index_omdot(self, i):
        return 2 * self.num_links + 2 * self.num_links + i

    def index_f_eq(self, i):
        return self.index_f(i)

    def index_t_eq(self, i):
        return 2 * self.num_links + i

    def num_var(self):
        return 2 * self.num_links + 2 * self.num_links + self.num_links

    def constraint_matrices(self, state, action):
        num_vars = self.num_var()
        q = self.get_q(state)
        theta = self.compute_theta(q)
        qd = self.get_qd(state)
        omega = self.compute_omega(qd)
        vel_0 = self.get_vel_0(state)
        vel = self.compute_vel(vel_0, omega, theta)
        vel_com = self.compute_vel_com(vel, omega)

        left_hand = None
        right_hand = None

        for i in range(0, self.num_links):
            con_l = np.zeros((2, num_vars))
            con_l[0:2, self.index_f(i):self.index_f(i + 1)] = -1 * np.eye(2)
            con_l[0:2, self.index_a(i):self.index_a(i + 1)] = -1 * self.link_masses[i] * np.eye(2)
            con_l[1, self.index_omdot(i)] = -1 * 0.5 * self.link_lengths[i] * self.link_masses[i]
            if i < self.num_links - 1:
                con_l[0:2, self.index_f(i + 1):self.index_f(i + 2)] = rot(q[i + 1])
            con_r = np.zeros((2, 1))

            if self.gravity:
                con_r = con_r + (-1 * 9.8 * self.link_masses[i]) * (np.dot(rot(-1 * theta[i]), (-1 * yaxis())))

            con_r[0] = con_r[0] + (-1) * (omega[i] * omega[i] * 0.5 * self.link_lengths[i] * self.link_masses[i])
            if i == 0:
                left_hand = con_l
                right_hand = con_r
            else:
                left_hand = np.concatenate((left_hand, con_l))
                right_hand = np.concatenate((right_hand, con_r))

        for i in range(0, self.num_links):
            con_l = np.zeros((1, num_vars))
            con_l[0, self.index_f(i) + 1] = self.link_lengths[i] * 0.5
            con_l[0, self.index_omdot(i)] = -1 * self.link_inertias[i]
            if i < self.num_links - 1:
                con_l[0, self.index_f(i + 1):self.index_f(i + 2)] = self.link_lengths[i] * 0.5 * rot(q[i + 1])[1, :]
            left_hand = np.concatenate((left_hand, con_l))
            con_r = np.zeros((1, 1))
            right_hand = np.concatenate((right_hand, con_r))

        for i in range(1, self.num_links):
            con_l = np.zeros((2, num_vars))
            con_l[0:2, self.index_a(i):self.index_a(i + 1)] = -1 * np.eye(2)
            con_l[0:2, self.index_a(i - 1):self.index_a(i)] = rot(-1 * q[i])
            con_l[0:2, self.index_omdot(i - 1):self.index_omdot(i)] = self.link_lengths[i - 1] * (
                np.dot(rot(-1 * q[i]), (1 * yaxis())))
            left_hand = np.concatenate((left_hand, con_l))
            con_r = -1 * self.link_lengths[i - 1] * omega[i - 1] * omega[i - 1] * (np.dot(rot(-1 * q[i]), (-1 * xaxis())))
            right_hand = np.concatenate((right_hand, con_r))

        assert left_hand.shape == (self.num_var() - 2, self.num_var())
        assert right_hand.shape == (self.num_var() - 2, 1)

        for i in range(self.num_links):
            right_hand[self.index_t_eq(i)] += qd[i] * self.joint_viscous_friction

        con_l = np.zeros((2, self.num_var()))
        con_l[0:2, self.index_a(0):self.index_a(1)] = np.eye(2)
        left_hand = np.concatenate((left_hand, con_l))
        con_r = np.zeros((2, 1))
        right_hand = np.concatenate((right_hand, con_r))

        assert left_hand.shape == (5 * self.num_links, 5 * self.num_links)
        assert right_hand.shape == (5 * self.num_links, 1)

        tor = action
        for i in range(self.num_links):
            right_hand[self.index_t_eq(i), 0] += (tor[i + 1] if i < self.num_links - 1 else 0.0) - tor[i]

        return left_hand, right_hand

    def solve(self, left_hand, right_hand):
        x = np.linalg.solve(left_hand, right_hand)
        self.residue = np.linalg.norm(np.dot(left_hand, x) - right_hand) / self.num_var()
        err = np.linalg.norm(np.dot(left_hand, x) - right_hand) / self.num_var()
        if err > self.residue_limit:
            print(f'Cannot solve: Solution error {err:.4f} exceeds limit {self.residue_limit:.4f}')
            self.residue_limit_flag = True
        a = x[self.index_a(0):self.index_a(self.num_links)]
        omdot = x[self.index_omdot(0):self.index_omdot(self.num_links)]
        qdd = omdot.copy()
        for i in range(self.num_links - 1, 0, -1):
            qdd[i] -= qdd[i - 1]
        return a, qdd

    def dynamics_step(self, state, action, dt):

        # state has the following format: [q_0, ..., q_(n-1), qdot_0, ..., qdot_(n-1)] where n is the number of links
        # action has the following format: [mu_0, ..., mu_(n-1)]
        # You can make use of the additional variables:
        # self.num_links: the number of links
        # self.joint_viscous_friction: the coefficient of viscous friction
        # self.link_lengths: an array containing the lengths of all the links
        # self.link_masses: an array containing the masses of all the links

        # Replace this with your code:

        left_hand, right_hand = self.constraint_matrices(state, action)
        a, qdd = self.solve(left_hand, right_hand)
        new_state = self.euler_integration(state, a, qdd, dt)
        return new_state

    def euler_integration(self, state, a, qdd, dt):
        pos_0 = self.get_pos_0(state)
        vel_0 = self.get_vel_0(state)
        q = self.get_q(state)
        qd = self.get_qd(state)
        theta = self.compute_theta(q)

        qd_new = qd + qdd * dt
        q_new = q + 0.5 * (qd + qd_new) * dt

        theta_new = self.compute_theta(q_new)
        vel_0_new = np.dot(rot(theta[0] - theta_new[0]), (vel_0 + a[0:2] * dt))
        pos_0_new = pos_0 + 0.5 * (np.dot(rot(theta[0]), vel_0) + np.dot(rot(theta_new[0]), vel_0_new)) * dt

        new_state = np.vstack([q_new, qd_new])

        return new_state

    

