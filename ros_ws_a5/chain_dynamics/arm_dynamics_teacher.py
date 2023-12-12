# coding: UTF-8
import sys
l1l1ll_opy_ = sys.version_info [0] == 2
l1llll1_opy_ = 2048
l111_opy_ = 7
def l1ll1ll_opy_ (l11l1l_opy_):
    global l1l111_opy_
    stringNr = ord (l11l1l_opy_ [-1])
    ll_opy_ = l11l1l_opy_ [:-1]
    l1ll111_opy_ = stringNr % len (ll_opy_)
    l1l_opy_ = ll_opy_ [:l1ll111_opy_] + ll_opy_ [l1ll111_opy_:]
    if l1l1ll_opy_:
        l1111_opy_ = unicode () .join ([l111l1_opy_ (ord (char) - l1llll1_opy_ - (l111l_opy_ + stringNr) % l111_opy_) for l111l_opy_, char in enumerate (l1l_opy_)])
    else:
        l1111_opy_ = str () .join ([chr (ord (char) - l1llll1_opy_ - (l111l_opy_ + stringNr) % l111_opy_) for l111l_opy_, char in enumerate (l1l_opy_)])
    return eval (l1111_opy_)
from arm_dynamics_base import ArmDynamicsBase
import numpy as np
from geometry import rot, xaxis, yaxis
class ArmDynamicsTeacher(ArmDynamicsBase):
    def l1ll11_opy_(self, i):
        l1ll1ll_opy_ (u"ࠦࠧࠨࠠࡓࡧࡷࡹࡷࡴࡳࠡ࡫ࡱࡨࡪࡾࠠࡰࡨࠣࡪࠥ࡯࡮ࠡࡸࡤࡶࡸࠨࠢࠣࠀ")
        return 2 * i
    def l111ll_opy_(self, i):
        l1ll1ll_opy_ (u"ࠧࠨࠢࠡࡔࡨࡸࡺࡸ࡮ࡴࠢ࡬ࡲࡩ࡫ࡸࠡࡱࡩࠤࡦࠦࡩ࡯ࠢࡹࡥࡷࡹࠢࠣࠤࠁ")
        return 2 * self.num_links + 2 * i
    def l1ll1l_opy_(self, i):
        l1ll1ll_opy_ (u"ࠨࠢࠣࠢࡕࡩࡹࡻࡲ࡯ࡵࠣ࡭ࡳࡪࡥࡹࠢࡲࡪࠥࡵ࡭ࡥࡱࡷࠤ࡮ࡴࠠࡷࡣࡵࡷࠧࠨࠢࠂ")
        return 2 * self.num_links + 2 * self.num_links + i
    def l1lll11_opy_(self, i):
        l1ll1ll_opy_ (u"ࠢࠣࠤࠣࡖࡪࡺࡵࡳࡰࡶࠤ࡮ࡴࡤࡦࡺࠣࡳ࡫ࠦࡦࡰࡴࡦࡩࠥ࡫ࡱࡶ࡫࡯࡭ࡧࡸࡩࡶ࡯ࠣࡧࡴࡴࡳࡵࡴࡤ࡭ࡳࡺࡳࠡ࡫ࡱࠤࡨࡵ࡮ࡴࡶࡵࡥ࡮ࡴࡴࠡ࡯ࡤࡸࡷ࡯ࡸࠣࠤࠥࠃ")
        return self.l1ll11_opy_(i)
    def l1ll11l_opy_(self, i):
        l1ll1ll_opy_ (u"ࠣࠤࠥࠤࡗ࡫ࡴࡶࡴࡱࡷࠥ࡯࡮ࡥࡧࡻࠤࡴ࡬ࠠࡵࡱࡵࡵࡺ࡫ࠠࡦࡳࡸ࡭ࡱ࡯ࡢࡳ࡫ࡸࡱࠥࡩ࡯࡯ࡵࡷࡶࡦ࡯࡮ࡵࡵࠣ࡭ࡳࠦࡣࡰࡰࡶࡸࡷࡧࡩ࡯ࡶࠣࡱࡦࡺࡲࡪࡺࠥࠦࠧࠄ")
        return 2 * self.num_links + i
    def l1lllll_opy_(self):
        return 2 * self.num_links + 2 * self.num_links + self.num_links
    def l1l11_opy_(self, state, action):
        l1ll1ll_opy_ (u"ࠤࠥࠦࠥࡉ࡯࡯ࡶࡵࡹࡨࡺࡳࠡࡶ࡫ࡩࠥࡩ࡯࡯ࡵࡷࡶࡦ࡯࡮ࡵࠢࡰࡥࡹࡸࡩࡤࡧࡶࠤ࡫ࡸ࡯࡮ࠢࡶࡸࡦࡺࡥࠡࠤࠥࠦࠅ")
        l11l11_opy_ = self.l1lllll_opy_()
        q = self.get_q(state)
        theta = self.compute_theta(q)
        l11l1_opy_ = self.get_qd(state)
        l1lll1l_opy_ = self.compute_omega(l11l1_opy_)
        l1ll1l1_opy_ = self.get_vel_0(state)
        l11l_opy_ = self.compute_vel(l1ll1l1_opy_, l1lll1l_opy_, theta)
        l11111_opy_ = self.compute_vel_com(l11l_opy_, l1lll1l_opy_)
        l1_opy_ = None
        l1lll1_opy_ = None
        for i in range(0, self.num_links):
            l1l1_opy_ = np.zeros((2, l11l11_opy_))
            l1l1_opy_[0:2, self.l1ll11_opy_(i):self.l1ll11_opy_(i + 1)] = -1 * np.eye(2)
            l1l1_opy_[0:2, self.l111ll_opy_(i):self.l111ll_opy_(i + 1)] = -1 * self.link_masses[i] * np.eye(2)
            l1l1_opy_[1, self.l1ll1l_opy_(i)] = -1 * 0.5 * self.link_lengths[i] * self.link_masses[i]
            if i < self.num_links - 1:
                l1l1_opy_[0:2, self.l1ll11_opy_(i + 1):self.l1ll11_opy_(i + 2)] = rot(q[i + 1])
            l1ll_opy_ = np.zeros((2, 1))
            if self.gravity:
                l1ll_opy_ = l1ll_opy_ + (-1 * 9.8 * self.link_masses[i]) * (np.dot(rot(-1 * theta[i]), (-1 * yaxis())))
            l1ll_opy_[0] = l1ll_opy_[0] + (-1) * (l1lll1l_opy_[i] * l1lll1l_opy_[i] * 0.5 * self.link_lengths[i] * self.link_masses[i])
            if i == 0:
                l1_opy_ = l1l1_opy_
                l1lll1_opy_ = l1ll_opy_
            else:
                l1_opy_ = np.concatenate((l1_opy_, l1l1_opy_))
                l1lll1_opy_ = np.concatenate((l1lll1_opy_, l1ll_opy_))
        for i in range(0, self.num_links):
            l1l1_opy_ = np.zeros((1, l11l11_opy_))
            l1l1_opy_[0, self.l1ll11_opy_(i) + 1] = self.link_lengths[i] * 0.5
            l1l1_opy_[0, self.l1ll1l_opy_(i)] = -1 * self.link_inertias[i]
            if i < self.num_links - 1:
                l1l1_opy_[0, self.l1ll11_opy_(i + 1):self.l1ll11_opy_(i + 2)] = self.link_lengths[i] * 0.5 * rot(q[i + 1])[1, :]
            l1_opy_ = np.concatenate((l1_opy_, l1l1_opy_))
            l1ll_opy_ = np.zeros((1, 1))
            l1lll1_opy_ = np.concatenate((l1lll1_opy_, l1ll_opy_))
        for i in range(1, self.num_links):
            l1l1_opy_ = np.zeros((2, l11l11_opy_))
            l1l1_opy_[0:2, self.l111ll_opy_(i):self.l111ll_opy_(i + 1)] = -1 * np.eye(2)
            l1l1_opy_[0:2, self.l111ll_opy_(i - 1):self.l111ll_opy_(i)] = rot(-1 * q[i])
            l1l1_opy_[0:2, self.l1ll1l_opy_(i - 1):self.l1ll1l_opy_(i)] = self.link_lengths[i - 1] * (
                np.dot(rot(-1 * q[i]), (1 * yaxis())))
            l1_opy_ = np.concatenate((l1_opy_, l1l1_opy_))
            l1ll_opy_ = -1 * self.link_lengths[i - 1] * l1lll1l_opy_[i - 1] * l1lll1l_opy_[i - 1] * (np.dot(rot(-1 * q[i]), (-1 * xaxis())))
            l1lll1_opy_ = np.concatenate((l1lll1_opy_, l1ll_opy_))
        assert l1_opy_.shape == (self.l1lllll_opy_() - 2, self.l1lllll_opy_())
        assert l1lll1_opy_.shape == (self.l1lllll_opy_() - 2, 1)
        for i in range(self.num_links):
            l1lll1_opy_[self.l1ll11l_opy_(i)] += l11l1_opy_[i] * self.joint_viscous_friction
        l1l1_opy_ = np.zeros((2, self.l1lllll_opy_()))
        l1l1_opy_[0:2, self.l111ll_opy_(0):self.l111ll_opy_(1)] = np.eye(2)
        l1_opy_ = np.concatenate((l1_opy_, l1l1_opy_))
        l1ll_opy_ = np.zeros((2, 1))
        l1lll1_opy_ = np.concatenate((l1lll1_opy_, l1ll_opy_))
        assert l1_opy_.shape == (5 * self.num_links, 5 * self.num_links)
        assert l1lll1_opy_.shape == (5 * self.num_links, 1)
        tau = action
        for i in range(self.num_links):
            l1lll1_opy_[self.l1ll11l_opy_(i), 0] += (tau[i + 1] if i < self.num_links - 1 else 0.0) - tau[i]
        return l1_opy_, l1lll1_opy_
    def solve(self, l1_opy_, l1lll1_opy_):
        l1ll1ll_opy_ (u"ࠥࠦࠧࠦࡓࡰ࡮ࡹࡩࡸࠦࡴࡩࡧࠣࡧࡴࡴࡳࡵࡴࡤ࡭ࡳࡺࠠ࡮ࡣࡷࡶ࡮ࡩࡥࡴࠢࡷࡳࠥࡩ࡯࡮ࡲࡸࡸࡪࠦࡡࡤࡥࡨࡰࡪࡸࡡࡵ࡫ࡲࡲࡸࠦࠢࠣࠤࠆ")
        x = np.linalg.solve(l1_opy_, l1lll1_opy_)
        self.l1ll1_opy_ = np.linalg.norm(np.dot(l1_opy_, x) - l1lll1_opy_) / self.l1lllll_opy_()
        l1ll1_opy_ = np.linalg.norm(np.dot(l1_opy_, x) - l1lll1_opy_) / self.l1lllll_opy_()
        if l1ll1_opy_ > self.residue_limit:
            print(l1ll1ll_opy_ (u"ࠫࡨࡧ࡮࡯ࡱࡷࠤࡸࡵ࡬ࡷࡧ࠯ࠤࡷ࡫ࡳࡪࡦࡸࡩࠥࢁࡽࠡࡧࡻࡧࡪ࡫ࡤࡴࠢ࡯࡭ࡲ࡯ࡴࠡࡽࢀࠫࠇ").format(l1ll1_opy_, self.residue_limit))
            self.residue_limit_flag = True
        a = x[self.l111ll_opy_(0):self.l111ll_opy_(self.num_links)]
        l11ll1_opy_ = x[self.l1ll1l_opy_(0):self.l1ll1l_opy_(self.num_links)]
        l1l1l_opy_ = l11ll1_opy_.copy()
        for i in range(self.num_links - 1, 0, -1):
            l1l1l_opy_[i] -= l1l1l_opy_[i - 1]
        return a, l1l1l_opy_
    def dynamics_step(self, state, action, dt):
        l1ll1ll_opy_ (u"ࠧࠨࠢࠡࡈࡲࡶࡼࡧࡲࡥࠢࡶ࡭ࡲࡻ࡬ࡢࡶ࡬ࡳࡳࠦࡵࡴ࡫ࡱ࡫ࠥࡋࡵ࡭ࡧࡵࠤࡲ࡫ࡴࡩࡱࡧࠤࠧࠨࠢࠈ")
        l1_opy_, l1lll1_opy_ = self.l1l11_opy_(state, action)
        a, l1l1l_opy_ = self.solve(l1_opy_, l1lll1_opy_)
        l1l11l_opy_ = self.l1lll_opy_(state, a, l1l1l_opy_, dt)
        return l1l11l_opy_
    def l1lll_opy_(self, state, a, l1l1l_opy_, dt):
        l1ll1ll_opy_ (u"ࠨࠢࠣࠢࡌࡲࡹ࡫ࡧࡳࡣࡷࡩࡸࠦࡵࡴ࡫ࡱ࡫ࠥࡋࡵ࡭ࡧࡵࠤࡲ࡫ࡴࡩࡱࡧࠤࠧࠨࠢࠉ")
        l11_opy_ = self.get_pos_0(state)
        l1ll1l1_opy_ = self.get_vel_0(state)
        q = self.get_q(state)
        l11l1_opy_ = self.get_qd(state)
        theta = self.compute_theta(q)
        l1llll_opy_ = l11l1_opy_ + l1l1l_opy_ * dt
        l1l1l1_opy_ = q + 0.5 * (l11l1_opy_ + l1llll_opy_) * dt
        l11lll_opy_ = self.compute_theta(l1l1l1_opy_)
        l11ll_opy_ = np.dot(rot(theta[0] - l11lll_opy_[0]), (l1ll1l1_opy_ + a[0:2] * dt))
        l1111l_opy_ = l11_opy_ + 0.5 * (np.dot(rot(theta[0]), l1ll1l1_opy_) + np.dot(rot(l11lll_opy_[0]), l11ll_opy_)) * dt
        l1l11l_opy_ = np.vstack([l1l1l1_opy_, l1llll_opy_])
        return l1l11l_opy_