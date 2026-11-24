"""TagMapEKF 单元测试：间距的可观测性、规范自由度、跨对位保留、与批量解一致、一致性."""
import math

import numpy as np
import pytest

from scr_arm_controller.tag_map import TagMap
from scr_arm_controller.tag_map_slam import batch_wall_pass, TagMapEKF, TagMapEstimate
from scr_arm_controller.wall_ekf import WallEKF

R_WALL = np.diag([1.3e-3 ** 2, math.radians(0.26) ** 2])
R_TAG = np.diag([3e-3 ** 2, 0.3e-3 ** 2])
NOMINAL = TagMap({0: 0.0, 2: 0.6, 4: 1.2}, reference_id=0, sigma_position=0.005)
TRUE_U = {0: 0.0, 2: 0.604, 4: 1.195}           # 实际粘贴位置：ID 2 偏 +4 mm、ID 4 偏 −5 mm


def tag_z(X, u_tag, rng):
    return WallEKF.h_tag(X, u_tag) + rng.multivariate_normal(np.zeros(2), R_TAG)


def wall_z(X, rng):
    return WallEKF.h_wall(X) + rng.multivariate_normal(np.zeros(2), R_WALL)


def wall_pass(f, rng, u0=-0.25, u1=1.45, steps=340, slip=0.03, window=0.4, true_u=TRUE_U,
              epochs=None, see=None):
    """
    正对墙面从 u0 横移到 u1：里程计横移高估 slip，激光每步更新，视野 ±window 内的标记更新.

    返回终点真值；epochs 不为 None 时记录批量解需要的历元。see：只看这些 ID。
    """
    truth = np.array([0.70, u0, 0.0])
    step = (u1 - u0) / steps
    for k in range(steps + 1):
        e = {'odom': None, 'wall': None, 'tags': []}
        if k > 0:
            truth[1] += step
            dy = -step * (1.0 + slip)                 # 正对墙面：向右 = 机体 −y
            f.predict(0.0, dy, 0.0)
            e['odom'] = (0.0, dy, 0.0)
        zw = wall_z(truth, rng)
        f.update_wall(*zw, R_WALL)
        e['wall'] = (zw, R_WALL)
        for i, ut in true_u.items():
            if abs(truth[1] - ut) < window and (see is None or i in see):
                z = tag_z(truth, ut, rng)
                f.update_tag_id(z, R_TAG, i)
                e['tags'].append((i, z, R_TAG))
        if epochs is not None:
            epochs.append(e)
    return truth


def test_state_layout_and_prior():
    f = TagMapEKF(NOMINAL, estimate_scale=True)
    assert f.map_ids == [2, 4] and f.n == 6
    f.reset(0.7, 0.0, 0.0)
    assert f.x[4:] == pytest.approx([0.6, 1.2])
    assert f.tag_u(0) == 0.0 and f.tag_u(2) == pytest.approx(0.6) and f.tag_u(7) is None
    assert f.tag_sigma(4) == pytest.approx(0.005)
    assert f.scale[1] == pytest.approx(0.05)


def test_spacing_from_two_tags_in_view_needs_no_odometry():
    """静止时同一帧看到两个标记：间距直接可观测，与里程计、横移比例无关."""
    rng = np.random.default_rng(1)
    m = TagMap({0: 0.0, 1: 0.3}, reference_id=0, sigma_position=0.01)
    f = TagMapEKF(m, estimate_scale=True)
    truth = np.array([0.70, 0.15, 0.01])
    f.reset(*truth, sigma=(0.002, 0.05, 0.01))
    for _ in range(50):
        f.predict(0.0, 0.0, 0.0)
        f.update_wall(*wall_z(truth, rng), R_WALL)
        f.update_tag_id(tag_z(truth, 0.0, rng), R_TAG, 0)
        f.update_tag_id(tag_z(truth, 0.297, rng), R_TAG, 1)
    assert f.tag_u(1) == pytest.approx(0.297, abs=3 * f.tag_sigma(1))
    assert f.tag_sigma(1) < 0.2e-3
    assert f.scale[1] == pytest.approx(0.05, rel=0.01)    # s 不参与，仍为先验


def test_misplaced_tags_are_estimated_during_one_pass():
    rng = np.random.default_rng(2)
    f = TagMapEKF(NOMINAL, estimate_scale=True)
    f.reset(0.70, -0.25, 0.0, sigma=(0.002, 0.002, 0.002))
    truth = wall_pass(f, rng)
    est = f.map_estimate()
    for i, m, s in zip(est.ids, est.mean, est.sigma()):
        assert m == pytest.approx(TRUE_U[i], abs=max(3 * s, 0.3e-3))
        assert s < 1.5e-3
    assert f.x[1] - truth[1] == pytest.approx(0.0, abs=1.5e-3)
    assert f.scale[0] == pytest.approx(1 / 1.03 - 1, abs=3 * f.scale[1] + 1e-3)


def test_nominal_map_error_enters_one_to_one_without_estimation():
    """对照：不估计地图时，终点（只看到 ID 4）的误差约为该标记的粘贴误差."""
    rng = np.random.default_rng(2)
    f = WallEKF(estimate_scale=True)
    f.reset(0.70, -0.25, 0.0, sigma=(0.002, 0.002, 0.002))
    truth = np.array([0.70, -0.25, 0.0])
    step = 1.70 / 340
    for _ in range(340):
        truth[1] += step
        f.predict(0.0, -step * 1.03, 0.0)
        f.update_wall(*wall_z(truth, rng), R_WALL)
        for i, ut in TRUE_U.items():
            if abs(truth[1] - ut) < 0.4:
                f.update_tag(tag_z(truth, ut, rng), R_TAG, u_tag=NOMINAL.u[i], sigma_map=0.005)
    assert f.x[1] - truth[1] == pytest.approx(NOMINAL.u[4] - TRUE_U[4], abs=1.5e-3)


def test_gauge_without_the_reference_tag():
    """
    从不看到参考标记：只有“全体平移”不可观测.

    m_2、m_4 各自的 σ 停在先验附近（由先验固定规范），间距 m_4 − m_2 却被测量确定。
    """
    rng = np.random.default_rng(3)
    f = TagMapEKF(NOMINAL, estimate_scale=True)
    f.reset(0.70, 0.45, 0.0, sigma=(0.002, 0.05, 0.002))      # 沿墙位置未知（σ 5 cm）
    wall_pass(f, rng, u0=0.45, u1=1.45, steps=200, see={2, 4})
    est = f.map_estimate()
    sig = est.sigma()
    diff_var = est.cov[0, 0] + est.cov[1, 1] - 2 * est.cov[0, 1]
    assert sig[0] > 0.6 * 0.005 and sig[1] > 0.6 * 0.005
    assert math.sqrt(diff_var) < 0.3 * 0.005
    assert est.mean[1] - est.mean[0] == pytest.approx(TRUE_U[4] - TRUE_U[2],
                                                      abs=3 * math.sqrt(diff_var))


def test_map_and_scale_survive_a_new_alignment():
    rng = np.random.default_rng(4)
    f = TagMapEKF(NOMINAL, estimate_scale=True)
    f.reset(0.70, -0.25, 0.0, sigma=(0.002, 0.002, 0.002))
    wall_pass(f, rng)
    x_keep, P_keep = f.x[3:].copy(), f.P[3:, 3:].copy()
    f.clear()
    assert not f.initialized
    assert f.tag_u(2) == pytest.approx(x_keep[1]) and f.scale[0] == pytest.approx(x_keep[0])
    f.reset(0.7, 0.0, 0.0)
    assert f.x[3:] == pytest.approx(x_keep)
    assert np.allclose(f.P[3:, 3:], P_keep)                 # 含 s 与 m 的互协方差
    assert np.allclose(f.P[:3, 3:], 0.0)


def test_estimate_round_trip_with_covariance():
    rng = np.random.default_rng(5)
    f = TagMapEKF(NOMINAL, estimate_scale=False)
    f.reset(0.70, -0.25, 0.0, sigma=(0.002, 0.002, 0.002))
    wall_pass(f, rng, slip=0.0)
    est = f.map_estimate()
    d = est.to_dict()
    # 打乱协方差的 ID 顺序，载入时应按地图顺序重排
    c = d['tag_map']['covariance']
    c['ids'] = c['ids'][::-1]
    c['matrix'] = [row[::-1] for row in c['matrix'][::-1]]
    m, cov = TagMapEstimate.prior_from_dict(d)
    assert m.u[2] == pytest.approx(est.mean[0]) and m.sigma(4) == pytest.approx(est.sigma()[1])
    assert np.allclose(cov, est.cov, rtol=1e-5)
    g = TagMapEKF(m, prior_cov=cov)
    assert g.map_estimate().cov == pytest.approx(cov)


def test_joint_frame_update_matches_sequential_updates():
    rng = np.random.default_rng(6)
    truth = np.array([0.70, 0.30, 0.01])
    z0, z2 = tag_z(truth, 0.0, rng), tag_z(truth, 0.604, rng)
    a, b = TagMapEKF(NOMINAL), TagMapEKF(NOMINAL)
    for f in (a, b):
        f.reset(0.70, 0.302, 0.01, sigma=(0.002, 0.003, 0.002))
    a.update_tag_id(z0, R_TAG, 0)
    a.update_tag_id(z2, R_TAG, 2)
    assert b.update_tags([(0, z0, R_TAG), (2, z2, R_TAG)]) == [0, 2]
    # 顺序更新在两次更新之间重新线性化，差异只在微米以下
    assert a.x == pytest.approx(b.x, abs=1e-5)
    assert np.sqrt(np.diag(a.P)) == pytest.approx(np.sqrt(np.diag(b.P)), rel=0.02)


def test_ekf_matches_batch_least_squares():
    """线性近似良好时，滤波器最后时刻的地图估计与批量高斯–牛顿解一致."""
    rng = np.random.default_rng(7)
    kw = dict(alpha_trans=0.02, alpha_rot=0.02, alpha_cross=0.002)
    f = TagMapEKF(NOMINAL, estimate_scale=True, **kw)
    x0 = np.array([0.70, -0.25, 0.0])
    P0 = np.diag([0.002, 0.002, 0.002]) ** 2
    f.reset(*x0, sigma=(0.002, 0.002, 0.002))
    epochs = []
    wall_pass(f, rng, steps=120, epochs=epochs)
    sol = batch_wall_pass(epochs, NOMINAL, 0.005, x0, P0, estimate_scale=True, ekf_kw=kw)
    est = f.map_estimate()
    assert sol['m'] == pytest.approx(est.mean, abs=0.2e-3)
    assert np.sqrt(np.diag(sol['cov_m'])) == pytest.approx(est.sigma(), rel=0.25)


def test_map_estimate_is_consistent():
    """20 次独立运行：地图误差的 NEES 均值落在 χ²(2) 均值 2 的合理范围内."""
    nees = []
    for seed in range(20):
        rng = np.random.default_rng(100 + seed)
        true_u = {0: 0.0, 2: 0.6 + rng.normal(0, 0.005), 4: 1.2 + rng.normal(0, 0.005)}
        f = TagMapEKF(NOMINAL, estimate_scale=True)
        f.reset(0.70, -0.25, 0.0, sigma=(0.002, 0.002, 0.002))
        wall_pass(f, rng, steps=200, true_u=true_u)
        est = f.map_estimate()
        e = est.mean - np.array([true_u[i] for i in est.ids])
        nees.append(float(e @ np.linalg.solve(est.cov, e)))
    # 20 次平均的 χ²(40)/20 的 99 % 区间约为 [1.1, 3.2]；测量噪声小于 R 时偏小一侧
    assert 0.3 < np.mean(nees) < 3.2


class _StubNode:
    """WallStateEstimator 只需要节点的订阅 / 发布 / 日志接口."""

    def create_subscription(self, *a, **k):
        return None

    def create_publisher(self, *a, **k):
        return None

    def get_logger(self):
        class _L:
            def info(self, *a):
                pass
        return _L()


class _StubServo:
    tag_id = 0


def test_ros_wrapper_defaults_to_known_map_and_saves_estimate(tmp_path):
    import yaml
    wsr = pytest.importorskip('scr_arm_controller.wall_estimator_ros')
    plain = wsr.WallStateEstimator(_StubNode(), _StubServo(), tag_map=NOMINAL, estimate_topic='')
    assert type(plain.ekf) is WallEKF and not plain.estimate_tag_map
    assert plain.save_map(force=True) is None

    out = tmp_path / 'map_est.yaml'
    est = wsr.WallStateEstimator(_StubNode(), _StubServo(), tag_map=NOMINAL, estimate_topic='',
                                 estimate_scale=True, estimate_tag_map=True, map_output=str(out))
    assert isinstance(est.ekf, TagMapEKF) and est.tag_u(2) == pytest.approx(0.6)
    rng = np.random.default_rng(8)
    est.ekf.reset(0.70, -0.25, 0.0, sigma=(0.002, 0.002, 0.002))
    wall_pass(est.ekf, rng)
    assert est.save_map(force=True) == str(out)
    m, cov = TagMapEstimate.prior_from_dict(yaml.safe_load(out.read_text(encoding='utf-8')))
    assert m.u[2] == pytest.approx(est.ekf.tag_u(2), abs=1e-6)
    assert cov.shape == (2, 2)
    # 下一遍：载入估计作为先验
    nxt = wsr.WallStateEstimator(_StubNode(), _StubServo(), tag_map=m, estimate_topic='',
                                 estimate_tag_map=True, map_prior_cov=cov)
    assert nxt.ekf.tag_sigma(4) == pytest.approx(math.sqrt(cov[1, 1]))


def test_updates_inside_the_save_period_are_written_by_flush(tmp_path):
    """最后一站才第一次看到的标记：限频窗口内的更新由 flush_map() 写出（Gazebo 中发现的缺陷）."""
    import yaml
    wsr = pytest.importorskip('scr_arm_controller.wall_estimator_ros')
    out = tmp_path / 'map_est.yaml'
    est = wsr.WallStateEstimator(_StubNode(), _StubServo(), tag_map=NOMINAL, estimate_topic='',
                                 estimate_scale=True, estimate_tag_map=True, map_output=str(out),
                                 map_save_period=5.0)
    rng = np.random.default_rng(3)

    def saved_sigma_mm(tag):
        m = yaml.safe_load(out.read_text(encoding='utf-8'))['tag_map']
        return 1e3 * next(t['sigma'] for t in m['tags'] if t['id'] == tag)

    est.ekf.reset(0.70, -0.25, 0.0, sigma=(0.002, 0.002, 0.002))
    wall_pass(est.ekf, rng, u1=0.85, see={0, 2})            # 只看到 ID 0、2
    est._hist.append((10.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    est._map_dirty = True
    assert est.save_map() == str(out)
    assert saved_sigma_mm(4) == pytest.approx(5.0)
    est.ekf.reset(0.70, 0.85, 0.0, sigma=(0.002, 0.002, 0.002))
    wall_pass(est.ekf, rng, u0=0.85, u1=1.45, steps=150, see={2, 4})   # 最后才看到 ID 4
    est._hist.append((12.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    est._map_dirty = True
    assert est.save_map() is None                          # 限频：距上次写出不到 5 s
    assert saved_sigma_mm(4) == pytest.approx(5.0)
    assert est.flush_map() == str(out)
    assert saved_sigma_mm(4) < 2.0
    assert est.flush_map() is None                         # 没有新更新时不重复写


def test_reinitialization_after_a_new_alignment_keeps_counters(tmp_path):
    """reset() 清零计数时保留 tag_init：之后的首次观测重新初始化不应抛出 KeyError."""
    wsr = pytest.importorskip('scr_arm_controller.wall_estimator_ros')
    est = wsr.WallStateEstimator(_StubNode(), _StubServo(), tag_map=NOMINAL, estimate_topic='',
                                 estimate_tag_map=True, map_prior_sigma=0.002,
                                 map_output=str(tmp_path / 'm.yaml'))
    est.ekf.reset(0.70, 0.0, 0.0)
    est.reset()                                            # 新一次对位
    assert set(est.ekf.stats) >= {'wall', 'tag', 'tag_pair', 'tag_init'}
    est.ekf.reset(0.70, 1.2, 0.0, sigma=(0.001, 0.001, 0.001))
    far = {0: 0.0, 2: 0.6, 4: 1.225}                       # ID 4 实际偏 +25 mm（先验 σ 2 mm）
    rng = np.random.default_rng(5)
    ok, _ = est.ekf.update_tag_id(tag_z(np.array([0.70, 1.2, 0.0]), far[4], rng), R_TAG, 4)
    assert ok and est.ekf.stats['tag_init'] == 1 and est.ekf.reinit_count == 1
    assert est.ekf.tag_u(4) == pytest.approx(1.225, abs=0.003)
    est.reset()
    assert est.ekf.stats['tag_init'] == 0 and est.ekf.reinit_count == 1


def test_systematic_floor_only_inflates_the_exported_covariance():
    a, b = TagMapEKF(NOMINAL), TagMapEKF(NOMINAL, map_floor=0.4e-3)
    for f in (a, b):
        f.reset(0.70, 0.0, 0.0)
    assert np.allclose(a.P, b.P)
    d = b.map_estimate().cov - a.map_estimate().cov
    assert np.allclose(d, np.eye(2) * 0.4e-3 ** 2)


@pytest.mark.parametrize('reinit', [True, False])
def test_tag_far_outside_its_prior_is_reinitialized_on_first_sighting(reinit):
    """ID 2 实际偏 +18 mm（先验 σ 5 mm 的 3.6 倍）：首次看到时按测量重新初始化，而不是一直拒收."""
    rng = np.random.default_rng(9)
    true_u = {0: 0.0, 2: 0.618, 4: 1.1995}
    f = TagMapEKF(NOMINAL, estimate_scale=True, reinit_first_sighting=reinit)
    f.reset(0.70, -0.25, 0.0, sigma=(0.002, 0.002, 0.002))
    truth = wall_pass(f, rng, true_u=true_u)
    if reinit:
        assert f.stats['tag_init'] >= 1
        assert f.tag_u(2) == pytest.approx(0.618, abs=1e-3)
        assert f.x[1] - truth[1] == pytest.approx(0.0, abs=1.5e-3)
    else:
        assert f.stats['tag_init'] == 0
        assert abs(f.tag_u(2) - 0.618) > 5e-3            # 对照：一直被拒收，学不到


def test_loaded_estimate_is_not_reinitialized():
    m = TagMap({0: 0.0, 2: 0.6}, reference_id=0)
    f = TagMapEKF(m, prior_cov=np.array([[1e-6]]))
    f.reset(0.70, 0.3, 0.0, sigma=(0.002, 0.002, 0.002))
    z = WallEKF.h_tag((0.70, 0.3, 0.0), 0.62)
    ok, _ = f.update_tag_id(z, R_TAG, 2)
    assert not ok and f.stats['tag_init'] == 0


def test_map_estimation_requires_scale_estimation():
    from scr_arm_controller.tag_map_slam import map_estimation_allowed
    assert map_estimation_allowed(True, True) == (True, None)
    assert map_estimation_allowed(False, True) == (False, None)
    assert map_estimation_allowed(False, False) == (False, None)
    on, why = map_estimation_allowed(True, False)
    assert not on and why
    assert map_estimation_allowed(True, False, without_scale=True) == (True, None)
