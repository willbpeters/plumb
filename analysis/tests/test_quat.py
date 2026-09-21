import numpy as np
import pytest

from plumb import quat


def test_identity_rotates_nothing():
    v = np.array([1.0, 2.0, 3.0])
    np.testing.assert_allclose(quat.rotate(quat.identity(), v), v, atol=1e-12)


def test_rotation_about_z_moves_x_to_y():
    q = quat.from_axis_angle(np.array([0.0, 0.0, 1.0]), np.pi / 2)
    got = quat.rotate(q, np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(got, [0.0, 1.0, 0.0], atol=1e-12)


def test_multiply_composes_in_body_to_reference_order():
    # q_ab then q_bc applied to a body vector must equal the composed rotation.
    q_ab = quat.from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.3)
    q_bc = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.4)
    v = np.array([0.2, -0.5, 0.9])
    direct = quat.rotate(quat.multiply(q_ab, q_bc), v)
    stepwise = quat.rotate(q_ab, quat.rotate(q_bc, v))
    np.testing.assert_allclose(direct, stepwise, atol=1e-12)


def test_conjugate_inverts():
    q = quat.from_axis_angle(np.array([1.0, 2.0, 3.0]), 0.7)
    v = np.array([0.1, 0.2, 0.3])
    back = quat.rotate(quat.conjugate(q), quat.rotate(q, v))
    np.testing.assert_allclose(back, v, atol=1e-12)


def test_integrate_constant_rate_matches_closed_form():
    # 1 rad/s about Z for 1 s, in 1000 steps, must land on 1 rad.
    q = quat.identity()
    omega = np.array([0.0, 0.0, 1.0])
    dt = 1e-3
    for _ in range(1000):
        q = quat.integrate(q, omega, dt)
    expected = quat.from_axis_angle(np.array([0.0, 0.0, 1.0]), 1.0)
    np.testing.assert_allclose(quat.rotate(q, np.array([1.0, 0.0, 0.0])),
                               quat.rotate(expected, np.array([1.0, 0.0, 0.0])),
                               atol=1e-9)


def test_twist_about_axis_extracts_pure_rotation():
    axis = np.array([0.0, 0.0, 1.0])
    q = quat.from_axis_angle(axis, 0.25)
    assert quat.twist_angle(q, axis) == pytest.approx(0.25, abs=1e-12)


def test_twist_ignores_rotation_perpendicular_to_axis():
    # A pure rotation about X has no twist about Z.
    q = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.4)
    assert quat.twist_angle(q, np.array([0.0, 0.0, 1.0])) == pytest.approx(0.0, abs=1e-12)


def test_from_rotation_matrix_roundtrip():
    q = quat.from_axis_angle(np.array([0.3, -0.7, 0.2]), 1.1)
    np.testing.assert_allclose(
        quat.rotate(quat.from_matrix(quat.to_matrix(q)), np.array([1.0, 0.0, 0.0])),
        quat.rotate(q, np.array([1.0, 0.0, 0.0])),
        atol=1e-12,
    )


def test_from_rotation_matrix_roundtrip_trace_negative():
    # A 180-degree rotation gives trace(m) == -1, exercising the branch of
    # from_matrix that the near-identity roundtrip test above never reaches.
    q = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), np.pi)
    m = quat.to_matrix(q)
    assert np.trace(m) == pytest.approx(-1.0, abs=1e-12)
    np.testing.assert_allclose(
        quat.rotate(quat.from_matrix(m), np.array([0.0, 1.0, 0.0])),
        quat.rotate(q, np.array([0.0, 1.0, 0.0])),
        atol=1e-12,
    )
