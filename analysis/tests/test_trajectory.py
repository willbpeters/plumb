import numpy as np
import pytest

from plumb import quat
from plumb.trajectory import ArcType, StrokeParams, generate


@pytest.mark.parametrize("tempo", [1.5, 2.0, 2.5, 3.0])
def test_impact_occurs_exactly_where_swing_angle_crosses_zero(tempo):
    """Must hold exactly, for every tempo, not just to within a sample.

    The face rotation carries an `arc_gain * theta` term that is designed to
    vanish at impact. If theta is merely near zero there, that term leaks into
    ground truth and the leak scales with arc_gain -- which would make recovery
    look arc-type-dependent and fake a violation of invariant 1."""
    t = generate(StrokeParams(tempo_ratio=tempo))
    assert t.theta[t.impact_index] == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("tempo", [1.5, 2.0, 2.5, 3.0])
def test_achieved_tempo_ratio_is_recorded_and_close_to_requested(tempo):
    """A 500 Hz grid cannot land an arbitrary tempo ratio on a whole sample, so
    the generator snaps to samples and reports what it actually produced."""
    t = generate(StrokeParams(tempo_ratio=tempo))
    backswing = t.time[t.transition_index] - t.time[t.address_end_index]
    downswing = t.time[t.impact_index] - t.time[t.transition_index]
    assert backswing / downswing == pytest.approx(t.true_tempo_ratio, rel=1e-12)
    assert t.true_tempo_ratio == pytest.approx(tempo, rel=5e-3)


@pytest.mark.parametrize("arc", list(ArcType))
def test_face_angle_at_impact_is_independent_of_arc_type(arc):
    """The recovered quantity must not depend on how much the face rotates
    during the stroke. This is invariant 1 expressed in the generator."""
    p = StrokeParams(face_angle_at_impact_deg=2.0, arc_type=arc)
    t = generate(p)
    assert t.true_face_angle_deg == pytest.approx(2.0, abs=1e-9)


def test_projected_face_angle_differs_from_shaft_rotation_when_lie_is_nonzero():
    """Guards the refinement in the plan preamble: rotating the shaft by phi
    does not move the face normal by phi in the ground plane."""
    p = StrokeParams(face_angle_at_impact_deg=2.0, lie_angle_deg=20.0)
    t = generate(p)
    phi_impact = np.degrees(t.phi[t.impact_index])
    assert phi_impact == pytest.approx(2.128, abs=1e-3)
    assert phi_impact != pytest.approx(2.0, abs=1e-3)


def test_face_normal_azimuth_matches_requested_face_angle():
    """Independent check of the ground truth, computed from the attitude
    rather than from the generator's own parameterization."""
    p = StrokeParams(face_angle_at_impact_deg=2.0, lie_angle_deg=20.0)
    t = generate(p)
    q0 = t.q_true[t.address_end_index]
    qi = t.q_true[t.impact_index]
    g_world = np.array([0.0, 0.0, 1.0])
    face_body = np.array([1.0, 0.0, 0.0])

    def azimuth(q):
        n = quat.rotate(q, face_body)
        n = n - np.dot(n, g_world) * g_world
        return np.arctan2(n[1], n[0])

    delta = np.degrees(azimuth(qi) - azimuth(q0))
    assert delta == pytest.approx(2.0, abs=1e-6)


def test_angular_velocity_matches_numerical_differentiation_of_attitude():
    """The analytic omega must agree with differencing the attitude it claims
    to describe. This catches a sign error or an axis mix-up in either one.

    Central difference, not forward. A forward difference estimates the AVERAGE
    rate over [t, t+dt], which differs from the rate AT t by (dt/2)*omega_dot --
    about 4e-3 rad/s at the phase boundaries where the raised-cosine profile's
    curvature peaks, which exceeds any tolerance worth asserting here.

    The residual that remains after central differencing is O(|omega|^2 * dt):
    the difference is expressed in the body frame at i-1 rather than at i. That
    floors this check at roughly 1e-3 rad/s, which is fine for its purpose -- the
    errors it exists to catch are sign flips and axis swaps, which show up at
    order |omega| itself, a thousand times larger."""
    t = generate(StrokeParams())
    dt = t.time[1] - t.time[0]
    for i in range(t.address_end_index + 10, t.impact_index, 37):
        dq = quat.multiply(quat.conjugate(t.q_true[i - 1]), t.q_true[i + 1])
        omega_numeric = dq[1:] / dt
        np.testing.assert_allclose(omega_numeric, t.omega_true[i], atol=2e-3)


def test_zero_torque_case_really_has_almost_no_face_rotation():
    straight = generate(StrokeParams(arc_type=ArcType.NEAR_ZERO_ROTATION))
    arced = generate(StrokeParams(arc_type=ArcType.ARCED))
    assert np.ptp(straight.phi) < 0.1 * np.ptp(arced.phi)
