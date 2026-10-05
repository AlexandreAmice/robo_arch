"""Authoritative array operations shared by scalar and batched feedback laws."""


def feedback(q, v, desired_q, desired_v, kp, kd):
    """PD feedback in joint order; array implementations broadcast gains."""
    return kp * (desired_q - q) + kd * (desired_v - v)


def inverse_dynamics(state, desired_q, desired_v, kp, kd, mass, bias):
    """Full generalized effort, including the nominal gravity/damping bias."""
    count = desired_q.shape[-1]
    acceleration = feedback(
        state[..., :count], state[..., count:], desired_q, desired_v, kp, kd
    )
    return (mass @ acceleration[..., None])[..., 0] + bias
