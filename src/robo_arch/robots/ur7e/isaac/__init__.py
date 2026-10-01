"""ur7e effort articulation from its canonical, mounted-device URDF."""

from robo_arch.core.worlds.isaac.urdf import add_to_stage as add_urdf


def add_to_stage(stage, *, name, X_WB, directory, urdf):
    return add_urdf(stage, name=name, X_WB=X_WB, directory=directory, urdf=urdf)
