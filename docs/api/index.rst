Python API reference
====================

These selected public APIs are documented from the installed package. Python
docstrings and extracted C++ comments contribute to the same reference.
Simulator adapters are described in the owning packages' README files; importing
this reference requires neither a simulator SDK nor a running world.

Joint PD control
----------------

.. autoclass:: robo_arch.core.controllers.joint_pd.definition.JointPdParameters

.. autofunction:: robo_arch.core.controllers.joint_pd.native.compute

Sphere separation constraints
-----------------------------

.. autoclass:: robo_arch.core.controllers.cbf.barrier.BarrierConstraint

.. autoclass:: robo_arch.core.controllers.cbf.barrier.BarrierConstraints
   :members: rows

.. autofunction:: robo_arch.core.controllers.cbf.barrier.sphere_constraint

.. autofunction:: robo_arch.core.controllers.cbf.barrier.sphere_constraints

Configuration loading
---------------------

.. autofunction:: robo_arch.core.config.loading.resolve_resource

.. autofunction:: robo_arch.core.config.loading.load_robot

.. autofunction:: robo_arch.core.config.loading.load_world

.. autofunction:: robo_arch.core.config.loading.load_run

.. autoclass:: robo_arch.core.config.declarations.RunConfiguration

.. autoclass:: robo_arch.core.config.declarations.SceneConfiguration
