Shared controller APIs
======================

Joint control
-------------

.. autoclass:: robo_arch.core.controllers.joint_pd.definition.JointPdParameters

.. autofunction:: robo_arch.core.controllers.joint_pd.native.compute

.. autoclass:: robo_arch.core.controllers.joint_tracking.definition.JointTrackingParameters

Protection declarations
-----------------------

.. autoclass:: robo_arch.core.controllers.cbf.definition.Sphere

.. autoclass:: robo_arch.core.controllers.cbf.definition.SpherePair

.. autoclass:: robo_arch.core.controllers.cbf.definition.Plane

.. autoclass:: robo_arch.core.controllers.cbf.definition.SpherePlanePair

.. autoclass:: robo_arch.core.controllers.cbf.definition.CbfParameters

.. autoclass:: robo_arch.core.controllers.cbf.config.ProtectionParameters

Collision coverage and assembly
-------------------------------

.. autoclass:: robo_arch.core.controllers.cbf.geometry.SphereProfile

.. autofunction:: robo_arch.core.controllers.cbf.geometry.cover_box

.. autofunction:: robo_arch.core.controllers.cbf.geometry.collision_boxes

.. autofunction:: robo_arch.core.controllers.cbf.geometry.load_sphere_profile

.. autoclass:: robo_arch.core.controllers.cbf.assembly.ProtectionGeometry

.. autofunction:: robo_arch.core.controllers.cbf.assembly.resolve_geometry

Numerical barrier rows
----------------------

.. autoclass:: robo_arch.core.controllers.cbf.barrier.BarrierConstraint

.. autoclass:: robo_arch.core.controllers.cbf.barrier.BarrierConstraints
   :members: rows

.. autofunction:: robo_arch.core.controllers.cbf.barrier.sphere_constraint

.. autofunction:: robo_arch.core.controllers.cbf.barrier.sphere_constraints

.. autofunction:: robo_arch.core.controllers.cbf.barrier.plane_constraints
