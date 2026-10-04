Configuration and declarations
==============================

Loading and resource references
--------------------------------------------------

.. autofunction:: robo_arch.core.config.loading.load_run

.. autofunction:: robo_arch.core.config.loading.load_world

.. autofunction:: robo_arch.core.config.loading.load_robot

.. autofunction:: robo_arch.core.config.loading.read_validated_yaml

.. autofunction:: robo_arch.core.config.loading.resolve_resource

.. autofunction:: robo_arch.core.config.resources.validate_package_reference

.. autofunction:: robo_arch.core.config.worlds.parse_world

Validation policy
-----------------

.. autoclass:: robo_arch.core.config.schema.Schema

.. autoclass:: robo_arch.core.config.parameters.Parameters

Physical composition and run selections
--------------------------------------------------

.. autoclass:: robo_arch.core.config.declarations.Pose

.. autoclass:: robo_arch.core.config.declarations.RobotInstance

.. autoclass:: robo_arch.core.config.declarations.SensorInstance

.. autoclass:: robo_arch.core.config.declarations.RobotSystem

.. autoclass:: robo_arch.core.config.declarations.ObjectInstance

.. autoclass:: robo_arch.core.config.declarations.SceneConfiguration

.. autoclass:: robo_arch.core.config.declarations.TaskSelection

.. autoclass:: robo_arch.core.config.declarations.AutonomySelection

.. autoclass:: robo_arch.core.config.declarations.RunConfiguration
   :members: scene, world, time_step, resources

Reusable model metadata
-----------------------

.. autoclass:: robo_arch.core.config.declarations.RobotDefinition

.. autoclass:: robo_arch.core.config.declarations.SensorDefinition

.. autoclass:: robo_arch.core.config.declarations.ObjectDefinition
