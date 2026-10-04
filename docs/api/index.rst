Shared Python API reference
===========================

This catalogue covers shared configuration, controller declarations and numerical
algorithms, geometry preparation, device discovery and world settings. It imports
the installed package without simulator SDKs. Python-source docstrings and
extracted C++ comments contribute to the same public reference; private extension
names are excluded.

.. toctree::
   :maxdepth: 2

   configuration
   controllers
   worlds

Start with :func:`robo_arch.core.config.loading.load_run` to load a scenario,
then :func:`robo_arch.core.worlds.assembly.resolve_devices` to inspect its physical
instances. World settings are declarations: validating them does not construct a
simulator or establish hardware support.

Coverage boundary
-----------------

The repository's documentation index is ``docs/README.md``. Architecture and
build decisions remain in its linked guides; owner READMEs contain examples,
model provenance and runtime limitations. Source docstrings own API contracts.

The following remain outside this reference and need a separate runtime audit:

* Drake/Isaac scene construction, controller adapters, visualization and execution.
* Drake-dependent transform helpers (``pose_transform`` and ``base_pose``),
  even though their containing module can be imported without Drake.
* Device-specific sensor implementations, scenario runners and evaluation APIs.
* Internal XML conversion helpers, command-line tools and schema validator methods.

These APIs are not implied to be documented by the SDK-independent build. The
catalogue is explicit; passing documentation checks does not measure completeness
outside its listed entries or automatically verify the semantics of prose.
