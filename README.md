# Robotics architecture design

Design documents for a shared autonomy stack across Drake, hardware and batched simulation. No implementation yet.

- [Architecture](docs/architecture.md): scenario, autonomy, world, composition and shared execution.
- [Build and layout](docs/build_and_layout.md): application-grouped configuration, component folders, Python/C++ and packaging.
- [Implementation plan](docs/implementation_tasks.md): private GitHub setup, parallel work packages, dependencies, and controller-reuse acceptance gates.
- [Agent guidance](AGENTS.md): scope, concise documentation and coding style.

The earlier scenario/composition and tensor-execution documents have been consolidated into the architecture; these links are the current reading list.

Clone the private repository with an authorized GitHub account:

```sh
gh repo clone AlexandreAmice/robo_arch
cd robo_arch
```
