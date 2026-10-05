# The build command supplies the base digest from third_party/ros/packages.lock.json.
ARG ROS_BASE
FROM ${ROS_BASE}
COPY third_party/ros/packages.lock.json /opt/robo-inputs/packages.lock.json
COPY deployment/ur/install_underlay.py /opt/robo-inputs/install_underlay.py
RUN python3 /opt/robo-inputs/install_underlay.py /opt/robo-inputs/packages.lock.json
COPY build/ros/requirements.txt /opt/robo-inputs/requirements.txt
COPY build/ros/*.whl /opt/robo-inputs/
RUN python3 -m venv --system-site-packages /opt/robo && \
    /opt/robo/bin/pip install --no-deps --require-hashes -r /opt/robo-inputs/requirements.txt && \
    /opt/robo/bin/pip install --no-deps /opt/robo-inputs/*.whl
ENV PATH="/opt/robo/bin:${PATH}"
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
WORKDIR /run/robo
