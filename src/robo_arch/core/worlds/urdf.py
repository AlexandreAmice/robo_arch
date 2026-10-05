"""Compose packaged robot and mounted-device URDFs for native conversion."""

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

from robo_arch.core.config.declarations import SensorInstance
from robo_arch.core.config.loading import resolve_resource
from robo_arch.core.worlds.assembly import PlacedRobot
from robo_arch.core.worlds.devices import DeviceDefinitions


def sensor_link(name: str, link: str) -> str:
    """Unambiguous URDF identifier; instance names cannot contain punctuation."""
    return "sensor_" + name.encode().hex() + "__" + link


def _asset(reference: str) -> ET.Element:
    path = resolve_resource(reference)
    if path.suffix != ".urdf":
        raise ValueError(f"Unsupported robot assembly asset: {reference}")
    model = ET.parse(path).getroot()
    for mesh in model.findall(".//mesh"):
        mesh.set("filename", str((path.parent / mesh.get("filename")).resolve()))
    return model


def compose(
    robot: PlacedRobot,
    sensors: tuple[SensorInstance, ...],
    definitions: DeviceDefinitions,
    destination: Path,
) -> None:
    """Preserve all sensing joints and inertias, including disabled observations."""
    model = _asset(definitions.robots[robot.model].asset)
    model.set("name", "assembly")
    for sensor in sensors:
        parent, frame = sensor.parent.rsplit("/", 1)
        if parent != robot.name:
            continue
        definition = definitions.sensors[sensor.model]
        body = _asset(
            f"package://robo_arch/sensors/{sensor.model}/{definition.resource}"
        )
        for original in body:
            node = copy.deepcopy(original)
            for element in node.iter():
                for attribute in ("name", "link"):
                    if attribute in element.attrib:
                        element.set(
                            attribute, sensor_link(sensor.name, element.get(attribute))
                        )
            model.append(node)
        joint = ET.SubElement(
            model, "joint", name=sensor_link(sensor.name, "attachment"), type="fixed"
        )
        ET.SubElement(joint, "parent", link=frame)
        ET.SubElement(
            joint, "child", link=sensor_link(sensor.name, definition.base_frame)
        )
        ET.SubElement(
            joint,
            "origin",
            xyz=" ".join(map(str, sensor.pose.translation)),
            rpy=" ".join(map(str, sensor.pose.rpy)),
        )
    ET.indent(model, space="  ")
    ET.ElementTree(model).write(destination, encoding="unicode", xml_declaration=True)


def robot_link(name: str, link: str) -> str:
    """Lossless identifier for a device inside a compound URDF."""
    return "robot_" + name.encode().hex() + "__" + link


def compose_mechanism(mechanism, sensors, definitions, destination: Path):
    """Compose a connected tree and return each device's native joint names.

    The physical root keeps its original names for existing scalar consumers.
    Child devices use deterministic names; no actuator or inertia is discarded.
    """
    from robo_arch.core.worlds.assembly import attachment_pose

    root_robot = mechanism.robots[0]
    model = _asset(definitions.robots[root_robot.model].asset)
    model.set("name", "assembly")
    names = {}

    def native(device, name):
        return name if device == mechanism.root else robot_link(device, name)

    def append_asset(asset, rename):
        for original in asset:
            node = copy.deepcopy(original)
            for element in node.iter():
                for attribute in ("name", "link", "joint"):
                    if attribute in element.attrib:
                        element.set(attribute, rename(element.get(attribute)))
            model.append(node)

    for robot in mechanism.robots:
        definition = definitions.robots[robot.model]
        names[robot.name] = tuple(native(robot.name, j) for j in definition.joints)
        if robot.parent is None:
            continue
        parent, frame = robot.parent.rsplit("/", 1)
        asset = _asset(definition.asset)
        links = {link.get("name") for link in asset.findall("link")}
        if (robot.mount_frame or definition.base_frame) != definition.base_frame:
            raise ValueError(
                "Compound URDF child mount must be its declared base frame"
            )
        if definition.base_frame not in links:
            raise ValueError(f"Missing child base frame for {robot.name}")
        if native(parent, frame) not in {
            link.get("name") for link in model.findall("link")
        }:
            raise ValueError(f"Missing parent frame {robot.parent}")
        append_asset(asset, lambda name, device=robot.name: native(device, name))
        joint = ET.SubElement(
            model, "joint", name=robot_link(robot.name, "mount"), type="fixed"
        )
        ET.SubElement(joint, "parent", link=native(parent, frame))
        ET.SubElement(joint, "child", link=native(robot.name, definition.base_frame))
        transform = attachment_pose(robot)
        from pydrake.math import RollPitchYaw

        ET.SubElement(
            joint,
            "origin",
            xyz=" ".join(map(str, transform.translation())),
            rpy=" ".join(map(str, RollPitchYaw(transform.rotation()).vector())),
        )
    members = {robot.name for robot in mechanism.robots}
    for sensor in sensors:
        parent, frame = sensor.parent.rsplit("/", 1)
        if parent not in members:
            continue
        definition = definitions.sensors[sensor.model]
        append_asset(
            _asset(f"package://robo_arch/sensors/{sensor.model}/{definition.resource}"),
            lambda name, device=sensor.name: sensor_link(device, name),
        )
        joint = ET.SubElement(
            model, "joint", name=sensor_link(sensor.name, "attachment"), type="fixed"
        )
        ET.SubElement(joint, "parent", link=native(parent, frame))
        ET.SubElement(
            joint, "child", link=sensor_link(sensor.name, definition.base_frame)
        )
        pose = sensor.calibration.pose if sensor.calibration else sensor.pose
        ET.SubElement(
            joint,
            "origin",
            xyz=" ".join(map(str, pose.translation)),
            rpy=" ".join(map(str, pose.rpy)),
        )
    ET.indent(model, space="  ")
    ET.ElementTree(model).write(destination, encoding="unicode", xml_declaration=True)
    return names
