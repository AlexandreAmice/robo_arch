"""Compose packaged robot and mounted-device URDFs for native conversion."""

import copy
import xml.etree.ElementTree as ET
from importlib.resources import files
from pathlib import Path

from robo_arch.core.config.declarations import SensorInstance
from robo_arch.core.worlds.assembly import PlacedRobot
from robo_arch.core.worlds.devices import DeviceDefinitions


def sensor_link(name: str, link: str) -> str:
    """Unambiguous URDF identifier; instance names cannot contain punctuation."""
    return "sensor_" + name.encode().hex() + "__" + link


def _asset(package: str, resource: str) -> ET.Element:
    path = Path(str(files(package).joinpath(resource)))
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
    model = _asset(f"robo_arch.robots.{robot.model}", "assets/model.urdf")
    model.set("name", "assembly")
    for sensor in sensors:
        parent, frame = sensor.parent.rsplit("/", 1)
        if parent != robot.name:
            continue
        definition = definitions.sensors[sensor.model]
        body = _asset(f"robo_arch.sensors.{sensor.model}", definition.resource)
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
