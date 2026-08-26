from pathlib import Path
import xml.etree.ElementTree as ET


def test_simulation_dependency_is_limited_to_the_sim_profile() -> None:
    package_xml = Path(__file__).resolve().parents[1] / "package.xml"
    root = ET.parse(package_xml).getroot()
    dependencies = [
        element
        for element in root.findall("exec_depend")
        if (element.text or "").strip() == "iii_drone_simulation"
    ]

    assert len(dependencies) == 1
    assert dependencies[0].attrib == {"condition": "$III_SYSTEM_PROFILE == 'sim'"}
