#!/usr/bin/env python3
"""Regression checks for the PLCopen simulator interface and C adapter."""

import re
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "pousHydSimulator.xml"
HEADER = ROOT / "include" / "hydro_sim_fb.h"
SOURCE = ROOT / "src" / "sim" / "hydro_sim_fb.c"
IEC_TYPE_TAGS = {"BOOL", "SINT", "USINT", "INT", "UINT", "DINT", "UDINT", "REAL", "LREAL"}

POUS = {
    "HYD_CreateSimAxis": {
        "c_type": "HYD_CREATESIMAXIS",
        "function": "__mcl_cmd_createSimAxis",
        "inputs": ["AXISTYPE", "MAXVEL", "MAXACC", "MAXDEC"],
        "outputs": ["AXISID", "DONE"],
    },
    "HYD_MoveSimAxis": {
        "c_type": "HYD_MOVESIMAXIS",
        "function": "__mcl_cmd_moveSimAxis",
        "inputs": ["ENABLE", "AXISID", "CMD_RPM", "DIRECTION"],
        "outputs": ["BUSY"],
    },
    "HYD_ReadSimAxis": {
        "c_type": "HYD_READSIMAXIS",
        "function": "__mcl_cmd_readSimAxis",
        "inputs": ["ENABLE", "AXISID"],
        "outputs": ["ACTIVE", "POS_MM", "VEL_MM_S", "PRESSURE_BAR", "BUSY"],
    },
    "HYD_PressureModel": {
        "c_type": "HYD_PRESSUREMODEL",
        "function": "__mcl_cmd_updatePressureModel",
        "inputs": [
            "ENABLE",
            "MOTOR_RPM",
            "TIME_S",
            "MODEL_TYPE",
            "K_NUM",
            "TTAU",
            "DELAYTIME",
        ],
        "outputs": [
            "ACTIVE",
            "MEASURED_PRESSURE_BAR",
            "REAL_PRESSURE_BAR",
            "ACTUAL_MOTOR_RPM",
        ],
    },
}


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def parse_xml_interface() -> dict[str, dict[str, object]]:
    root = ET.parse(XML).getroot()
    result = {}
    for pou in root.iter():
        if local_name(pou.tag) != "pou" or pou.attrib.get("pouType") != "functionBlock":
            continue
        name = pou.attrib.get("name")
        if name not in POUS:
            continue
        sections = {}
        interface = next(
            (child for child in pou if local_name(child.tag) == "interface"),
            None,
        )
        for section_name in ("inputVars", "outputVars"):
            section = next(
                (child for child in interface if local_name(child.tag) == section_name),
                None,
            ) if interface is not None else None
            variables = []
            if section is not None:
                for variable in section:
                    if local_name(variable.tag) != "variable":
                        continue
                    type_element = next(
                        (node for node in variable.iter() if local_name(node.tag) in IEC_TYPE_TAGS),
                        None,
                    )
                    variables.append((variable.attrib["name"], local_name(type_element.tag)))
            sections[section_name] = variables
        body_text = "".join(
            node.text or ""
            for node in pou.iter()
            if local_name(node.tag) == "p"
        )
        body_match = re.search(
            r"extern\s+void\s+(?P<function>__mcl_cmd_[A-Za-z0-9_]+)\s*"
            r"\(\s*(?P<c_type>HYD_[A-Z0-9_]+)\s*\*data__\s*\)",
            body_text,
        )
        sections["body"] = body_match.groupdict() if body_match else None
        result[name] = sections
    return result


def parse_header_structs() -> dict[str, list[tuple[str, str]]]:
    text = HEADER.read_text(encoding="utf-8")
    result = {}
    for match in re.finditer(r"typedef\s+struct\s*\{(.*?)\}\s*([A-Za-z0-9_]+)\s*;", text, re.S):
        body, name = match.groups()
        fields = [
            (field, c_type)
            for c_type, field in re.findall(
                r"__DECLARE_VAR\(\s*([A-Za-z0-9_]+)\s*,\s*([A-Za-z0-9_]+)\s*\)", body
            )
        ]
        if fields:
            result[name] = fields
    return result


def parse_source_functions() -> dict[str, tuple[str, str]]:
    text = SOURCE.read_text(encoding="utf-8")
    matches = list(re.finditer(
        r"void\s+(?P<function>__mcl_cmd_[A-Za-z0-9_]+)\s*\(\s*"
        r"(?P<c_type>HYD_[A-Z0-9_]+)\s*\*data__\s*\)\s*\{",
        text,
    ))
    result = {}
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        result[match.group("function")] = (match.group("c_type"), text[match.end():end])
    return result


def fail(message: str) -> int:
    print(f"FAIL: {message}")
    return 1


def main() -> int:
    xml_interfaces = parse_xml_interface()
    header_structs = parse_header_structs()
    source_functions = parse_source_functions()

    for pou_name, expected in POUS.items():
        xml = xml_interfaces.get(pou_name)
        if xml is None:
            return fail(f"missing XML POU {pou_name}")

        expected_inputs = expected["inputs"]
        expected_outputs = expected["outputs"]
        xml_inputs = [name for name, _ in xml["inputVars"]]
        xml_outputs = [name for name, _ in xml["outputVars"]]
        if xml_inputs != expected_inputs:
            return fail(f"{pou_name} XML inputs {xml_inputs} != {expected_inputs}")
        if xml_outputs != expected_outputs:
            return fail(f"{pou_name} XML outputs {xml_outputs} != {expected_outputs}")

        body = xml["body"]
        if body is None:
            return fail(f"{pou_name} XML body has no adapter declaration")
        if body["function"] != expected["function"] or body["c_type"] != expected["c_type"]:
            return fail(
                f"{pou_name} XML body {body} != "
                f"{{'function': '{expected['function']}', 'c_type': '{expected['c_type']}'}}"
            )

        fields = header_structs.get(expected["c_type"])
        if fields is None:
            return fail(f"missing C struct {expected['c_type']}")
        public_fields = [(name, c_type) for name, c_type in fields if name not in {"EN", "ENO"}]
        xml_fields = xml["inputVars"] + xml["outputVars"]
        if public_fields != xml_fields:
            return fail(f"{pou_name} XML fields {xml_fields} != C fields {public_fields}")

        source = source_functions.get(expected["function"])
        if source is None:
            return fail(f"missing C adapter function {expected['function']}")
        source_type, body = source
        if source_type != expected["c_type"]:
            return fail(f"{expected['function']} uses {source_type}, expected {expected['c_type']}")

        reads = set(re.findall(r"__GET_VAR\(data__->([A-Za-z0-9_]+)\)", body))
        writes = set(re.findall(r"__SET_VAR\(data__->,\s*([A-Za-z0-9_]+)", body))
        if not set(expected_inputs).issubset(reads):
            function = expected["function"]
            return fail(f"{function} does not read all XML inputs: {sorted(set(expected_inputs) - reads)}")
        if not set(expected_outputs).issubset(writes):
            function = expected["function"]
            return fail(f"{function} does not write all XML outputs: {sorted(set(expected_outputs) - writes)}")

    print("hydro simulator interface consistency tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
