#!/usr/bin/env python3
import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


POU_TO_C_TYPE = {
    "HYD_MoveProfile": "HYD_MOVEPROFILE",
    "HYD_LoadProfile": "HYD_LOADPROFILE",
    "HYD_Stop": "HYD_STOP",
    "HYD_Hold": "HYD_HOLD",
    "HYD_Resume": "HYD_RESUME",
    "HYD_MoveAbsolute": "HYD_MOVEABSOLUTE",
    "HYD_MoveVelocity": "HYD_MOVEVELOCITY",
    "HYD_Reset": "HYD_RESET",
    "HYD_PressureHandle": "HYD_PRESSUREHANDLE",
    "HYD_CreateMotion": "HYD_CREATEMOTION",
    "HYD_ConfigureToggleMechanism": "HYD_CONFIGURETOGGLEMECHANISM",
    "HYD_ReadToggleMechanism": "HYD_READTOGGLEMECHANISM",
    "HYD_SetAxisFeedback": "HYD_SETAXISFEEDBACK",
    "HYD_SetPumpFeedback": "HYD_SETPUMPFEEDBACK",
    "HYD_GetPumpRequest": "HYD_GETPUMPREQUEST",
    "HYD_ReadStatus": "HYD_READSTATUS",
    "HYD_ReadError": "HYD_READERROR",
    "HYD_ReadSimFeedback": "HYD_READSIMFEEDBACK",
    "HYD_ReadParameter": "HYD_READPARAMETER",
    "HYD_WriteParameter": "HYD_WRITEPARAMETER",
    "HYD_ReadBoolParameter": "HYD_READBOOLPARAMETER",
    "HYD_WriteBoolParameter": "HYD_WRITEBOOLPARAMETER",
}

POU_ALLOWED_C_ONLY_FIELDS = {
    "HYD_GetPumpRequest": {"ALLOW_NEGATIVE0"},
}

XML_TO_C_FIELD = {
    "ContinuousUpdate": "CONTINUOUSUPDATE",
    "Execute0": "EXECUTE0",
    "Done0": "DONE0",
    "Active0": "ACTIVE0",
}


def normalize_field(name: str) -> str:
    return XML_TO_C_FIELD.get(name, name).upper()


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def children_named(elem: ET.Element, name: str) -> list[ET.Element]:
    return [child for child in list(elem) if local_name(child.tag) == name]


def descendants_named(elem: ET.Element, name: str) -> list[ET.Element]:
    return [child for child in elem.iter() if local_name(child.tag) == name]


def parse_xml(xml_path: Path):
    tree = ET.parse(xml_path)
    root = tree.getroot()

    data_types = {}
    for data_type in descendants_named(root, "dataType"):
        name = data_type.attrib.get("name")
        if not name:
            continue
        fields = []
        for base_type in children_named(data_type, "baseType"):
            for struct in children_named(base_type, "struct"):
                fields.extend(normalize_field(v.attrib["name"]) for v in children_named(struct, "variable"))
        if fields:
            data_types[name.upper()] = fields

    pous = {}
    for pou in descendants_named(root, "pou"):
        if pou.attrib.get("pouType") != "functionBlock":
            continue
        pou_name = pou.attrib.get("name")
        if not pou_name:
            continue
        fields = []
        interface = next(iter(children_named(pou, "interface")), None)
        if interface is None:
            continue
        for section in ("inputVars", "outputVars", "localVars"):
            for section_elem in children_named(interface, section):
                for var in children_named(section_elem, "variable"):
                    fields.append(normalize_field(var.attrib["name"]))
        if fields:
            pous[pou_name] = fields

    return data_types, pous


def extract_block(text: str, start: int, open_char: str, close_char: str) -> tuple[str, int]:
    depth = 0
    i = start
    begin = -1
    while i < len(text):
        ch = text[i]
        if ch == open_char:
            if depth == 0:
                begin = i + 1
            depth += 1
        elif ch == close_char:
            depth -= 1
            if depth == 0:
                return text[begin:i], i + 1
        i += 1
    raise ValueError("unbalanced block")


def parse_header(header_path: Path):
    text = header_path.read_text(encoding="utf-8")
    structs = {}

    typedef_pattern = re.compile(r"typedef\s+struct\s*\{", re.MULTILINE)
    pos = 0
    while True:
        match = typedef_pattern.search(text, pos)
        if not match:
            break
        body, end = extract_block(text, match.end() - 1, "{", "}")
        tail = re.match(r"\s*([A-Za-z0-9_]+)\s*;", text[end:])
        if tail:
            name = tail.group(1)
            fields = re.findall(r"__DECLARE_VAR\(\s*[A-Za-z0-9_]+\s*,\s*([A-Za-z0-9_]+)\s*\)", body)
            if fields:
                structs[name.upper()] = [field.upper() for field in fields]
        pos = end

    for match in re.finditer(r"__DECLARE_STRUCT_TYPE\(\s*([A-Za-z0-9_]+)\s*,", text, re.MULTILINE):
        name = match.group(1)
        start = text.find("(", match.start())
        body, _ = extract_block(text, start, "(", ")")
        comma = body.find(",")
        field_body = body[comma + 1 :] if comma >= 0 else body
        fields = re.findall(r"\b([A-Za-z0-9_]+)\s+([A-Za-z0-9_]+)\s*;", field_body)
        if fields:
            structs[name.upper()] = [field_name.upper() for _, field_name in fields]

    return structs


def compare_sequences(
    label: str,
    expected: list[str],
    actual: list[str],
    errors: list[str],
    allowed_extra: set[str] | None = None,
) -> None:
    allowed_extra = allowed_extra or set()
    unexpected = [field for field in actual if field not in set(expected) | allowed_extra]
    filtered_actual = [field for field in actual if field in set(expected)]
    if expected != filtered_actual or unexpected:
        errors.append(
            f"{label} mismatch\n"
            f"  xml: {expected}\n"
            f"  c  : {filtered_actual}\n"
            f"  unexpected c-only fields: {unexpected}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", required=True, type=Path)
    parser.add_argument("--header", required=True, type=Path)
    args = parser.parse_args()

    xml_types, xml_pous = parse_xml(args.xml)
    header_structs = parse_header(args.header)

    errors: list[str] = []

    for xml_name, expected in xml_types.items():
        actual = header_structs.get(xml_name)
        if actual is None:
            errors.append(f"{xml_name} missing in header")
            continue
        compare_sequences(xml_name, expected, actual, errors)

    for pou_name, expected in xml_pous.items():
        c_name = POU_TO_C_TYPE.get(pou_name)
        if c_name is None:
            continue
        actual = header_structs.get(c_name)
        if actual is None:
            errors.append(f"{pou_name} ({c_name}) missing in header")
            continue
        allowed_extra = {"EN", "ENO"} | POU_ALLOWED_C_ONLY_FIELDS.get(pou_name, set())
        compare_sequences(pou_name, expected, actual, errors, allowed_extra)

    if errors:
        print("Interface layout consistency check failed:")
        for err in errors:
            print(err)
        return 1

    print("Interface layout consistency check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
