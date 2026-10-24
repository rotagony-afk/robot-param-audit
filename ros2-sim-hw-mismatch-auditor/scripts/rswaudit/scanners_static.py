"""Static scanners: URDF/Xacro, ROS 2 YAML, launch files and C++/Python sources.

Design rules that matter for correctness:

* Every extracted value remembers the exact line and column of its literal, so
  a later patch proposal can rewrite precisely that number and nothing else.
* Values that are not plain numbers (xacro/launch substitutions such as
  dollar-brace expressions) are kept as *unresolved* facts instead of being
  guessed. An unresolved fact never takes part in a numeric comparison.
* Nothing is inferred from a filename. A parameter exists only if a value was
  actually read from a file.
"""

from __future__ import annotations

import bisect
import re
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Iterator

from .evidence import Evidence, Fact, fmt
from . import registry

EXPR_MARKER = "$" + "{"   # built at runtime so this source stays literal-safe
LAUNCH_MARKER = "$("

_NUM_RE = re.compile(r"^[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?$")
_NUMISH_RE = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")
_TRIM_EXCERPT = 160


def line_starts(text: str) -> list[int]:
    starts = [0]
    for idx, ch in enumerate(text):
        if ch == "\n":
            starts.append(idx + 1)
    return starts


def pos_to_linecol(starts: list[int], pos: int) -> tuple[int, int]:
    line = bisect.bisect_right(starts, pos)
    col = pos - starts[line - 1] + 1
    return line, col


def line_text(text: str, lineno: int) -> str:
    lines = text.splitlines()
    if 1 <= lineno <= len(lines):
        raw = lines[lineno - 1].strip()
        return raw[:_TRIM_EXCERPT]
    return ""


def parse_number(token: str) -> float | None:
    if token is None:
        return None
    tok = token.strip().replace("_", "")
    if len(tok) >= 2 and tok[0] == tok[-1] and tok[0] in "\"'":
        tok = tok[1:-1].strip()
    if not _NUM_RE.match(tok):
        return None
    try:
        return float(tok)
    except ValueError:
        return None


def is_expression(token: str) -> bool:
    return EXPR_MARKER in token or LAUNCH_MARKER in token or token.strip().startswith("<")


def literal_span(value: str) -> tuple[str, int]:
    """Return (literal, 0-based offset of the literal inside value)."""
    start = 0
    while start < len(value) and value[start].isspace():
        start += 1
    end = len(value)
    while end > start and value[end - 1].isspace():
        end -= 1
    body = value[start:end]
    if len(body) >= 2 and body[0] == body[-1] and body[0] in "\"'":
        body = body[1:-1]
        start += 1
    return body, start


# --------------------------------------------------------------------------- #
# XML (URDF / SDF fragments)
# --------------------------------------------------------------------------- #

_XML_TOKEN_RE = re.compile(
    r"<!--.*?-->"
    r"|<!\[CDATA\[.*?\]\]>"
    r"|<\?.*?\?>"
    r"|<![A-Za-z]+[^>]*>"
    r"|<(/?)([A-Za-z_][\w:.\-]*)((?:[^<>\"']|\"[^\"]*\"|'[^']*')*?)(/?)>",
    re.S,
)
_XML_ATTR_RE = re.compile(r"([A-Za-z_][\w:.\-]*)\s*=\s*(\"([^\"]*)\"|'([^']*)')")


@dataclass
class XmlTag:
    name: str
    attrs: dict[str, str]
    attr_pos: dict[str, tuple[int, int]]   # attr -> (line, col of value)
    line: int
    closing: bool = False
    self_closing: bool = False
    text: str = ""


def scan_xml_tags(text: str, relpath: str) -> list[XmlTag]:
    starts = line_starts(text)
    tags: list[XmlTag] = []
    for match in _XML_TOKEN_RE.finditer(text):
        token = match.group(0)
        if token.startswith("<!") or token.startswith("<?"):
            continue
        closing = match.group(1) == "/"
        name = match.group(2)
        attrs: dict[str, str] = {}
        attr_pos: dict[str, tuple[int, int]] = {}
        attrs_text = match.group(3) or ""
        base = match.start(3)
        for am in _XML_ATTR_RE.finditer(attrs_text):
            key = am.group(1)
            val = am.group(3) if am.group(3) is not None else am.group(4)
            attrs[key] = val
            vstart = base + (am.start(3) if am.group(3) is not None else am.start(4))
            attr_pos[key] = pos_to_linecol(starts, vstart)
        # immediate text content, needed by SDF (<radius>0.075</radius>)
        tail_text = ""
        if not closing and match.group(4) != "/":
            next_lt = text.find("<", match.end())
            if next_lt == -1:
                next_lt = len(text)
            tail_text = text[match.end():next_lt]
        tags.append(XmlTag(
            name=name,
            attrs=attrs,
            attr_pos=attr_pos,
            line=pos_to_linecol(starts, match.start())[0],
            closing=closing,
            self_closing=(match.group(4) == "/"),
            text=tail_text,
        ))
    return tags


@dataclass
class RobotModel:
    path: str = ""
    links: dict[str, dict] = field(default_factory=dict)
    joints: dict[str, dict] = field(default_factory=dict)
    gazebo_rates: list[dict] = field(default_factory=list)
    plugins: list[str] = field(default_factory=list)
    wheel_links: list[str] = field(default_factory=list)


def _vec(text: str) -> tuple[float, ...] | None:
    parts = text.replace(",", " ").split()
    if not parts:
        return None
    out = []
    for part in parts:
        val = parse_number(part)
        if val is None:
            return None
        out.append(val)
    return tuple(out)


def extract_urdf(text: str, relpath: str) -> tuple[list[Fact], RobotModel]:
    tags = scan_xml_tags(text, relpath)
    model = RobotModel(path=relpath)
    facts: list[Fact] = []
    open_stack: list[XmlTag] = []

    def ancestor(name: str) -> XmlTag | None:
        for tag in reversed(open_stack):
            if tag.name == name:
                return tag
        return None

    for tag in tags:
        if tag.closing:
            for idx in range(len(open_stack) - 1, -1, -1):
                if open_stack[idx].name == tag.name:
                    del open_stack[idx:]
                    break
            continue
        parent = ancestor("link")
        parent_name = parent.attrs.get("name", "") if parent else ""
        tag_kind = ancestor("inertial") is not None

        if tag.name == "link":
            name = tag.attrs.get("name", "")
            model.links.setdefault(name, {"name": name, "mass": None, "inertia": None,
                                          "mass_ev": None, "inertia_ev": None,
                                          "radius": None, "radius_ev": None,
                                          "line": tag.line})
        elif tag.name == "joint":
            name = tag.attrs.get("name", "")
            model.joints.setdefault(name, {
                "name": name, "type": tag.attrs.get("type", ""),
                "parent": "", "child": "", "axis": None, "axis_ev": None,
                "origin": None, "origin_ev": None, "joint_ev": None,
                "axis_line": tag.line,
            })
            model.joints[name]["joint_ev"] = Evidence(relpath, tag.line, None,
                                                      line_text(text, tag.line),
                                                      "joint " + name)
        elif tag.name == "mass" and tag_kind and parent_name:
            value = parse_number(tag.attrs.get("value", ""))
            line, col = tag.attr_pos.get("value", (tag.line, None))
            if value is not None:
                model.links.setdefault(parent_name, {})["mass"] = value
                model.links[parent_name]["mass_ev"] = Evidence(
                    relpath, line, col, line_text(text, line), "link " + parent_name + " mass")
                facts.append(Fact("mass", "mass", value, tag.attrs.get("value", ""),
                                  "kg", parent_name, "urdf",
                                  model.links[parent_name]["mass_ev"]))
        elif tag.name == "inertia" and tag_kind and parent_name:
            diag = [parse_number(tag.attrs.get(k, "0")) for k in ("ixx", "iyy", "izz")]
            model.links.setdefault(parent_name, {})["inertia"] = [d or 0.0 for d in diag]
            model.links[parent_name]["inertia_ev"] = Evidence(
                relpath, tag.line, None, line_text(text, tag.line),
                "link " + parent_name + " inertia")
        elif tag.name == "cylinder" and parent_name:
            value = parse_number(tag.attrs.get("radius", ""))
            line, col = tag.attr_pos.get("radius", (tag.line, None))
            if value is not None and "wheel" in parent_name.lower():
                ev = Evidence(relpath, line, col, line_text(text, line),
                              "link " + parent_name + " cylinder radius")
                model.links.setdefault(parent_name, {})["radius"] = value
                model.links[parent_name]["radius_ev"] = ev
                if parent_name not in model.wheel_links:
                    model.wheel_links.append(parent_name)
                facts.append(Fact("wheel_radius", "cylinder/@radius", value,
                                  tag.attrs.get("radius", ""), "m", parent_name,
                                  "urdf", ev,
                                  note="geometry radius of wheel link"))
        elif tag.name == "parent" and ancestor("joint") is not None:
            joint = ancestor("joint").attrs.get("name", "")
            if joint in model.joints:
                model.joints[joint]["parent"] = tag.attrs.get("link", "")
        elif tag.name == "child" and ancestor("joint") is not None:
            joint = ancestor("joint").attrs.get("name", "")
            if joint in model.joints:
                model.joints[joint]["child"] = tag.attrs.get("link", "")
        elif tag.name == "origin" and ancestor("joint") is not None:
            joint = ancestor("joint").attrs.get("name", "")
            line, col = tag.attr_pos.get("xyz", (tag.line, None))
            xyz = _vec(tag.attrs.get("xyz", ""))
            if joint in model.joints and model.joints[joint]["origin"] is None and xyz:
                model.joints[joint]["origin"] = xyz
                model.joints[joint]["origin_ev"] = Evidence(
                    relpath, line, col, line_text(text, line), "joint " + joint + " origin")
        elif tag.name == "axis" and ancestor("joint") is not None:
            joint = ancestor("joint").attrs.get("name", "")
            xyz = _vec(tag.attrs.get("xyz", ""))
            if joint in model.joints and xyz:
                model.joints[joint]["axis"] = xyz
                model.joints[joint]["axis_ev"] = Evidence(
                    relpath, tag.attr_pos.get("xyz", (tag.line, None))[0],
                    tag.attr_pos.get("xyz", (tag.line, None))[1],
                    line_text(text, tag.line), "joint " + joint + " axis")
        elif tag.name == "gazebo":
            ref = tag.attrs.get("reference", "")
            if ref:
                parent_name = ref
                model.links.setdefault(ref, {"name": ref, "mass": None, "inertia": None,
                                             "mass_ev": None, "inertia_ev": None,
                                             "radius": None, "radius_ev": None})
        elif tag.name in ("mu1", "mu2"):
            ref = ancestor("gazebo").attrs.get("reference", "") if ancestor("gazebo") else ""
            value = parse_number(tag.attrs.get("value", ""))
            line, col = tag.attr_pos.get("value", (tag.line, None))
            if value is not None and ref:
                ev = Evidence(relpath, line, col, line_text(text, line),
                              "gazebo " + ref + " " + tag.name)
                facts.append(Fact("friction_mu", tag.name, value, tag.attrs.get("value", ""),
                                  "", ref, "urdf", ev,
                                  confidence="HIGH", note="wheel surface friction"))

        if not tag.self_closing:
            open_stack.append(tag)

    # <update_rate> carries element text rather than an attribute.
    for match in re.finditer(r"<update_rate>\s*([-+]?[\d.]+(?:[eE][-+]?\d+)?)\s*</update_rate>", text):
        line, col = pos_to_linecol(line_starts(text), match.start(1))
        ev = Evidence(relpath, line, col, line_text(text, line), "gazebo update_rate")
        model.gazebo_rates.append({"value": float(match.group(1)), "evidence": ev})
        facts.append(Fact("sim_update_rate", "update_rate", float(match.group(1)),
                          match.group(1), "Hz", "gazebo", "urdf", ev))

    for match in re.finditer(r"<real_time_update_rate>\s*([-+]?[\d.]+)\s*</real_time_update_rate>", text):
        line, col = pos_to_linecol(line_starts(text), match.start(1))
        ev = Evidence(relpath, line, col, line_text(text, line), "gazebo real_time_update_rate")
        model.gazebo_rates.append({"value": float(match.group(1)), "evidence": ev})
        facts.append(Fact("sim_update_rate", "real_time_update_rate", float(match.group(1)),
                          match.group(1), "Hz", "gazebo", "urdf", ev))

    for match in re.finditer(r"<plugin\b[^>]*name\s*=\s*[\"']([^\"']+)[\"']", text):
        model.plugins.append(match.group(1))

    return facts, model


# --------------------------------------------------------------------------- #
# ROS 2 YAML
# --------------------------------------------------------------------------- #

_YAML_KEY_RE = re.compile(r"^(\s*)([\"']?)([A-Za-z0-9_.\-/]+)\2\s*:(.*)$")
_FLOW_SEQ_RE = re.compile(r"^\[(.*)\]$")


@dataclass
class YamlEntry:
    path: tuple[str, ...]
    key: str
    literal: str
    line: int
    col: int | None
    raw_value: str
    index: int | None = None
    quoted: bool = False


def _strip_comment(line: str) -> str:
    quote = ""
    for idx, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = ""
        elif ch in "\"'":
            quote = ch
        elif ch == "#":
            return line[:idx]
    return line


def _split_flow(body: str) -> list[tuple[str, int]]:
    items: list[tuple[str, int]] = []
    depth = 0
    quote = ""
    start = 0
    for idx, ch in enumerate(body):
        if quote:
            if ch == quote:
                quote = ""
            continue
        if ch in "\"'":
            quote = ch
        elif ch in "[{":
            depth += 1
        elif ch in "]}":
            depth -= 1
        elif ch == "," and depth == 0:
            items.append((body[start:idx], start))
            start = idx + 1
    items.append((body[start:], start))
    return [(item, off) for item, off in items if item.strip()]


def scan_yaml(text: str, relpath: str) -> list[YamlEntry]:
    entries: list[YamlEntry] = []
    stack: list[tuple[int, str]] = []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = _strip_comment(raw).rstrip()
        if not line.strip():
            continue
        stripped = line.strip()
        if stripped in ("---", "...") or stripped.startswith("%YAML"):
            stack.clear()
            continue
        indent = len(line) - len(line.lstrip(" \t"))
        if stripped.startswith("- ") or stripped == "-":
            token = stripped[1:].strip()
            if stack and token:
                literal, off = literal_span(token)
                entries.append(YamlEntry(
                    path=tuple(k for _, k in stack[:-1]), key=stack[-1][1],
                    literal=literal, line=lineno, col=indent + 2 + off,
                    raw_value=token, index=None,
                    quoted=token.strip()[:1] in ("'", "\""),
                ))
            continue
        match = _YAML_KEY_RE.match(line)
        if not match:
            continue
        key = match.group(3)
        rest = match.group(4)
        while stack and stack[-1][0] >= indent:
            stack.pop()
        value = rest.strip()
        if not value:
            stack.append((indent, key))
            continue
        value_col = len(line) - len(rest) + (len(rest) - len(rest.lstrip())) + 1
        seq = _FLOW_SEQ_RE.match(value)
        if seq:
            inner = seq.group(1)
            for item, off in _split_flow(inner):
                literal, ioff = literal_span(item)
                entries.append(YamlEntry(
                    path=tuple(k for _, k in stack), key=key,
                    literal=literal, line=lineno, col=value_col + 1 + off + ioff,
                    raw_value=item.strip(), index=len([e for e in entries if e.line == lineno and e.key == key]),
                    quoted=item.strip()[:1] in ("'", "\""),
                ))
            continue
        literal, off = literal_span(value)
        entries.append(YamlEntry(
            path=tuple(k for _, k in stack), key=key, literal=literal,
            line=lineno, col=value_col + off, raw_value=value,
            quoted=value[:1] in ("'", "\""),
        ))
    return entries


def yaml_entries_to_facts(entries: Iterable[YamlEntry], relpath: str, text: str) -> list[Fact]:
    facts: list[Fact] = []
    for entry in entries:
        hit = registry.resolve(entry.key)
        if not hit:
            continue
        canonical, factor, tier = hit
        scope = ".".join(entry.path)
        if registry.is_scoped(canonical):
            scope = entry.path[-1] if entry.path else ""
        value = parse_number(entry.literal)
        unresolved = value is None and (is_expression(entry.literal) or entry.literal not in ("",))
        ev = Evidence(relpath, entry.line, entry.col, line_text(text, entry.line),
                      "yaml " + ".".join(entry.path + (entry.key,))
                      + (("[" + str(entry.index) + "]") if entry.index is not None else ""))
        facts.append(Fact(
            canonical=canonical, raw_name=entry.key,
            value=(value * factor) if value is not None else None,
            literal=entry.literal, unit=registry.unit_of(canonical),
            scope=scope, source="yaml", evidence=ev, factor=factor,
            unresolved=bool(unresolved and value is None),
            confidence="HIGH" if tier == 0 else "MEDIUM",
            note="" if value is not None else "非数值，无法参与数值比对",
        ))
    return facts


# --------------------------------------------------------------------------- #
# launch files
# --------------------------------------------------------------------------- #

_LAUNCH_ARG_PY = re.compile(
    r"DeclareLaunchArgument\(\s*[\"']([\w.]+)[\"']"
    r"(?:\s*,\s*default_value\s*=\s*(?:[\"']([^\"']*)[\"']|([-+\d.eE]+)))?", re.S)
_LAUNCH_DICT_PY = re.compile(
    r"[\"']([\w.]+)[\"']\s*:\s*(?:ParameterValue\(\s*(?:value\s*=\s*)?)?"
    r"(?:([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)|[\"']([^\"']*)[\"']|(\$\([^)]*\)))")
_LAUNCH_NODE_PY = re.compile(
    r"Node\(\s*(?:[^)]*?)package\s*=\s*[\"']([\w.]+)[\"'].*?executable\s*=\s*[\"']([\w.]+)[\"']", re.S)
_LAUNCH_ARG_XML = re.compile(r"<arg\s+name=[\"']([\w.]+)[\"'](?:\s+default=[\"']([^\"']*)[\"'])?")
_LAUNCH_PARAM_XML = re.compile(
    r"<param\s+name=[\"']([\w.]+)[\"'](?:\s+type=[\"'](\w+)[\"'])?(?:\s+value=[\"']([^\"']*)[\"'])?")
_LAUNCH_NODE_XML = re.compile(r"<node\s+pkg=[\"']([\w.]+)[\"']\s+(?:exec|type)=[\"']([\w.]+)[\"']")


def scan_launch(text: str, relpath: str) -> tuple[list[Fact], dict]:
    starts = line_starts(text)
    facts: list[Fact] = []
    info: dict[str, Any] = {"args": {}, "params": [], "nodes": [], "remaps": [], "kind": ""}

    def add_fact(raw_name: str, token: str, pos: int, kind: str, scope: str = "") -> None:
        line, col = pos_to_linecol(starts, pos)
        hit = registry.resolve(raw_name)
        ev = Evidence(relpath, line, col, line_text(text, line), kind + " " + raw_name)
        if not hit:
            info["params"].append({"name": raw_name, "value": token, "evidence": ev, "canonical": None})
            return
        canonical, factor, tier = hit
        value = parse_number(token)
        facts.append(Fact(
            canonical=canonical, raw_name=raw_name,
            value=(value * factor) if value is not None else None,
            literal=token, unit=registry.unit_of(canonical), scope=scope,
            source="launch", evidence=ev, factor=factor,
            unresolved=value is None, confidence="HIGH" if tier == 0 else "MEDIUM",
            note="" if value is not None else "非数值，无法参与数值比对",
        ))
        info["params"].append({"name": raw_name, "value": token, "evidence": ev,
                               "canonical": canonical})

    if relpath.endswith(".xml"):
        info["kind"] = "launch.xml"
        for match in _LAUNCH_ARG_XML.finditer(text):
            val = match.group(2)
            info["args"][match.group(1)] = val if val is not None else ""
            if val is not None:
                add_fact(match.group(1), val, match.start(2), "launch arg")
        for match in _LAUNCH_PARAM_XML.finditer(text):
            if match.group(3) is None:
                continue
            add_fact(match.group(1), match.group(3), match.start(3), "launch param")
        for match in _LAUNCH_NODE_XML.finditer(text):
            info["nodes"].append({"package": match.group(1), "executable": match.group(2)})
        return facts, info

    info["kind"] = "launch.py"
    for match in _LAUNCH_ARG_PY.finditer(text):
        name = match.group(1)
        default = match.group(2) if match.group(2) is not None else match.group(3)
        info["args"][name] = default if default is not None else ""
        if default is not None:
            pos = match.start(2) if match.group(2) is not None else match.start(3)
            add_fact(name, default, pos, "launch arg")
    for match in _LAUNCH_DICT_PY.finditer(text):
        name = match.group(1)
        for group in (2, 3, 4):
            if match.group(group) is not None:
                token = match.group(group)
                add_fact(name, token, match.start(group), "launch param")
                break
    for match in _LAUNCH_NODE_PY.finditer(text):
        info["nodes"].append({"package": match.group(1), "executable": match.group(2)})
    return facts, info


# --------------------------------------------------------------------------- #
# C++ / Python sources
# --------------------------------------------------------------------------- #

_SOURCE_RULES = [
    ("cpp_const", re.compile(
        r"\b(?:constexpr|static\s+constexpr|static\s+const|const)\s+"
        r"(?:double|float|int|unsigned|std::size_t|size_t)\s+([A-Za-z_]\w*)\s*=\s*"
        r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"), 1, 2),
    ("cpp_auto", re.compile(
        r"\b(?:constexpr|static\s+constexpr|const)\s+auto\s+([A-Za-z_]\w*)\s*=\s*"
        r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"), 1, 2),
    ("define", re.compile(
        r"^[ \t]*#[ \t]*define[ \t]+([A-Za-z_]\w*)[ \t]+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)[ \t]*$",
        re.M), 1, 2),
    ("py_assign", re.compile(
        r"^[ \t]*(?:self\.)?([A-Za-z_]\w*)[ \t]*(?::[ \t]*(?:float|int|double)[ \t]*)?="
        r"[ \t]*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)[ \t]*(?:#.*)?$", re.M), 1, 2),
]

_SOURCE_EXTS = {".py", ".cpp", ".cc", ".cxx", ".hpp", ".hh", ".h", ".c"}
_SKIP_DIRS = {"build", "install", "log", "logs", ".git", "__pycache__", ".venv",
              "venv", "node_modules", ".pytest_cache", "site-packages"}


def scan_source(text: str, relpath: str) -> list[Fact]:
    starts = line_starts(text)
    facts: list[Fact] = []
    seen: set[tuple[str, int]] = set()
    lines = text.splitlines()
    for kind, rule, name_group, value_group in _SOURCE_RULES:
        for match in rule.finditer(text):
            start_pos = match.start()
            line, col = pos_to_linecol(starts, start_pos)
            raw_line = lines[line - 1] if line - 1 < len(lines) else ""
            code_part = raw_line.split("#", 1)[0] if relpath.endswith(".py") else raw_line
            if "//" in code_part and code_part.index("//") < (match.start() - starts[line - 1]):
                continue
            if code_part.lstrip().startswith("#") and kind != "define":
                continue
            name = match.group(name_group)
            token = match.group(value_group)
            hit = registry.resolve(name)
            if not hit:
                continue
            canonical, factor, tier = hit
            value = parse_number(token)
            if value is None:
                continue
            key = (relpath, line)
            if key in seen:
                continue
            seen.add(key)
            vpos = match.start(value_group)
            vline, vcol = pos_to_linecol(starts, vpos)
            ev = Evidence(relpath, vline, vcol, raw_line.strip()[:160],
                          kind + " " + name)
            facts.append(Fact(
                canonical=canonical, raw_name=name, value=value * factor,
                literal=token, unit=registry.unit_of(canonical), scope="",
                source="source", evidence=ev, factor=factor,
                confidence="HIGH" if tier == 0 else "MEDIUM",
                note="hard-coded constant in source code",
            ))
    return facts


# --------------------------------------------------------------------------- #
# project walking
# --------------------------------------------------------------------------- #

@dataclass
class StaticInputs:
    facts: list[Fact] = field(default_factory=list)
    yaml_entries: list[tuple[str, list[YamlEntry], str]] = field(default_factory=list)
    robots: list[RobotModel] = field(default_factory=list)
    launch_info: list[tuple[str, dict]] = field(default_factory=list)
    files: list[str] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)
    labels: dict[str, str] = field(default_factory=dict)


def classify(path: Path) -> str | None:
    suffix = path.suffix.lower()
    name = path.name.lower()
    if suffix in (".urdf", ".xacro"):
        return "urdf"
    if suffix in (".launch",) or (suffix == ".py" and "launch" in name):
        return "launch"
    if suffix == ".xml" and "launch" in name:
        return "launch"
    if suffix in (".yaml", ".yml"):
        return "yaml"
    if suffix in _SOURCE_EXTS and "launch" not in name:
        return "source"
    return None


def collect_static(root: str | Path, max_files: int = 400) -> StaticInputs:
    root_path = Path(root)
    inputs = StaticInputs()
    if not root_path.exists():
        return inputs
    for dirpath, dirnames, filenames in os.walk(root_path):
        dirnames[:] = [d for d in dirnames if d not in _SKIP_DIRS]
        for filename in sorted(filenames):
            full = Path(dirpath) / filename
            rel = full.relative_to(root_path).as_posix()
            kind = classify(full)
            if kind is None:
                continue
            try:
                text = full.read_text(encoding="utf-8", errors="replace")
            except OSError:
                inputs.skipped.append(rel + " (unreadable)")
                continue
            if len(text) > 2_000_000:
                inputs.skipped.append(rel + " (>2MB, skipped)")
                continue
            inputs.labels[rel] = kind
            inputs.files.append(rel)
            try:
                if kind == "urdf":
                    facts, model = extract_urdf(text, rel)
                    inputs.facts.extend(facts)
                    inputs.robots.append(model)
                elif kind == "yaml":
                    entries = scan_yaml(text, rel)
                    inputs.yaml_entries.append((rel, entries, text))
                    inputs.facts.extend(yaml_entries_to_facts(entries, rel, text))
                elif kind == "launch":
                    facts, info = scan_launch(text, rel)
                    inputs.facts.extend(facts)
                    inputs.launch_info.append((rel, info))
                elif kind == "source":
                    inputs.facts.extend(scan_source(text, rel))
            except Exception as exc:  # never abort the whole audit on one file
                inputs.skipped.append(f"{rel} (parse error: {exc.__class__.__name__}: {exc})")
            if len(inputs.files) >= max_files:
                inputs.skipped.append("file limit reached (%d)" % max_files)
                return inputs
    return inputs
