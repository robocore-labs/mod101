"""Versioned package-defined controls; source xacro keeps saved build values.

Each mod101_tool_*/config/configurator.yaml defines optional enum/boolean
arguments. No package-specific frontend code is needed. Values are validated
before forwarding to xacro or writing a configuration file.
"""
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape
import yaml

RESERVED = {'tool', 'arm_dof', 'wrist_camera', 'shoulder_ext_length', 'elbow_ext_length',
            'shoulder_mount', 'elbow_mount', 'use_sim', 'hardware', 'prefix'}


def load_schema(package):
    path = package / 'config/configurator.yaml'
    if path.exists():
        schema = yaml.safe_load(path.read_text())
        if not isinstance(schema, dict) or schema.get('schema_version') != 1:
            raise ValueError(f'{path}: unsupported configurator schema')
    else:
        controllers = package / 'config/controllers.yaml'
        params = yaml.safe_load(controllers.read_text()) if controllers.exists() else {}
        schema = {'schema_version': 1, 'label': package.name.removeprefix('mod101_tool_'),
                  'actuated': bool(params), 'options': {}}
    schema = {**schema, 'options': schema.get('options') or {}}
    schema.setdefault('servo_count', 1 if schema.get('actuated') else 0)
    if not isinstance(schema['options'], dict):
        raise ValueError(f'{path}: options must be a mapping')
    for name, spec in schema['options'].items():
        if name in RESERVED or not re.fullmatch(r'[a-z][a-z0-9_]*', name):
            raise ValueError(f'{path}: invalid/reserved argument {name}')
        normalize(spec, spec['default'])
    return schema


def discover(src):
    return {p.name.removeprefix('mod101_tool_'): load_schema(p)
            for p in sorted(src.glob('mod101_tool_*')) if (p / 'package.xml').exists()}


def specs(src):
    out = {}
    for schema in discover(src).values():
        for name, spec in schema['options'].items():
            if name in out:
                raise ValueError(f'Duplicate tool option {name}; namespace arguments by tool')
            out[name] = spec
    return out


def normalize(spec, value):
    if spec['type'] == 'boolean':
        if isinstance(value, bool):
            return value
        if isinstance(value, str) and value.lower() in ('true', 'false'):
            return value.lower() == 'true'
        raise ValueError('Boolean option must be true or false')
    if spec['type'] == 'enum':
        choices = [c['value'] for c in spec['choices']]
        if value not in choices:
            raise ValueError(f'Option must be one of {choices}')
        return value
    raise ValueError(f'Unsupported tool option type {spec["type"]}')


def validate(values, definitions):
    if not isinstance(values, dict):
        raise ValueError('tool_options must be an object')
    unknown = values.keys() - definitions.keys()
    if unknown:
        raise ValueError(f'Unknown tool options: {sorted(unknown)}')
    return {name: normalize(definitions[name], value) for name, value in values.items()}


def read_values(text, definitions):
    defaults = {e.get('name'): e.get('default')
                for e in ET.fromstring(text).findall('{http://www.ros.org/wiki/xacro}arg')}
    return {name: normalize(spec, defaults.get(name, spec['default']))
            for name, spec in definitions.items()}


def write_values(text, values, definitions):
    for name, value in validate(values, definitions).items():
        raw = str(value).lower() if isinstance(value, bool) else str(value)
        raw = escape(raw, {'"': '&quot;'})
        pattern = rf'(<xacro:arg\s+name="{re.escape(name)}"\s+default=")[^"]*("\s*/>)'
        text, count = re.subn(pattern, lambda m: m[1] + raw + m[2], text)
        if count == 0:
            text = text.replace('</robot>', f'<xacro:arg name="{name}" default="{raw}"/>\n</robot>')
        elif count != 1:
            raise ValueError(f'Expected one configuration argument {name}')
    return text


def tool_option_arguments():
    """Discover option arguments from installed ROS tool package manifests."""
    from ament_index_python.packages import get_packages_with_prefixes, get_package_share_directory
    names = []
    for package in sorted(get_packages_with_prefixes()):
        if package.startswith('mod101_tool_'):
            schema = load_schema(Path(get_package_share_directory(package)))
            for name in schema['options']:
                if name in names:
                    raise ValueError(f'Duplicate tool argument {name}')
                names.append(name)
    return tuple(names)
