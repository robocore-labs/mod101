"""Use the same source schema API as ROS, without requiring a workspace build."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/mod101_description'))
from mod101_description.tool_config import (discover, load_schema, normalize,
                                           read_values, specs, validate, write_values)
