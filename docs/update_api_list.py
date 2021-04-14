import pathlib
import sys

git_root = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(git_root / "tests"))

import conftest


def create_section(name, objects, prefix=""):
    object_names = [f"{prefix}{obj.__module__}.{obj.__name__}" for obj in objects]
    separator = "\n    "
    return f"""

{name}
{"^" * len(name)}

.. autosummary::
   :toctree: generated
{separator}{separator.join(object_names)}
"""


class_section = create_section("Classes", conftest.find_all_device_classes())
callable_section = create_section("Functions", conftest.find_all_callables())

print(f"""
Full API
########

{class_section}

{callable_section}
""")
