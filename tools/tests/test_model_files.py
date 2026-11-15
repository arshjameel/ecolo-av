"""Every simulator model and world is well-formed XML (a '--' in a comment breaks SDF)."""

import glob
import os
import xml.dom.minidom

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FILES = sorted(glob.glob(os.path.join(ROOT, 'ros', 'sim', 'aav_gazebo', 'models', '*', '*.sdf'))
               + glob.glob(os.path.join(ROOT, 'ros', 'sim', 'aav_gazebo', 'worlds', '*.sdf')))


def test_there_are_models():
    assert FILES


@pytest.mark.parametrize('path', FILES, ids=lambda p: os.path.relpath(p, ROOT))
def test_each_parses_as_xml(path):
    xml.dom.minidom.parse(path)
