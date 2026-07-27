from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

@pytest.fixture
def fixtures() -> Path:
    return Path(__file__).parent / "fixtures"
