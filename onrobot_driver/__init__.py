"""OnRobot Driver Python package."""
try:
    from .TwoFG import TwoFG
except ImportError:
    try:
        from TwoFG import TwoFG
    except ImportError:
        TwoFG = None

try:
    from .ThreeFG import ThreeFG
except ImportError:
    try:
        from ThreeFG import ThreeFG
    except ImportError:
        ThreeFG = None

try:
    from .VGC10 import VGC10
except ImportError:
    try:
        from VGC10 import VGC10
    except ImportError:
        VGC10 = None

__all__ = ["TwoFG", "ThreeFG", "VGC10"]
