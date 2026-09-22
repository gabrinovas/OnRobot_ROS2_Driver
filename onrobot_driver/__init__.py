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

__all__ = ["TwoFG", "ThreeFG"]
