"""High-level local client for Hanback AIoT SerBot control over SSH."""

from .client import MotionStep, RobotConfig, SerBot, SerBotError

__version__ = "0.2.0"

__all__ = ["MotionStep", "RobotConfig", "SerBot", "SerBotError", "__version__"]
