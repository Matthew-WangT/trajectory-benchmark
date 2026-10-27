"""Evaluation stages.

Each analyzer declares what it needs and is skipped (with a reason) when the CSV or the robot profile cannot
provide it: a robot without a URDF gets the joint and TCP stages, the Marvin replay additionally the model based
ones and the health of its replay chain.
"""

from .base import Analyzer, AnalyzerResult, Context
from .config_margin import ConfigMarginAnalyzer
from .ee_tracking import EeTrackingAnalyzer
from .health import HealthAnalyzer
from .joint_error import JointErrorAnalyzer
from .joint_tracking import JointTrackingAnalyzer
from .sanity import SanityAnalyzer
from .tcp_tracking import TcpTrackingAnalyzer

# 执行顺序即依赖顺序: joint 的延迟结果供 ee 使用, ee 的雅可比供 margin 使用。
DEFAULT_ANALYZERS: list[type[Analyzer]] = [
    SanityAnalyzer,
    JointErrorAnalyzer,
    JointTrackingAnalyzer,
    TcpTrackingAnalyzer,
    EeTrackingAnalyzer,
    ConfigMarginAnalyzer,
    HealthAnalyzer,
]

__all__ = ["Analyzer", "AnalyzerResult", "Context", "DEFAULT_ANALYZERS"]
