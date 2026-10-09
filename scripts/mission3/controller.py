"""기존 실행기·ROS·시험에 대한 호환 진입점.

실제 판단 관리자는 supervisor.py, 단계 제어는 *_control.py에 있다.
Command 형식과 Mission3 생성/update API를 유지해 실행 경로를 함께 전환한다.
"""
from .command import Command
from .supervisor import MissionSupervisor as Mission3

__all__ = ["Command", "Mission3"]
