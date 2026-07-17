import threading
import time
from dataclasses import dataclass
from typing import Any

from ntcore import NetworkTableInstance, NetworkTable, StructPublisher

from wpimath.geometry import Rotation2d, Pose3d

from wpiutil.wpistruct import make_wpistruct

from wpimath.units import (
    degrees,
    inches,
    amperes,
)
from wpimath.geometry import Rotation2d

from phoenix6.units import rotations_per_second, rotation
from phoenix6.status_signal import StatusSignal


@make_wpistruct
@dataclass
class CameraData:
    hasTarget: bool
    translationStdev: float
    rotationStdev: float
    estimatedRobotPose: Pose3d


@make_wpistruct
@dataclass
class ShooterLeftCameraData(CameraData):
    pass


@make_wpistruct
@dataclass
class ShooterRightCameraData(CameraData):
    pass


@make_wpistruct
@dataclass
class BraveData:
    ShooterLeftCameraData: ShooterLeftCameraData
    ShooterRightCameraData: ShooterRightCameraData


class BraveLogger:
    _nettable: NetworkTable

    _dataPub: StructPublisher

    _data: BraveData

    _statusSignals: dict[str, list[StatusSignal]] = {"": []}
    _statusSignalIndex: int = 0
    _statusSignalBatchSize: int = 1

    def __init__(
        self,
    ) -> None:
        BraveLogger._nettable = NetworkTableInstance.getDefault().getTable(
            "000BraveLogger"
        )
        BraveLogger._dataPub = self._nettable.getStructTopic(
            "Data", BraveData
        ).publish()
        BraveLogger._data = BraveData(
            ShooterLeftCameraData(False, 0, 0, Pose3d()),
            ShooterRightCameraData(False, 0, 0, Pose3d()),
        )

        def _log_loop():
            while True:
                time.sleep(0.10)
                try:
                    BraveLogger.log()
                except Exception:
                    pass

        def _refresh_loop():
            while True:
                time.sleep(0.04)
                try:
                    BraveLogger.refreshStatusSignals()
                except Exception:
                    pass

        threading.Thread(target=_log_loop, daemon=True, name="BraveLogger-log").start()
        threading.Thread(
            target=_refresh_loop, daemon=True, name="BraveLogger-refresh"
        ).start()

    @staticmethod
    def log() -> None:
        BraveLogger._dataPub.set(BraveLogger._data)

    @staticmethod
    def refreshStatusSignals() -> None:
        """
        Refreshes the status signals for the BraveLogger subsystem.
        This method is called periodically to update the status signals.
        """

        for _bus, signals in BraveLogger._statusSignals.items():
            if signals:
                StatusSignal.refresh_all(signals)  # type: ignore

    @staticmethod
    def pushSubsystemData(data: Any) -> None:
        """
        Pushes the given data to the network table for the subsystem.

        :param data: The data to push to the network table.
        :type data: Any wpistruct type (e.g., TurretData, ClimberData, etc.)
        """
        if isinstance(data, ShooterLeftCameraData):
            BraveLogger._data.ShooterLeftCameraData = data
        elif isinstance(data, ShooterRightCameraData):
            BraveLogger._data.ShooterRightCameraData = data

    @staticmethod
    def registerStatusSignal(
        signal: StatusSignal | list[StatusSignal], bus: str = ""
    ) -> None:
        """
        Registers a status signal to be refreshed periodically.

        :param signal: The status signal to register.
        :type signal: StatusSignal
        """
        if bus not in BraveLogger._statusSignals.keys():
            BraveLogger._statusSignals[bus] = []
        if isinstance(signal, list):
            BraveLogger._statusSignals[bus].extend(signal)
        else:
            BraveLogger._statusSignals[bus].append(signal)
