from math import pi
from typing import Callable

from commands2 import Command

from wpilib import Timer

from wpimath.units import meters_per_second, degrees_per_second
from wpimath.controller import PIDController
from wpimath.geometry import Transform2d, Rotation2d

from phoenix6 import swerve


from subsystems.drivetrain import CommandSwerveDrivetrain
from subsystems.vision import Vision


class AlignToTargets(Command):
    # translational PID constants
    t_kP: float = 0.5
    t_kI: float = 0.0
    t_kD: float = 0.0

    # rotational PID constants
    r_kP: float = 0.02
    r_kI: float = 0.0
    r_kD: float = 0

    max_velocity: meters_per_second = 1.0
    max_angular_rate: degrees_per_second = pi

    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        vision: Vision,
        target_ids: list[int],
        desired_offset: Transform2d,
        get_drive_x: Callable[[], float],
        get_drive_y: Callable[[], float],
        get_drive_r: Callable[[], float],
        tolerance: Transform2d = Transform2d(0.1, 0.1, Rotation2d(5)),
    ):
        super().__init__()
        self.drivetrain = drivetrain
        self.vision = vision
        self.target_ids = target_ids
        self.desired_offset = desired_offset
        self.tolerance = tolerance
        self.addRequirements(drivetrain)

        self.drive_x = get_drive_x
        self.drive_y = get_drive_y
        self.drive_r = get_drive_r

        self.xPID = PIDController(self.t_kP, self.t_kI, self.t_kD)
        self.yPID = PIDController(self.t_kP, self.t_kI, self.t_kD)
        self.rPID = PIDController(self.r_kP, self.r_kI, self.r_kD)

        self.xPID.setTolerance(self.tolerance.X())
        self.yPID.setTolerance(self.tolerance.Y())
        self.rPID.setTolerance(self.tolerance.rotation().degrees())
        self.rPID.enableContinuousInput(-180, 180)

        self._drive = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
            .with_deadband(0)
        )

        self.most_recent_offset: Transform2d | None = None
        self.most_recent_timer = Timer()

    def initialize(self):
        self.most_recent_offset = None
        self.most_recent_timer.reset()

    def execute(self):
        if self.most_recent_timer.isRunning():
            print(self.most_recent_timer.get())
        # some amount of proving should be done here to make sure that
        # this is set if we get a reading or are within time
        offset: Transform2d = self.desired_offset
        for target_id in self.target_ids:
            offset3d = self.vision.get_offset_to_target(target_id)
            if offset3d is not None:
                offset = Transform2d(
                    offset3d.X(), offset3d.Y(), offset3d.rotation().toRotation2d()
                )
                self.most_recent_timer.stop()
                break
        else:
            print("NO TARGETS")
            if (
                self.most_recent_timer.hasElapsed(0.5)
                or self.most_recent_offset is None
            ):
                self.drivetrain.set_control(
                    self._drive.with_velocity_x(self.drive_x())
                    .with_velocity_y(self.drive_y())
                    .with_rotational_rate(self.drive_r())
                )
            else:
                speeds = self.drivetrain.get_state().speeds
                self.most_recent_offset += Transform2d(
                    speeds.vy * 0.02, speeds.vy * 0.02, speeds.omega * 0.02
                )
            if not self.most_recent_timer.isRunning():
                self.most_recent_timer.restart()
            return
        if offset is not None:
            self.most_recent_timer.reset()
            self.most_recent_offset = offset

        print(self.most_recent_offset)

        x_speed = -self.xPID.calculate(
            self.most_recent_offset.X(), self.desired_offset.X()
        )
        y_speed = -self.yPID.calculate(
            self.most_recent_offset.Y(), self.desired_offset.Y()
        )

        r_speed = -self.rPID.calculate(
            self.most_recent_offset.rotation().degrees(),
            self.desired_offset.rotation().degrees(),
        )

        if abs(x_speed) > self.max_velocity:
            x_speed = self.max_velocity * (x_speed / abs(x_speed))
        if self.xPID.atSetpoint():
            x_speed = 0.0
        if abs(y_speed) > self.max_velocity:
            y_speed = self.max_velocity * (y_speed / abs(y_speed))
        if self.yPID.atSetpoint():
            y_speed = 0.0
        if abs(r_speed) > self.max_angular_rate:
            r_speed = self.max_angular_rate * (r_speed / abs(r_speed))
        if self.rPID.atSetpoint():
            r_speed = 0.0

        self.drivetrain.set_control(
            self._drive.with_velocity_x(x_speed)
            .with_velocity_y(y_speed)
            .with_rotational_rate(r_speed)
        )

    def isFinished(self) -> bool:
        return (
            self.most_recent_offset is not None
            and abs(self.most_recent_offset.X() - self.desired_offset.X())
            < self.tolerance.X()
            and abs(self.most_recent_offset.Y() - self.desired_offset.Y())
            < self.tolerance.Y()
            and abs(
                self.most_recent_offset.rotation().degrees()
                - self.desired_offset.rotation().degrees()
            )
            < self.tolerance.rotation().degrees()
        )

    def end(self, interrupted: bool) -> None:
        self.drivetrain.set_control(
            self._drive.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        )
