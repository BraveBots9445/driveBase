from math import pi
from commands2 import Command

from ntcore import NetworkTableInstance
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import (
    Transform2d,
    Pose2d,
    Rotation2d,
    Transform3d,
    Pose3d,
    Rotation3d,
)
from wpimath.controller import PIDController

from phoenix6 import swerve

from subsystems.drivetrain import CommandSwerveDrivetrain
from subsystems.vision import Vision


class VisionAlignTag(Command):
    # everything happens in meters and radians
    translation_kP: float = 1.0
    translation_kI: float = 0
    translation_kD: float = 0.1

    rotation_kP: float = 0.5
    rotation_kI: float = 0
    rotation_kD: float = 0.1

    # how far from the camera should the tag be
    desired_displacement: Transform2d

    vision_enabled_at_start: bool

    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        vision: Vision,
        offset: Transform2d,
        tag_id: int = 17,
    ):
        super().__init__()
        self.drivetrain = drivetrain
        self.vision = vision
        self._drive = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )  # Use open-loop control for drive motors

        self.tX_pid = PIDController(
            self.translation_kP,
            self.translation_kI,
            self.translation_kD,
        )

        self.tY_pid = PIDController(
            self.translation_kP,
            self.translation_kI,
            self.translation_kD,
        )

        self.tX_pid.setTolerance(0.02)
        self.tY_pid.setTolerance(0.02)

        self.r_pid = PIDController(
            self.rotation_kP,
            self.rotation_kI,
            self.rotation_kD,
        )

        self.r_pid.enableContinuousInput(-pi, pi)

        self.r_pid.setTolerance(pi / 36)

        self.desired_displacement = offset
        self.tag = tag_id

        self.addRequirements(drivetrain)
        self.addRequirements(vision)

        self.setName(f"Align to tag {tag_id}")

        self.nettable = NetworkTableInstance.getDefault().getTable("0000VisionAlignTag")
        self.ideal_pose_pub = self.nettable.getStructTopic(
            "ideal_pose", Pose2d
        ).publish()
        self.transform_pub = self.nettable.getStructTopic(
            "target", Transform3d
        ).publish()
        self.has_tag_pub = self.nettable.getBooleanTopic("has_tag").publish()

    def initialize(self) -> None:
        self.tX_pid.reset()
        self.tY_pid.reset()
        self.r_pid.reset()

        self.vision_enabled_at_start = self.vision.enabled
        self.vision.disable_measurements()

    def execute(self) -> None:
        a = self.vision.get_displacement_to_tag(self.tag)
        if a:
            target = a
            self.transform_pub.set(target)
        else:
            target = None
        # the below section is for testing using the robot pose instead of vision data
        """
        p = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField).getTagPose(
            self.tag
        )
        if p:
            target = Transform3d(
                Pose3d(self.drivetrain.get_state().pose),
                p,
            )
            target = Transform3d(
                target.translation(),
                target.rotation().rotateBy(Rotation3d.fromDegrees(0, 0, 180)),
            )
        else:
            target = None
        """

        if target:
            x_speed = self.tX_pid.calculate(target.x, self.desired_displacement.x)
            y_speed = self.tY_pid.calculate(target.y, self.desired_displacement.y)
            rot_speed = self.r_pid.calculate(
                target.rotation().toRotation2d().radians(),
                self.desired_displacement.rotation().radians(),
            )
            self.has_tag_pub.set(True)
            self.ideal_pose_pub.set(
                self.drivetrain.get_state().pose.transformBy(
                    Transform2d(
                        target.X(), target.Y(), target.rotation().toRotation2d()
                    )
                    + self.desired_displacement
                )
            )
        else:
            x_speed = 0
            y_speed = 0
            rot_speed = 0
            self.has_tag_pub.set(False)

        self.nettable.putNumber("Output/x_speed", x_speed)
        self.nettable.putNumber("Output/y_speed", y_speed)
        self.nettable.putNumber("Output/rot_speed", rot_speed)
        self.nettable.putNumber("Error/x", self.tX_pid.getPositionError())
        self.nettable.putNumber("Error/y", self.tY_pid.getPositionError())
        self.nettable.putNumber("Error/rot", self.r_pid.getPositionError())

        req = (
            self._drive.with_velocity_x(-x_speed)
            .with_velocity_y(-y_speed)
            .with_rotational_rate(-rot_speed)
        )
        self.drivetrain.set_control(req)

    def isFinished(self) -> bool:
        return (
            self.tX_pid.atSetpoint()
            and self.tY_pid.atSetpoint()
            and self.r_pid.atSetpoint()
        )

    def end(self, interrupted: bool) -> None:
        self.drivetrain.set_control(
            self._drive.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        )
        if self.vision_enabled_at_start:
            self.vision.enable_measurements()
