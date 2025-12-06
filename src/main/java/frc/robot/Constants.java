package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public final class Constants {

    public static final class PortConstants {

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class TrajectoryConstants {

        public static final HolonomicDriveController kController = new HolonomicDriveController(
            new PIDController(1, 0, 0), new PIDController(1, 0, 0),
            new ProfiledPIDController(1, 0, 0,
              new TrapezoidProfile.Constraints(1, 0.5)));

        public static final Transform2d kPoseTransform = new Transform2d(
                new Translation2d(1.0, 1.0),
                new Rotation2d(0.0));
        
        public static final Pose2d kCenterField = new Pose2d(8.8, 4.0, new Rotation2d(0.0));
        
        public static final LinearVelocity kMaxVelocity = MetersPerSecond.of(3.0);
        public static final LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(3.0);
    }

    public static final class ArmConstants {
        public static final int kArmMotorId = 20;
        public static final String kArmCANBus = "SwerveCAN";

        public static final double kZeroOffsetRotations = -0.25;
        public static final double kMinPositionRotations = -0.3;
        public static final double kMaxPositionRotations = 0.3;

        public static final double kMMCruiseVelocity = 1.0; // rotations per second
        public static final double kMMAcceleration = 0.5; // rotations per second squared

        public static final double kGearRatio = 18.0; // Motor rotations to arm rotations

        public static final double kS = 0.046499; // Static gain
        public static final double kG = 0.29638; // Gravity gain
        public static final double kV = 2.009; // Velocity gain
        public static final double kA = 0.18252; // Acceleration gain

        public static final double kP = 0.2; // Proportional gain
        public static final double kI = 0.0; // Integral gain
        public static final double kD = 0.0; // Derivative gain
    }
}