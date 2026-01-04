package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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

        public static final double kMaxSpeed = 2; // Meters per second

        public static final double RotationalkP = 5;
        public static final double RotationalkI = 0;
        public static final double RotationalkD = 0.1;

        public static final double kXyOdomStdDevEnabled = 0.2;
        public static final double kThetaOdomStdDevEnabled = 0.15;

        public static final double kXyOdomStdDevDisabled = 1.0;
        public static final double kThetaOdomStdDevDisabled = 1.0;

        public static final Matrix<N3, N1> kOdomStdDevsEnabled = VecBuilder.fill(
            kXyOdomStdDevEnabled,
            kXyOdomStdDevEnabled,
            kThetaOdomStdDevEnabled);

        public static final Matrix<N3, N1> kOdomStdDevsDisabled = VecBuilder.fill(
            kXyOdomStdDevDisabled,
            kXyOdomStdDevDisabled,
            kThetaOdomStdDevDisabled);
        

        public static final double kMaxAccelerationMetersPerSecondSquared = Units.inchesToMeters(500);;
        //public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 2;
    }

    

    public static final class LimelightConstants {
        public static final String kLimelightOne = "limelight";
        public static final String kLimelightTwo = "limelight-two";

        public static final double kXyStdDev = 0.2;
        public static final double kThetaStdDev = 3;

        public static final Matrix<N3, N1> kStdDevs = VecBuilder.fill(
            kXyStdDev,
            kXyStdDev,
            kThetaStdDev);

        public static final double kMaxAmbiguity = 0.9;
        public static final double kMinAreaOdom = 0.0;
        public static final double kMinAreaGyro = 0.2;

        public static final double kMaxPoseDistance = 10;
        public static final double kMaxRotationDifference = 1000000.0; // degrees

        public static final double kMaxDistance = 3000.0;
        public static final int kMinTags = 1;
        public static final int kPipelineIndex = 0;

        public static final double kMaxTranslationalVelocity = 9.0; // meters per second
        public static final double kMaxRotationalVelocity = 360; // degrees per second


    }
}