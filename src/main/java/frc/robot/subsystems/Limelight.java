package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase{
    
    private CommandSwerveDrivetrain drivetrain;
    private LED candle;

    private Pose2d llPoseOne;
    private Pose2d llPoseTwo;
    private LimelightHelpers.PoseEstimate llResult;
    private LimelightHelpers.PoseEstimate llResult2;
    private int tags;
    public Limelight(CommandSwerveDrivetrain drivetrain, LED candle){
        this.drivetrain = drivetrain;
        this.candle = candle;

        llPoseOne = new Pose2d();
        llPoseTwo = new Pose2d();

        llResult = new LimelightHelpers.PoseEstimate();
        llResult2 = new LimelightHelpers.PoseEstimate();

        tags = 0;
        LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightOne, LimelightConstants.kPipelineIndex);
        LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightTwo, LimelightConstants.kPipelineIndex);

    }

    @Override
    public void periodic(){

        llResult = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.kLimelightOne);
        llResult2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.kLimelightTwo);

        tags = llResult.tagCount + llResult2.tagCount;
        

        if(llResult != null && llResult.tagCount >= LimelightConstants.kMinTags && llResult.rawFiducials != null && llResult.rawFiducials.length > 0 
           && llResult != null && llResult2.tagCount >= LimelightConstants.kMinTags && llResult2.rawFiducials != null && llResult2.rawFiducials.length > 0) {

            llPoseOne = llResult.pose;
            llPoseTwo = llResult2.pose;

            double avgX = (llPoseOne.getX() + llPoseTwo.getX()) / 2.0;
            double avgY = (llPoseOne.getY() + llPoseTwo.getY()) / 2.0;
            double avgTheta = (llPoseOne.getRotation().getRadians() + llPoseTwo.getRotation().getRadians()) / 2.0;

            Pose2d combinedPose2d = new Pose2d(
                avgX,
                avgY,
                new Rotation2d(avgTheta)
            );

            if(llResult.rawFiducials[0].ambiguity < LimelightConstants.kMaxAmbiguity
                && llResult.rawFiducials[0].distToCamera < LimelightConstants.kMaxDistance) {

                drivetrain.addVisionMeasurement(
                    combinedPose2d,
                    llResult.timestampSeconds,
                    LimelightConstants.kStdDevs
                    );

            }
        }
 
        if (hasValidTarget()){
            candle.cameraSetColor(Color.kGreen, 1);
        }
        else {
            candle.cameraClearColor();

        }

        
 
        SmartDashboard.putNumber("LL tag count", tags);
        SmartDashboard.putBoolean("LL has target", hasValidTarget());


        if(llResult != null && llResult.rawFiducials != null && llResult.rawFiducials.length == 1) {
            SmartDashboard.putNumber("LL ambiguity", llResult.rawFiducials[0].ambiguity);
            SmartDashboard.putNumber("LL Estimated Pose X", llPoseOne.getX());
            SmartDashboard.putNumber("LL Estimated Pose Y", llPoseOne.getY());
            SmartDashboard.putNumber("LL Estimated Pose Theta", llPoseOne.getRotation().getDegrees());
        }
       
    }

    public boolean hasValidTarget(){
        return (llResult != null && llResult != null && llResult.tagCount >= LimelightConstants.kMinTags && llResult.rawFiducials != null && llResult.rawFiducials.length > 0);
    }
    public int getNumTag() {
        return tags;
    }
    public Pose2d getEstimatedPose() {
        return llPoseOne;
    }
}