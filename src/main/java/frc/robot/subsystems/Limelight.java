package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase{
    
    private CommandSwerveDrivetrain drivetrain;
    private CANdle candle;

    private boolean hasValidTarget;
    private Pose2d llPose;
    private LimelightHelpers.PoseEstimate llResult;
    private int tags;

    public Limelight(CommandSwerveDrivetrain drivetrain, CANdle candle){
        this.drivetrain = drivetrain;
        this.candle = candle;

        hasValidTarget = false;
        llPose = new Pose2d();
        llResult = new LimelightHelpers.PoseEstimate();
        tags = 0;
        LimelightHelpers.setPipelineIndex(LimelightConstants.kName, LimelightConstants.kPipelineIndex);
    }

    @Override
    public void periodic(){

        llResult = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.kName);
      
        if(llResult.tagCount >= LimelightConstants.kMinTags && llResult.rawFiducials.length == 1) {

            llPose = llResult.pose;
            hasValidTarget = true;
            tags = llResult.tagCount;

            if(llResult.rawFiducials[0].ambiguity < LimelightConstants.kMaxAmbiguity
                && llResult.rawFiducials[0].distToCamera < LimelightConstants.kMaxDistance) {
                drivetrain.addVisionMeasurement(
                    llPose,
                    llResult.timestampSeconds,
                    LimelightConstants.kStdDevs);

            }
        }

        if (hasValidTarget) {
            candle.setControl(new SolidColor(0, 26).withColor(new RGBWColor(Color.kOrange).scaleBrightness(1)));
        }
        else {
            candle.setControl(new SolidColor(0, 26).withColor(new RGBWColor(new Color(0,0,0)).scaleBrightness(1)));
        }

        SmartDashboard.putNumber("LL tag count", tags);
        SmartDashboard.putNumber("LL ambiguity", llResult.rawFiducials[0].ambiguity);
        SmartDashboard.putBoolean("LL has target", hasValidTarget);
        SmartDashboard.putNumber("LL Estimated Pose X", llPose.getX());
        SmartDashboard.putNumber("LL Estimated Pose Y", llPose.getY());
        SmartDashboard.putNumber("LL Estimated Pose Theta", llPose.getRotation().getDegrees());
    }

    public boolean hasValidTarget(){
        return hasValidTarget;
    }
    public int getNumTag() {
        return tags;
    }
    public Pose2d getEstimatedPose() {
        return llPose;
    }
}
