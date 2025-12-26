package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase{
    
    private CommandSwerveDrivetrain drivetrain;
    private LED candle;

    private Pose2d llPose;
    private LimelightHelpers.PoseEstimate llResult;
    private Pose3d targetResult;

    private int tags;
    private StructArrayLogEntry<Pose3d> visionTargetsLog;

    public Limelight(CommandSwerveDrivetrain drivetrain, LED candle){
        this.drivetrain = drivetrain;
        this.candle = candle;

        llPose = new Pose2d();
        llResult = new LimelightHelpers.PoseEstimate();
        targetResult = new Pose3d();
        tags = 0;
        LimelightHelpers.setPipelineIndex(LimelightConstants.kName, LimelightConstants.kPipelineIndex);

        var log = DataLogManager.getLog();
        visionTargetsLog = StructArrayLogEntry.create(log, "/vision/targetPoses", Pose3d.struct);
    }

    @Override
    public void periodic(){

        llResult = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.kName);
        tags = 0;
        SmartDashboard.putBoolean("LL TV", LimelightHelpers.getTV(LimelightConstants.kName));
        SmartDashboard.putNumber("LL tx", LimelightHelpers.getTX(LimelightConstants.kName));
        SmartDashboard.putNumber("LL ty", LimelightHelpers.getTY(LimelightConstants.kName));
        SmartDashboard.putNumber("LL ta", LimelightHelpers.getTA(LimelightConstants.kName));
        
        if(llResult!= null){
            tags = llResult.tagCount;
        }
        if(llResult != null && llResult.tagCount >= LimelightConstants.kMinTags && llResult.rawFiducials != null && llResult.rawFiducials.length > 0 ) {
            targetResult = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.kName);

            llPose = llResult.pose;
            visionTargetsLog.append(new Pose3d[]{targetResult});
            SmartDashboard.putNumber("target coords x", targetResult.getX());
            SmartDashboard.putNumber("target coords Y", targetResult.getY());
            SmartDashboard.putNumber("target coords Z", targetResult.getZ());

            if(llResult.rawFiducials[0].ambiguity < LimelightConstants.kMaxAmbiguity
                && llResult.rawFiducials[0].distToCamera < LimelightConstants.kMaxDistance) {
                drivetrain.addVisionMeasurement(
                    llPose,
                    llResult.timestampSeconds,
                    LimelightConstants.kStdDevs
                    );

            }
        } else{
            visionTargetsLog.append(new Pose3d[0]);
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
            SmartDashboard.putNumber("LL Estimated Pose X", llPose.getX());
            SmartDashboard.putNumber("LL Estimated Pose Y", llPose.getY());
            SmartDashboard.putNumber("LL Estimated Pose Theta", llPose.getRotation().getDegrees());
        }
       
    }

    public boolean hasValidTarget(){
        return (llResult != null && llResult != null && llResult.tagCount >= LimelightConstants.kMinTags && llResult.rawFiducials != null && llResult.rawFiducials.length > 0);
    }
    public int getNumTag() {
        return tags;
    }
    public Pose2d getEstimatedPose() {
        return llPose;
    }
}
