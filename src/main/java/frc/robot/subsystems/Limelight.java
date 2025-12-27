package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase{
    
    private CommandSwerveDrivetrain drivetrain;
    private LED candle;

    private Pose2d llPose;
    private LimelightHelpers.PoseEstimate llResult;
    private LimelightHelpers.LimelightResults results;
    private Pose3d TargetPose;
    private AprilTagFieldLayout layout;
    private int tags;
    private StructArrayLogEntry<Pose3d> visionTargetsLog;

    public Limelight(CommandSwerveDrivetrain drivetrain, LED candle){
        this.drivetrain = drivetrain;
        this.candle = candle;

        llPose = new Pose2d();
        llResult = new LimelightHelpers.PoseEstimate();
        TargetPose = new Pose3d();
        tags = 0;
        LimelightHelpers.setPipelineIndex(LimelightConstants.kName, LimelightConstants.kPipelineIndex);

        try {
            this.layout = new AprilTagFieldLayout("/home/lvuser/deploy/2025-reefscape-welded.json");
        } catch (java.io.IOException e) {
            // Fallback to an empty layout if the file cannot be read
            this.layout = new AprilTagFieldLayout(java.util.List.of(), 0.0, 0.0);
            SmartDashboard.putString("LL layout error", e.getMessage());
        }

        var log = DataLogManager.getLog();
        visionTargetsLog = StructArrayLogEntry.create(log, "/vision/targetPoses", Pose3d.struct);
    }

    @Override
    public void periodic(){
        results = LimelightHelpers.getLatestResults(LimelightConstants.kName);
        llResult = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.kName);
        tags = 0;
        if(llResult!= null){
            tags = llResult.tagCount;
        }
        if(llResult != null && llResult.tagCount >= LimelightConstants.kMinTags && llResult.rawFiducials != null && llResult.rawFiducials.length > 0 ) {
            if(results.targets_Fiducials.length>0){
                LimelightHelpers.LimelightTarget_Fiducial[] target = results.targets_Fiducials;
                int id = (int) target[0].fiducialID;
                var tagposeoptional = layout.getTagPose(id);
                if(tagposeoptional.isPresent()){
                    TargetPose = tagposeoptional.get();
    
                }
            }
            llPose = llResult.pose;
            



            visionTargetsLog.append(new Pose3d[]{TargetPose});
            double[] targetPoseArray = new double[] {
                TargetPose.getX(), // X translation
                TargetPose.getY(), // Y translation
                TargetPose.getZ(), // Z translation
                TargetPose.getRotation().toRotation2d().getRadians()
            };
            
            SmartDashboard.putNumberArray("target", targetPoseArray);
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
