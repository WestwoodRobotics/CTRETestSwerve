package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class PhotonVisionCamera extends SubsystemBase{
    
    private CommandSwerveDrivetrain drivetrain;
    private LED candle;

    private Transform3d llPose;
    private PhotonCamera cameraOne;
    private PhotonCamera cameraTwo;    
    private PhotonPipelineResult PVresult;
    private PhotonPipelineResult PVresultTwo;
    private AprilTagFieldLayout layout;
    private int tags;
    private int tagsTwo;

    private Pose3d robotPose;

    public PhotonVisionCamera(CommandSwerveDrivetrain drivetrain, LED candle, AprilTagFieldLayout layout){
        this.drivetrain = drivetrain;
        this.candle = candle;
        this.cameraOne = new PhotonCamera("cameraone");
        this.cameraTwo = new PhotonCamera("cameratwo");
        this.PVresult = null;
        this.PVresultTwo = null;
        this.layout = layout;
        llPose = new Transform3d();
        robotPose = new Pose3d();
        tags = 0;
        tagsTwo = 0;
        LimelightHelpers.setPipelineIndex(LimelightConstants.kName, LimelightConstants.kPipelineIndex);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Camera Connected", cameraOne.isConnected());
        SmartDashboard.putBoolean("Result Not Null", PVresult != null);

        PVresult = cameraOne.getLatestResult();
        PVresultTwo = cameraOne.getLatestResult();

        if(PVresult.hasTargets()){
            tags = PVresult.getTargets().size();
        } else{
            tags = 0;
        }

        if(PVresultTwo.hasTargets()){
            tagsTwo = PVresultTwo.getTargets().size();
        } else{
            tagsTwo = 0;
        }


        if( PVresult.hasTargets()) {

            PhotonTrackedTarget bestTarget = PVresult.getBestTarget();

            SmartDashboard.putNumber("LL ambiguity", bestTarget.getPoseAmbiguity());

            if(bestTarget!= null && bestTarget.poseAmbiguity < LimelightConstants.kMaxAmbiguity
               ) {
                int tagId = bestTarget.getFiducialId();
                SmartDashboard.putNumber("tag id", tagId);
                
                llPose =bestTarget.getBestCameraToTarget();
                Pose3d tagPose = layout.getTagPose(bestTarget.getFiducialId()).orElse(null);
                SmartDashboard.putBoolean("tagpose", tagPose != null);
                if( tagPose!= null){
                    robotPose = tagPose.transformBy(llPose.inverse());
                    SmartDashboard.putNumber("robotpose x", robotPose.getX());
                    SmartDashboard.putNumber("robotpose y", robotPose.getY());


                    drivetrain.addVisionMeasurement(
                        robotPose.toPose2d(),
                        PVresult.getTimestampSeconds()
                        );
                }
              

            }
        }

        if( PVresultTwo.hasTargets()) {

            PhotonTrackedTarget bestTarget = PVresult.getBestTarget();

            SmartDashboard.putNumber("LL ambiguity", bestTarget.getPoseAmbiguity());

            if(bestTarget!= null && bestTarget.poseAmbiguity < LimelightConstants.kMaxAmbiguity
               ) {
                int tagId = bestTarget.getFiducialId();
                SmartDashboard.putNumber("tag id", tagId);
                
                llPose =bestTarget.getBestCameraToTarget();
                Pose3d tagPose = layout.getTagPose(bestTarget.getFiducialId()).orElse(null);
                SmartDashboard.putBoolean("tagpose", tagPose != null);
                if( tagPose!= null){
                    robotPose = tagPose.transformBy(llPose.inverse());
                    SmartDashboard.putNumber("robotpose x", robotPose.getX());
                    SmartDashboard.putNumber("robotpose y", robotPose.getY());


                    drivetrain.addVisionMeasurement(
                        robotPose.toPose2d(),
                        PVresult.getTimestampSeconds()
                        );
                }
              

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


        if(PVresult != null &&  PVresult.hasTargets()) {
            SmartDashboard.putNumber("LL Estimated Pose X", robotPose.getX());
            SmartDashboard.putNumber("LL Estimated Pose Y", robotPose.getY());
            SmartDashboard.putNumber("LL Estimated Pose Theta", robotPose.getRotation().toRotation2d().getDegrees());
        }
       
    }

    public boolean hasValidTarget(){
        return (PVresult != null && tags >= LimelightConstants.kMinTags);
    }
    public int getNumTag() {
        return tags;
    }
    public Pose2d getEstimatedPose() {
        Pose2d fieldPose = layout.getTagPose(PVresult.getBestTarget().getFiducialId()).orElse(new Pose3d()).toPose2d();
        return fieldPose;
        
    }
}
