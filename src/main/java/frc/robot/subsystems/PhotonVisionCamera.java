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

    private Transform3d firstllPose;
    private Transform3d secondllPose;
    private PhotonCamera cameraOne;   
    private PhotonCamera cameraTwo;    
    private PhotonPipelineResult PVresult;
    private PhotonPipelineResult PVresult2;
    private AprilTagFieldLayout layout;
    private int tagsize1;
    private int tagsize2;
    private Pose3d robotPose;

    public PhotonVisionCamera(CommandSwerveDrivetrain drivetrain, LED candle, AprilTagFieldLayout layout){
        this.drivetrain = drivetrain;
        this.candle = candle;
        this.cameraOne = new PhotonCamera("cameraone");
        this.cameraTwo = new PhotonCamera("cameratwo");
        this.PVresult = null;
        this.PVresult2 = null;
        this.layout = layout;
        this.firstllPose = new Transform3d();
        this.secondllPose = new Transform3d();
        robotPose = new Pose3d();
        tagsize1 = 0;
        tagsize2 = 0;
        LimelightHelpers.setPipelineIndex(LimelightConstants.kName, LimelightConstants.kPipelineIndex);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Camera Connected", cameraOne.isConnected());
        SmartDashboard.putBoolean("Result Not Null", PVresult != null);

        PVresult = cameraOne.getLatestResult();
        PVresult2 = cameraTwo.getLatestResult();        
        
        if(PVresult.hasTargets()){
            tagsize1 = PVresult.getTargets().size();
        } else{
            tagsize1 = 0;
        }
        if(PVresult2.hasTargets()){
            tagsize2 =  PVresult2.getTargets().size();
        } else{
            tagsize2 = 0;
        }


        if (PVresult.hasTargets() || PVresult2.hasTargets()) {

            PhotonTrackedTarget firstTarget = PVresult.getBestTarget();
            PhotonTrackedTarget secondTarget = PVresult.getBestTarget();

            SmartDashboard.putNumber("CamOne ambiguity", firstTarget.getPoseAmbiguity());
            SmartDashboard.putNumber("CamTwo ambiguity", secondTarget.getPoseAmbiguity());

            if((firstTarget!= null && firstTarget.poseAmbiguity < LimelightConstants.kMaxAmbiguity) 
            || (secondTarget != null && secondTarget.poseAmbiguity < LimelightConstants.kMaxAmbiguity)) {

                int firsttagId = firstTarget.getFiducialId();
                int secondtagId = secondTarget.getFiducialId();
                SmartDashboard.putNumber("first tag id", firsttagId);
                SmartDashboard.putNumber("second tag id", secondtagId);

                
                firstllPose =firstTarget.getBestCameraToTarget();
                secondllPose = secondTarget.getBestCameraToTarget();
                
                Pose3d firstTagPose = layout.getTagPose(firstTarget.getFiducialId()).orElse(null);
                SmartDashboard.putBoolean("tagpose", firstTagPose != null);
                if( firstTagPose!= null){
                    robotPose = tagPose.transformBy(firstllPose.inverse());
                    SmartDashboard.putNumber("robotpose x", robotPose.getX());
                    SmartDashboard.putNumber("robotpose y", robotPose.getY());


                    drivetrain.addVisionMeasurement(
                        robotPose.toPose2d(),
                        PVresult.getTimestampSeconds()
                        );
                }
              

            }
        }
 
        if (hasValidTargetCamOne() || hasValidTargetCamTwo()){
            candle.cameraSetColor(Color.kGreen, 1);
        }
        else {
            candle.cameraClearColor();

            

        }
 
        SmartDashboard.putNumber("CameraOne tag count", tagsize1);
        SmartDashboard.putBoolean("CameraOne has target", hasValidTargetCamOne());
        SmartDashboard.putNumber("CameraTwo tag count", tagsize2);
        SmartDashboard.putBoolean("CameraTwo has target", hasValidTargetCamTwo());


        if(PVresult != null && PVresult.hasTargets() || PVresult2 != null && PVresult2.hasTargets()) {
            SmartDashboard.putNumber("LL Estimated Pose X", robotPose.getX());
            SmartDashboard.putNumber("LL Estimated Pose Y", robotPose.getY());
            SmartDashboard.putNumber("LL Estimated Pose Theta", robotPose.getRotation().toRotation2d().getDegrees());
        }
       
    }

    public boolean hasValidTargetCamOne(){
        return (PVresult != null && tagsize1 >= LimelightConstants.kMinTags);
    }
    public boolean hasValidTargetCamTwo(){
        return (PVresult2 != null && tagsize2 >= LimelightConstants.kMinTags);
    }
    public int getNumTag() {
        return tagsize1 + tagsize2;
    }
    public Pose2d getEstimatedPose() {
        Pose2d fieldPose = layout.getTagPose(PVresult.getBestTarget().getFiducialId()).orElse(new Pose3d()).toPose2d();
        return fieldPose;
        
    }
}
