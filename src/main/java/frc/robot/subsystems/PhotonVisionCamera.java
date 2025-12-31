package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class PhotonVisionCamera extends SubsystemBase{
    
    private LED candle;

    private PhotonCamera cameraOne;
    private PhotonCamera cameraTwo;    
    private PhotonPipelineResult PVresult;
    private PhotonPipelineResult PVresultTwo;
    private AprilTagFieldLayout layout;
    private int tags;
    private Transform3d cameraToRobotOne = new Transform3d(
    new Translation3d(0.42, 0.0, 0.5),  // X, Y, Z in meters
    new Rotation3d(0, 0, 0)  // Roll, Pitch, Yaw in radians
    );
    private Transform3d cameraToRobotTwo = new Transform3d(
        new Translation3d(-0.42, 0.0, 0.5),  // X, Y, Z in meters
        new Rotation3d(0, 0, Math.PI)  // Roll, Pitch, Yaw in radians
        );



    public PhotonVisionCamera(LED candle, AprilTagFieldLayout layout){
        this.candle = candle;
        this.cameraOne = new PhotonCamera("cameraone");
        this.cameraTwo = new PhotonCamera("cameratwo");
        this.PVresult = null;
        this.PVresultTwo = null;
        this.layout = layout;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Camera Connected", cameraOne.isConnected());
        SmartDashboard.putBoolean("Result Not Null", PVresult != null);
        SmartDashboard.putBoolean("Camera Connected", cameraOne.isConnected());

        PVresult = cameraOne.getLatestResult();
        PVresultTwo = cameraTwo.getLatestResult();
        SmartDashboard.putBoolean("targets one", PVresult.hasTargets());
        SmartDashboard.putBoolean("targets two", PVresultTwo.hasTargets());

        tags = PVresult.getTargets().size() + PVresultTwo.getTargets().size();

        SmartDashboard.putNumber("tag count", tags);

        if (hasValidTarget()){

            candle.cameraSetColor(Color.kGreen, 1);
        }
        else {
            candle.cameraClearColor();       

        }
       
    }

    public PhotonPipelineResult getCamOneResult(){
        return PVresult;
    }
    public PhotonPipelineResult getCamTwoResult(){
        return PVresultTwo;
    }

    public Pose2d getPoseOne(){
        if (PVresultTwo == null || !PVresultTwo.hasTargets()) {
            return null;
        }
        PhotonTrackedTarget bestTarget = PVresultTwo.getBestTarget();
        if (bestTarget == null) {
            return null;
        }
        int tagId = bestTarget.getFiducialId();
        Optional<Pose3d> tagPoseOpt = layout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) {
            return null;
        }
        Pose2d camPose = PhotonUtils.estimateFieldToRobotAprilTag(
                bestTarget.getBestCameraToTarget(), tagPoseOpt.get(), cameraToRobotTwo).toPose2d();
        return camPose;
    }
    public Transform3d getCamToRobotOne(){
        return cameraToRobotOne;
    }

    
    public Transform3d getCamToRobotTwo(){
        return cameraToRobotTwo;
    }

    public AprilTagFieldLayout getLayout(){
        return layout;
    }

    public boolean hasValidTarget(){
        return (PVresult != null && tags >= LimelightConstants.kMinTags);
    }
    public int getNumTag() {
        return tags;
    }
   
}