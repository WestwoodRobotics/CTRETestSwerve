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
    private Transform3d cameraToRobot = new Transform3d(
    new Translation3d(0.5, 0.0, 0.1),  // X, Y, Z in meters
    new Rotation3d(0, 0, 0)  // Roll, Pitch, Yaw in radians
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

        PVresult = cameraOne.getLatestResult();
        PVresultTwo = cameraTwo.getLatestResult();

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

    public Transform3d getCamToRobot(){
        return cameraToRobot;
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
    public Pose2d getEstimatedPose() {
        Pose2d fieldPose = layout.getTagPose(PVresult.getBestTarget().getFiducialId()).orElse(new Pose3d()).toPose2d();
        return fieldPose;
        
    }
}