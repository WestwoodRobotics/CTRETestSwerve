package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class PhotonVisionCamera extends SubsystemBase{
    
    private CommandSwerveDrivetrain drivetrain;
    private LED candle;

    private Pose2d llPose;
    private PhotonCamera cameraOne;    
    private PhotonPipelineResult PVresult;
    private AprilTagFieldLayout layout;

    private int tags;
    private boolean isOrange;

    public PhotonVisionCamera(CommandSwerveDrivetrain drivetrain, LED candle, AprilTagFieldLayout layout){
        this.drivetrain = drivetrain;
        this.candle = candle;

        this.PVresult = null;
        this.layout = layout;
        llPose = new Pose2d();
        tags = 0;
        LimelightHelpers.setPipelineIndex(LimelightConstants.kName, LimelightConstants.kPipelineIndex);
    }

    @Override
    public void periodic(){

        PVresult = cameraOne.getLatestResult();
        tags = PVresult.getTargets().size();

        if(PVresult != null && llResult != null && llResult.tagCount >= LimelightConstants.kMinTags && llResult.rawFiducials != null && llResult.rawFiducials.length > 0 ) {

            llPose = llResult.pose;


            if(llResult.rawFiducials[0].ambiguity < LimelightConstants.kMaxAmbiguity
                && llResult.rawFiducials[0].distToCamera < LimelightConstants.kMaxDistance) {
                drivetrain.addVisionMeasurement(
                    llPose,
                    llResult.timestampSeconds
                    );

            }
        }
/* 
        if (hasValidTarget()){
            candle.setSolidColor(Color.kOrange, 1);
            isOrange = true;
        }
        else {
            if(isOrange){
                candle.clearColor();
                isOrange = false;

            }

        }
 */
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
