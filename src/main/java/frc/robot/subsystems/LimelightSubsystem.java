package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.RawFiducial;

public class LimelightSubsystem extends SubsystemBase{
    public LimelightResults llresult;
    public  RawFiducial[] fiducials;
    public double[] distances;
    public double ambiguity;

    private final String limelight;
    private final CommandSwerveDrivetrain drivetrain;
    public CANdle candle;

    public LimelightSubsystem(String limelightName, CommandSwerveDrivetrain drivetrain, CANdle candle){
        this.limelight = limelightName;   
        this.drivetrain = drivetrain;
        this.candle = candle;
    }

    public boolean hasTarget(){
        return llresult != null && llresult.valid && llresult.botpose_tagcount > 0 && fiducials != null && fiducials.length > 0;
        }

    @Override
    public void periodic(){
        llresult = LimelightHelpers.getLatestResults(limelight);
        this.fiducials = LimelightHelpers.getRawFiducials(limelight);

        if (hasTarget()){
            double [] distances ={
                fiducials[0].distToCamera
            };
             
            ambiguity = fiducials[0].ambiguity;
            SmartDashboard.putNumber("Limelight ambiguity", ambiguity);

            double area = fiducials[0].ta;
            SmartDashboard.putNumber("Limelight percent area", area);


            //distance, ambiguity, speed, angular velocity
            if((distances[0] > 0.75) && (distances[0] < 2) && ambiguity < 0.7  && drivetrain.getLinearSpeedMetersPerSecond() < 5 && drivetrain.getOmegaRadPerSec() < (Math.PI/4)){
                Pose2d llPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
                double llTimestamp = Timer.getFPGATimestamp() - (llresult.latency_pipeline / 1000.0) - (llresult.latency_capture/ 1000.0);
                drivetrain.setVisionMeasurementStdDevs(null);
                drivetrain.addVisionMeasurement(llPose, llTimestamp);
                candle.setControl(new SolidColor(0, 26).withColor(new RGBWColor(Color.kOrange).scaleBrightness(1)));
            } else{
                candle.setControl(new SolidColor(0, 26).withColor(new RGBWColor(new Color(0,0,0)).scaleBrightness(1)));
            
            }
        }
        else {
            candle.setControl(new SolidColor(0, 26).withColor(new RGBWColor(new Color(0,0,0)).scaleBrightness(1)));
        }
        if(llresult != null && llresult.valid ){
            SmartDashboard.putNumber("Number of apriltags", llresult.botpose_tagcount);
        } else {
            SmartDashboard.putNumber("Number of apriltags", -1);
        }
        SmartDashboard.putBoolean("llresult valid", llresult.valid);
        SmartDashboard.putBoolean("llresult not null", llresult != null);

    }

    
}
