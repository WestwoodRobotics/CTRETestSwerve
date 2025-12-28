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
    
    private CommandSwerveDrivetrain drivetrain;
    private LED candle;

    private Transform3d llPose;
    private PhotonCamera cameraOne;
    private PhotonCamera cameraTwo;    
    private PhotonPipelineResult PVresult;
    private PhotonPipelineResult PVresultTwo;
    private AprilTagFieldLayout layout;
    private int tags;
    private Transform3d cameraToRobot = new Transform3d(
    new Translation3d(0.5, 0.0, 0.1),  // X, Y, Z in meters
    new Rotation3d(0, Math.toRadians(-15), 0)  // Roll, Pitch, Yaw in radians
    );

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
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Camera Connected", cameraOne.isConnected());
        SmartDashboard.putBoolean("Result Not Null", PVresult != null);

        PVresult = cameraOne.getLatestResult();
        PVresultTwo = cameraTwo.getLatestResult();

        tags = PVresult.getTargets().size() + PVresultTwo.getTargets().size();

        // CAM ONE can see tag, CAM TWO cannot
        if(PVresult.hasTargets() && !PVresultTwo.hasTargets()) {

            PhotonTrackedTarget bestTarget = PVresult.getBestTarget();
            int tagId = bestTarget.getFiducialId();
            Optional<Pose3d> tagPoseOpt = layout.getTagPose(tagId);
            
            if(tagPoseOpt.isPresent()){
                
                robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                    bestTarget.getBestCameraToTarget(), tagPoseOpt.get(), cameraToRobot);

                double distance = PhotonUtils.getDistanceToPose(drivetrain.getState().Pose, tagPoseOpt.get().toPose2d());

                if(distance < LimelightConstants.kMaxDistance && bestTarget.poseAmbiguity < LimelightConstants.kMaxAmbiguityOdometry
                 && bestTarget.poseAmbiguity > LimelightConstants.kMaxAmbiguityGyro) {

                    Pose2d odomPose2d = new Pose2d(
                        robotPose.getX(),robotPose.getY(), drivetrain.getState().RawHeading
                    );

                    drivetrain.addVisionMeasurement(
                        odomPose2d,
                        PVresult.getTimestampSeconds(),   
                        LimelightConstants.kStdDevs
                        );
                
                }

                if(distance < LimelightConstants.kMaxDistance && bestTarget.poseAmbiguity < LimelightConstants.kMaxAmbiguityGyro){


                    drivetrain.addVisionMeasurement(
                        robotPose.toPose2d(),
                        PVresult.getTimestampSeconds(),   
                        LimelightConstants.kStdDevs
                        );
                
                }
            }
        }


        
        // CAM TWO can see tag, CAM ONE cannot
        if(PVresultTwo.hasTargets() && !PVresult.hasTargets()) {

            PhotonTrackedTarget bestTarget = PVresultTwo.getBestTarget();
            int tagId = bestTarget.getFiducialId();
            Optional<Pose3d> tagPoseOpt = layout.getTagPose(tagId);
            
            if(tagPoseOpt.isPresent()){
                
                robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                    bestTarget.getBestCameraToTarget(), tagPoseOpt.get(), cameraToRobot);

                double distance = PhotonUtils.getDistanceToPose(drivetrain.getState().Pose, tagPoseOpt.get().toPose2d());

                if(distance < LimelightConstants.kMaxDistance && bestTarget.poseAmbiguity < LimelightConstants.kMaxAmbiguityOdometry
                 && bestTarget.poseAmbiguity > LimelightConstants.kMaxAmbiguityGyro) {

                    Pose2d odomPose2d = new Pose2d(
                        robotPose.getX(),robotPose.getY(), drivetrain.getState().RawHeading
                    );

                    drivetrain.addVisionMeasurement(
                        odomPose2d,
                        PVresultTwo.getTimestampSeconds(),   
                        LimelightConstants.kStdDevs
                        );
                
                }

                if(distance < LimelightConstants.kMaxDistance && bestTarget.poseAmbiguity < LimelightConstants.kMaxAmbiguityGyro){

                    drivetrain.addVisionMeasurement(
                        robotPose.toPose2d(),
                        PVresultTwo.getTimestampSeconds(),   
                        LimelightConstants.kStdDevs
                        );

                }
            }
        }



        //CAM TWO AND CAM ONE can see tags
        if(PVresultTwo.hasTargets() && PVresult.hasTargets()){
            PhotonTrackedTarget bestTargetOne = PVresult.getBestTarget();
            int tagIdOne = bestTargetOne.getFiducialId();
            Optional<Pose3d> tagPoseOptOne = layout.getTagPose(tagIdOne);

            PhotonTrackedTarget bestTargetTwo = PVresultTwo.getBestTarget();
            int tagIdTwo = bestTargetTwo.getFiducialId();
            Optional<Pose3d> tagPoseOptTwo = layout.getTagPose(tagIdTwo);

            if(tagPoseOptOne.isPresent() && tagPoseOptTwo.isPresent()){
                
                double distanceOne =  PhotonUtils.getDistanceToPose(drivetrain.getState().Pose, tagPoseOptOne.get().toPose2d());
                double distanceTwo =  PhotonUtils.getDistanceToPose(drivetrain.getState().Pose, tagPoseOptTwo.get().toPose2d());

                double normalizedDistanceOne = 1 - Math.min((distanceOne / LimelightConstants.kMaxDistance), 1.0);
                double normalizedDistanceTwo = 1 - Math.min((distanceTwo / LimelightConstants.kMaxDistance), 1.0);

                double normalizedAmbiguityOne = 1 - bestTargetOne.poseAmbiguity;
                double normalizedAmbiguityTwo = 1 - bestTargetTwo.poseAmbiguity;

                double confidenceOne = normalizedDistanceOne * normalizedAmbiguityOne;
                double confidenceTwo = normalizedDistanceTwo * normalizedAmbiguityTwo;

                double totalConfidence = confidenceOne + confidenceTwo;

                double weightOne = confidenceOne / totalConfidence;
                double weightTwo = confidenceTwo / totalConfidence;

                Pose3d robotPoseOne = PhotonUtils.estimateFieldToRobotAprilTag(
                    bestTargetOne.getBestCameraToTarget(), tagPoseOptOne.get(), cameraToRobot);
                Pose3d robotPoseTwo = PhotonUtils.estimateFieldToRobotAprilTag(
                    bestTargetTwo.getBestCameraToTarget(), tagPoseOptTwo.get(), cameraToRobot);

                    Pose2d combinedPose = new Pose2d();

                boolean updategyroOne = bestTargetOne.poseAmbiguity < LimelightConstants.kMaxAmbiguityGyro;
                boolean updategyroTwo = bestTargetTwo.poseAmbiguity < LimelightConstants.kMaxAmbiguityGyro;

                
                //if the CAM ONE ambiguity is in the gyro update range but CAM TWO ambiguity is not, use CAM ONE for gyro
                if(distanceOne < LimelightConstants.kMaxDistance && distanceTwo < LimelightConstants.kMaxDistance &&
                updategyroOne && !(updategyroTwo)){
                    
                    combinedPose = new Pose2d(
                    robotPoseOne.getX() * weightOne + robotPoseTwo.getX() * weightTwo,
                    robotPoseOne.getY() * weightOne + robotPoseTwo.getY() * weightTwo,
                    robotPoseOne.toPose2d().getRotation()
                    );

                    drivetrain.addVisionMeasurement(
                        combinedPose,
                        (PVresult.getTimestampSeconds() + PVresultTwo.getTimestampSeconds()) / 2.0,  
                        LimelightConstants.kStdDevs
                        );

                }

                //if the CAM TWO ambiguity is in the gyro update range but CAM ONE ambiguity is not, use CAM TWO for gyro

                else if(distanceOne < LimelightConstants.kMaxDistance && distanceTwo < LimelightConstants.kMaxDistance &&
                !(updategyroOne) && updategyroTwo){
                    
                    combinedPose = new Pose2d(
                    robotPoseOne.getX() * weightOne + robotPoseTwo.getX() * weightTwo,
                    robotPoseOne.getY() * weightOne + robotPoseTwo.getY() * weightTwo,
                    robotPoseTwo.toPose2d().getRotation()
                    );

                    drivetrain.addVisionMeasurement(
                        combinedPose,
                        (PVresult.getTimestampSeconds() + PVresultTwo.getTimestampSeconds()) / 2.0,
                        LimelightConstants.kStdDevs
                        );

                }

                //if the CAM TWO ambiguity is in the gyro update range AND CAM ONE ambiguity is in the gyro update range, combine the rotations for gyro


                else if(distanceOne < LimelightConstants.kMaxDistance && distanceTwo < LimelightConstants.kMaxDistance &&
                updategyroOne && updategyroTwo){
                    
                    Rotation2d combinedrotation = robotPoseOne.toPose2d().getRotation().interpolate(robotPoseTwo.toPose2d().getRotation(), weightTwo);

                    combinedPose = new Pose2d(
                    robotPoseOne.getX() * weightOne + robotPoseTwo.getX() * weightTwo,
                    robotPoseOne.getY() * weightOne + robotPoseTwo.getY() * weightTwo,
                    combinedrotation
                    );

                    drivetrain.addVisionMeasurement(
                        combinedPose,
                        (PVresult.getTimestampSeconds() + PVresultTwo.getTimestampSeconds()) / 2.0, 
                        LimelightConstants.kStdDevs
                        );

                } 
                
                
                //if the CAM TWO ambiguity is NOT in the gyro update range AND CAM ONE ambiguity is NOT 
                //in the gyro update range, dont update gyro

                
                else if (distanceOne < LimelightConstants.kMaxDistance && distanceTwo < LimelightConstants.kMaxDistance &&
                !updategyroOne && !updategyroTwo){

                    combinedPose = new Pose2d(
                    robotPoseOne.getX() * weightOne + robotPoseTwo.getX() * weightTwo,
                    robotPoseOne.getY() * weightOne + robotPoseTwo.getY() * weightTwo,
                    drivetrain.getState().RawHeading
                    );

                    drivetrain.addVisionMeasurement(
                        combinedPose,
                        (PVresult.getTimestampSeconds() + PVresultTwo.getTimestampSeconds()) / 2.0, 
                        LimelightConstants.kStdDevs
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