package frc.robot.commands.vision;

import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionCamera;

public class PhotonDefault extends Command{
    private final PhotonVisionCamera vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout layout;
    private final Transform3d cameraToRobotOne;
    private final Transform3d cameraToRobotTwo;
    private Pose2d combinedPose = new Pose2d();
    private Pose2d cachedRobotPose = new Pose2d();
    private Rotation2d finalrotation = new Rotation2d();

    public PhotonDefault(PhotonVisionCamera camera, CommandSwerveDrivetrain drivetrain){
        this.vision = camera;
        this.drivetrain = drivetrain;
        this.layout = camera.getLayout();
        this.cameraToRobotOne = camera.getCamToRobotOne();
        this.cameraToRobotTwo = camera.getCamToRobotTwo();
        addRequirements(camera);
    }

    @Override
    public void execute(){
        PhotonPipelineResult PVresult = vision.getCamOneResult();
        PhotonPipelineResult PVresultTwo = vision.getCamTwoResult();
        SmartDashboard.putBoolean("pv one target", PVresult.hasTargets());
        SmartDashboard.putBoolean("pv two target", PVresultTwo.hasTargets());

        if(PVresult == null || PVresultTwo == null){
            return;
        }
        // CAM ONE can see tag, CAM TWO cannot
        if(PVresult.hasTargets() && !PVresultTwo.hasTargets()) {
            processSingleCam(PVresult, cameraToRobotOne);
        }   
        
        // CAM TWO can see tag, CAM ONE cannot
        else if(PVresultTwo.hasTargets() && !PVresult.hasTargets()) {
            processSingleCam(PVresultTwo, cameraToRobotTwo);
        }
        
        //CAM TWO AND CAM ONE can see tags
        else if(PVresultTwo.hasTargets() && PVresult.hasTargets()){
            processDoubleCam(PVresult, PVresultTwo);
        }
    }




    public void processSingleCam(PhotonPipelineResult PVresult, Transform3d cameraToRobot){

        PhotonTrackedTarget bestTarget = PVresult.getBestTarget();
        int tagId = bestTarget.getFiducialId();
        Optional<Pose3d> tagPoseOpt = layout.getTagPose(tagId);
        
        if(tagPoseOpt.isPresent()){
        
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                bestTarget.getBestCameraToTarget(), tagPoseOpt.get(), cameraToRobot);
            cachedRobotPose = drivetrain.getState().Pose;
            double distance = PhotonUtils.getDistanceToPose(cachedRobotPose, tagPoseOpt.get().toPose2d());
            double translationalVelocity = Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
            double rotationalVelocity = Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond);
            SmartDashboard.putNumber("area", bestTarget.area);
            if(distance < LimelightConstants.kMaxDistance /* 
            && bestTarget.area > LimelightConstants.kMinAreaOdom
            && bestTarget.area < LimelightConstants.kMinAreaGyro */
            && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
            && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity) {

                combinedPose = new Pose2d(
                    robotPose.getX(),robotPose.getY(), cachedRobotPose.getRotation()
                );

                drivetrain.addVisionMeasurement(
                    combinedPose,
                    PVresult.getTimestampSeconds(),   
                    LimelightConstants.kStdDevs
                    );
            
            }

            else if(distance < LimelightConstants.kMaxDistance
             && bestTarget.area > LimelightConstants.kMinAreaGyro
             && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
             && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity) {


                drivetrain.addVisionMeasurement(
                    robotPose.toPose2d(),
                    PVresult.getTimestampSeconds(),   
                    LimelightConstants.kStdDevs
                    );
            
            }
        }
        
    }





    public void processDoubleCam(PhotonPipelineResult PVresult, PhotonPipelineResult PVresultTwo){
        PhotonTrackedTarget bestTargetOne = PVresult.getBestTarget();
        int tagIdOne = bestTargetOne.getFiducialId();
        Optional<Pose3d> tagPoseOptOne = layout.getTagPose(tagIdOne);

        PhotonTrackedTarget bestTargetTwo = PVresultTwo.getBestTarget();
        int tagIdTwo = bestTargetTwo.getFiducialId();
        Optional<Pose3d> tagPoseOptTwo = layout.getTagPose(tagIdTwo);

        if(tagPoseOptOne.isPresent() && tagPoseOptTwo.isPresent()){
            cachedRobotPose = drivetrain.getState().Pose;

            double distanceOne =  PhotonUtils.getDistanceToPose(cachedRobotPose, tagPoseOptOne.get().toPose2d());
            double distanceTwo =  PhotonUtils.getDistanceToPose(cachedRobotPose, tagPoseOptTwo.get().toPose2d());

            double normalizedDistanceOne = 1 - Math.min((distanceOne / LimelightConstants.kMaxDistance), 1.0);
            double normalizedDistanceTwo = 1 - Math.min((distanceTwo / LimelightConstants.kMaxDistance), 1.0);

            double areaOne = bestTargetOne.area;
            double areaTwo = bestTargetTwo.area;

            double confidenceOne = normalizedDistanceOne * (areaOne / 100);
            double confidenceTwo = normalizedDistanceTwo * (areaTwo / 100);

            double totalConfidence = confidenceOne + confidenceTwo;

            double weightOne = confidenceOne / totalConfidence;
            double weightTwo = confidenceTwo / totalConfidence;

            Pose3d robotPoseOne = PhotonUtils.estimateFieldToRobotAprilTag(
                bestTargetOne.getBestCameraToTarget(), tagPoseOptOne.get(), cameraToRobotOne);
            Pose3d robotPoseTwo = PhotonUtils.estimateFieldToRobotAprilTag(
                bestTargetTwo.getBestCameraToTarget(), tagPoseOptTwo.get(), cameraToRobotTwo);


            boolean updategyroOne = bestTargetOne.area > LimelightConstants.kMinAreaGyro;
            boolean updategyroTwo = bestTargetTwo.area > LimelightConstants.kMinAreaGyro;

            double translationalVelocity = Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
            double rotationalVelocity = Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond);
            
            //if the CAM ONE ambiguity is in the gyro update range but CAM TWO ambiguity is not, use CAM ONE for gyro
            if(distanceOne < LimelightConstants.kMaxDistance 
            && distanceTwo < LimelightConstants.kMaxDistance 
            && updategyroOne && !(updategyroTwo)
            && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
            && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity){
                
                finalrotation = robotPoseOne.toPose2d().getRotation();
                
            }

            //if the CAM TWO ambiguity is in the gyro update range but CAM ONE ambiguity is not, use CAM TWO for gyro

            else if(distanceOne < LimelightConstants.kMaxDistance 
            && distanceTwo < LimelightConstants.kMaxDistance 
            && !(updategyroOne) && updategyroTwo
            && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
            && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity){
                
                finalrotation = robotPoseTwo.toPose2d().getRotation();

            }

            //if the CAM TWO ambiguity is in the gyro update range AND CAM ONE ambiguity is in the gyro update range, combine the rotations for gyro


            else if(distanceOne < LimelightConstants.kMaxDistance 
            && distanceTwo < LimelightConstants.kMaxDistance 
            && updategyroOne && updategyroTwo
            && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
            && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity){
                
                finalrotation = robotPoseOne.toPose2d().getRotation().interpolate(robotPoseTwo.toPose2d().getRotation(), weightTwo);

              

            } 
            
            
            //if the CAM TWO ambiguity is NOT in the gyro update range AND CAM ONE ambiguity is NOT 
            //in the gyro update range, dont update gyro

            
            else if (distanceOne < LimelightConstants.kMaxDistance 
            && distanceTwo < LimelightConstants.kMaxDistance 
            && !updategyroOne && !updategyroTwo
            && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
            && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity){
                finalrotation = drivetrain.getState().RawHeading;
            }

            combinedPose = new Pose2d(
                (robotPoseOne.getX() * weightOne) + (robotPoseTwo.getX() * weightTwo),
                (robotPoseOne.getY() * weightOne) + (robotPoseTwo.getY() * weightTwo),
                finalrotation
            );

            drivetrain.addVisionMeasurement(
                combinedPose,
                (PVresult.getTimestampSeconds() + PVresultTwo.getTimestampSeconds()) / 2.0,
                LimelightConstants.kStdDevs
            );
        }

    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
