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
    private final Transform3d cameraToRobot;

    public PhotonDefault(PhotonVisionCamera camera, CommandSwerveDrivetrain drivetrain){
        this.vision = camera;
        this.drivetrain = drivetrain;
        this.layout = camera.getLayout();
        this.cameraToRobot = camera.getCamToRobot();

        addRequirements(camera);
    }

    @Override
    public void execute(){
        PhotonPipelineResult PVresult = vision.getCamOneResult();
        PhotonPipelineResult PVresultTwo = vision.getCamTwoResult();

        if(PVresult == null || PVresultTwo == null){
            return;
        }
        // CAM ONE can see tag, CAM TWO cannot
        if(PVresult.hasTargets() && !PVresultTwo.hasTargets()) {
            processSingleCam(PVresult);
        }   
        
        // CAM TWO can see tag, CAM ONE cannot
        else if(PVresultTwo.hasTargets() && !PVresult.hasTargets()) {
            processSingleCam(PVresultTwo);
        }
        
        //CAM TWO AND CAM ONE can see tags
        else if(PVresultTwo.hasTargets() && PVresult.hasTargets()){
            processDoubleCam(PVresult, PVresultTwo);
        }
    }




    public void processSingleCam(PhotonPipelineResult PVresult){

        PhotonTrackedTarget bestTarget = PVresult.getBestTarget();
        int tagId = bestTarget.getFiducialId();
        Optional<Pose3d> tagPoseOpt = layout.getTagPose(tagId);
        
        if(tagPoseOpt.isPresent()){
        
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
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





    public void processDoubleCam(PhotonPipelineResult PVresult, PhotonPipelineResult PVresultTwo){
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

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
