package frc.robot.commands.vision;

import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class PhotonDefault extends Command{
    private final Limelight vision;
    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d combinedPose = new Pose2d();
    private Pose2d cachedRobotPose = new Pose2d();
    private Rotation2d finalrotation = new Rotation2d();
    private LimelightHelpers.LimelightResults resultsOne;
    private LimelightHelpers.LimelightResults resultsTwo;
    private Pose3d targetPoseOne = new Pose3d();
    private Pose3d targetPoseTwo = new Pose3d();
    private Double[] targetPoseArrayDashboard = new Double[4];

    public PhotonDefault(Limelight camera, CommandSwerveDrivetrain drivetrain){
        this.vision = camera;
        this.drivetrain = drivetrain;
        addRequirements(camera);
    }

    @Override
    public void execute(){
        LimelightHelpers.PoseEstimate llResultOne = vision.getCamOneResult();
        LimelightHelpers.PoseEstimate llResultTwo = vision.getCamTwoResult();

        resultsOne = LimelightHelpers.getLatestResults(LimelightConstants.kLimelightOne);
        resultsTwo = LimelightHelpers.getLatestResults(LimelightConstants.kLimelightTwo);

       

        boolean hasTargetsOne = llResultOne != null && llResultOne.tagCount > 0;
        boolean hasTargetsTwo = llResultTwo != null && llResultTwo.tagCount > 0;

        SmartDashboard.putBoolean("pv one target", hasTargetsOne);
        SmartDashboard.putBoolean("pv two target", hasTargetsTwo);

        SmartDashboard.putBoolean("result null", llResultOne != null);
        SmartDashboard.putBoolean("result null 2", llResultTwo != null);

        // CAM ONE can see tag, CAM TWO cannot
        if(hasTargetsOne && !hasTargetsTwo) {
            processSingleCam(llResultOne);
            if(resultsOne.targets_Fiducials.length >0){

                LimelightHelpers.LimelightTarget_Fiducial[] target = resultsOne.targets_Fiducials;
                
                int id = (int) target[0].fiducialID;
        
                var tagposeoptional = vision.getLayout().getTagPose(id);
        
                if(tagposeoptional.isPresent()){
                    targetPoseOne = tagposeoptional.get();
        
                }
                targetPoseArrayDashboard[0] = targetPoseOne.getX(); // X translation
                targetPoseArrayDashboard[1] = targetPoseOne.getY();// Y translation
                targetPoseArrayDashboard[2] = targetPoseOne.getZ(); // Z translation
                targetPoseArrayDashboard[3] = targetPoseOne.getRotation().toRotation2d().getRadians();

                SmartDashboard.putNumberArray("target", targetPoseArrayDashboard);
            }
        }   
        
        // CAM TWO can see tag, CAM ONE cannot
        else if(!hasTargetsOne && hasTargetsTwo) {
            processSingleCam(llResultTwo);
        }
        
        //CAM TWO AND CAM ONE can see tags
        else if(hasTargetsOne && hasTargetsTwo){
            processDoubleCam(llResultOne, llResultTwo);
        }
    }




    public void processSingleCam(LimelightHelpers.PoseEstimate llResult){
        SmartDashboard.putBoolean("rawfid", llResult.rawFiducials !=null);

        if (llResult.rawFiducials == null || llResult.rawFiducials.length == 0) {
            return;
        }

       
        double totalArea = getTotalTagArea(llResult);

        SmartDashboard.putNumber("area", totalArea);

        Pose2d robotPose = llResult.pose;
        cachedRobotPose = drivetrain.getState().Pose;

        double translationalVelocity = Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
        double rotationalVelocity = Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        
        Matrix<N3, N1> stdDevs = calculateDynamicStdDevs(totalArea, translationalVelocity, rotationalVelocity);

        double poseDistance = cachedRobotPose.getTranslation().getDistance(robotPose.getTranslation());
        SmartDashboard.putNumber("distance", poseDistance);
        double rotationdiffrence = Math.abs(cachedRobotPose.getRotation().minus(robotPose.getRotation()).getDegrees());
        
        if( totalArea > LimelightConstants.kMinAreaOdom
        && totalArea < LimelightConstants.kMinAreaGyro 
        && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
        && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity
        && poseDistance < LimelightConstants.kMaxPoseDistance) {

            combinedPose = new Pose2d(
                robotPose.getX(),robotPose.getY(), cachedRobotPose.getRotation()
            ); 

            drivetrain.addVisionMeasurement(
                combinedPose,
                llResult.timestampSeconds,   
                stdDevs
                );
        
        }

        else if(totalArea > LimelightConstants.kMinAreaGyro
            && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
            && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity
            && poseDistance < LimelightConstants.kMaxPoseDistance
            && rotationdiffrence < LimelightConstants.kMaxRotationDifference){ 


            drivetrain.addVisionMeasurement(
                robotPose,
                llResult.timestampSeconds,
                stdDevs
                );
        
        }
    }
    
    





    public void processDoubleCam(LimelightHelpers.PoseEstimate llResult, LimelightHelpers.PoseEstimate llResultTwo){

        processSingleCam(llResult);
        processSingleCam(llResultTwo);

      /*   cachedRobotPose = drivetrain.getState().Pose;


        double areaOne = getTotalTagArea(llResult);
        double areaTwo = getTotalTagArea(llResultTwo);
        
        double translationalVelocity = Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
        double rotationalVelocity = Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond);

        double velocityPenalty = 1.0 - (translationalVelocity / LimelightConstants.kMaxTranslationalVelocity);
        velocityPenalty = Math.max(0.1, velocityPenalty);

        double confidenceOne = velocityPenalty * (areaOne );
        double confidenceTwo = velocityPenalty * (areaTwo);

        double totalConfidence = confidenceOne + confidenceTwo;

        if (totalConfidence == 0) {
            return;
        }
        
        double weightOne = confidenceOne / totalConfidence;
        double weightTwo = confidenceTwo / totalConfidence;

        Pose2d robotPoseOne = llResult.pose;
        Pose2d robotPoseTwo = llResultTwo.pose;


        boolean updategyroOne = areaOne > LimelightConstants.kMinAreaGyro;
        boolean updategyroTwo = areaTwo > LimelightConstants.kMinAreaGyro;

        

        
        //if the CAM ONE area is in the gyro update range but CAM TWO area is not, use CAM ONE for gyro
        if(updategyroOne && !(updategyroTwo)
        && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
        && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity){
            
            finalrotation = robotPoseOne.getRotation();
            
        }

        //if the CAM TWO area is in the gyro update range but CAM ONE area is not, use CAM TWO for gyro

        else if(!(updategyroOne) && updategyroTwo
        && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
        && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity){
            
            finalrotation = robotPoseTwo.getRotation();

        }

        //if the CAM TWO area is in the gyro update range AND CAM ONE area is in the gyro update range, combine the rotations for gyro


        else if(updategyroOne && updategyroTwo
        && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
        && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity){
            
            finalrotation = robotPoseOne.getRotation().interpolate(robotPoseTwo.getRotation(), weightTwo);

            

        } 
        
        
        //if the CAM TWO area is NOT in the gyro update range AND CAM ONE area is NOT 
        //in the gyro update range, dont update gyro

        
        else if (!updategyroOne && !updategyroTwo
        && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
        && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity){
            finalrotation = drivetrain.getState().RawHeading;
        }

        //if BOTH CAMS are NOT in the gyro update range AND velocities too high, do not take anything
        else{
            return;
        }

        combinedPose = new Pose2d(
            (robotPoseOne.getX() * weightOne) + (robotPoseTwo.getX() * weightTwo),
            (robotPoseOne.getY() * weightOne) + (robotPoseTwo.getY() * weightTwo),
            finalrotation
        );

        double poseDistance = cachedRobotPose.getTranslation().getDistance(combinedPose.getTranslation());
        double rotationdiffrence = Math.abs(cachedRobotPose.getRotation().minus(combinedPose.getRotation()).getDegrees());

        if (poseDistance > LimelightConstants.kMaxPoseDistance){
            return;
        }
        if(rotationdiffrence > LimelightConstants.kMaxRotationDifference){
            return;
        }
        
        drivetrain.addVisionMeasurement(
            combinedPose,
            (llResult.timestampSeconds + llResultTwo.timestampSeconds) / 2.0,
            LimelightConstants.kStdDevs
        ); */
    }

    

    @Override
    public boolean isFinished(){
        return false;
    }

    private double getTotalTagArea(LimelightHelpers.PoseEstimate llresult){
        if (llresult.rawFiducials == null || llresult.rawFiducials.length == 0) {
            return 0.0;
        }
        double totalArea = 0.0;

        for (LimelightHelpers.RawFiducial target : llresult.rawFiducials) {
            totalArea += target.ta;
        }
        return totalArea;
    }

     private edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> calculateDynamicStdDevs(
        double totalArea, double translationalVelocity, double rotationalVelocity) {
        
        double basexyStdDev = LimelightConstants.kXyStdDev;
        double basethetaStdDev = LimelightConstants.kThetaStdDev;

        double areafactor = 1.0;
        
        if (totalArea > LimelightConstants.kMinAreaGyro) {
            // Good area: scale between 0.1 and 0.5
            areafactor = 0.5 - (0.4 * Math.min(totalArea / 20.0, 1.0)); // Assumes max useful area ~20%
        } else if (totalArea > LimelightConstants.kMinAreaOdom) {
            // Medium area: scale between 0.5 and 1.0
            double areaRange = LimelightConstants.kMinAreaGyro - LimelightConstants.kMinAreaOdom;
            double areaNormalized = (totalArea - LimelightConstants.kMinAreaOdom) / areaRange;
            areafactor = 1.0 - (0.5 * areaNormalized);
        }

        double transVelFactor = 0.5 + (1.5 * (translationalVelocity / LimelightConstants.kMaxTranslationalVelocity));

        double rotVelFactor = 0.5 + (1.5 * (rotationalVelocity / LimelightConstants.kMaxRotationalVelocity));


        double xyStdDev = basexyStdDev * areafactor * transVelFactor;
        double thetaStdDev = basethetaStdDev * areafactor * rotVelFactor;

        return edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    } 
    
}
