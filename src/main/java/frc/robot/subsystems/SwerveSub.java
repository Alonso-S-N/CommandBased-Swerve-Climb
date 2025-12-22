// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.nio.file.FileSystem;
import java.util.Optional;

import org.dyn4j.geometry.Rotation;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSub extends SubsystemBase {
  private final SwerveDrive swerve;
  
  private double tx,ty;
  private double MAX_SPEED = 3;
  private final NetworkTable limelight;

   /* ================== PHOTON ================== */
  private final PhotonCamera camera = new PhotonCamera("camera");
  private final PhotonPoseEstimator photonEstimator;

  private VisionSystemSim visionSim;

  public SwerveSub() {

    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    try{
      File json = new File(Filesystem.getDeployDirectory(), "yagsl");
      swerve = new SwerveParser(json).createSwerveDrive(3.0);
      RobotConfig config = RobotConfig.fromGUISettings();

        AutoBuilder.configure(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::getRobotVelocity,
            (speeds, feedforwards) -> swerve.setChassisSpeeds(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), 
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> DriverStation.isAutonomous(), 
            this
        );
    } catch(Exception e){
      throw new RuntimeException("Deu Ruim", e);
    }

        AprilTagFieldLayout layout =
        AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();

        photonEstimator = new PhotonPoseEstimator(
          layout,
    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    new Transform3d(
        new Translation3d(-0.2, 0.6, 0.6),
        new Rotation3d(0, Math.toRadians(25), 0)
    )
      );
      

    swerve.setVisionMeasurementStdDevs(
      VecBuilder.fill(
          0.5,   // X (metros)
          0.5,   // Y
          9999   // gyro manda
      )
  );
    /* ========== SIMULAÇÃO ========== */
    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("VisionSim");
      visionSim.addAprilTags(layout);

      SimCameraProperties props = new SimCameraProperties();
      props.setCalibration(1920, 1080, Rotation2d.fromDegrees(90));
      props.setFPS(30);

      PhotonCameraSim camSim = new PhotonCameraSim(camera, props);
      visionSim.addCamera(camSim, photonEstimator.getRobotToCameraTransform());
    }
  }
   public void drive (Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    if (swerve == null) return;
    swerve.drive(translation, rotation, fieldRelative, isOpenLoop);
   }

   public void drivePath(ChassisSpeeds speeds) {
    swerve.drive(speeds);
}

   public SwerveModuleState[] getModuleStates(){
    return swerve.getStates();
   }

   public double limelightAimProportional(){
  double kp = 0.1;
    return limelight.getEntry("tx").getDouble(0.0) * kp;
   }

   public double limelight_range_proportional()
   {    
   double kp = 0.4;
   return limelight.getEntry("ty").getDouble(0.0) * kp;
   }
   public boolean hasTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1;
  }


  public Optional<Pose2d> getLimelightPose() {
  if (limelight.getEntry("tv").getDouble(0) != 1) {
    return Optional.empty();
  }


  double[] pose = limelight
      .getEntry("botpose_wpiblue")
      .getDoubleArray(new double[6]);

  return Optional.of(
      new Pose2d(
          pose[0],
          pose[1],
          Rotation2d.fromDegrees(pose[5])
      )
  );
}

  

  @Override
  public void periodic() {
    swerve.updateOdometry();
  
     getLimelightPose().ifPresent(pose -> {
    swerve.addVisionMeasurement(
        pose,
        Timer.getFPGATimestamp()
    );
  });
  }

  @Override
  public void simulationPeriodic() {
    visionSim.update(getPose());
  }
  public Pose2d getPose() {
    return swerve.getPose();
  }

  public Rotation2d getHeading(){
    return swerve.getYaw();
  }
      public Optional<Pose2d> getVisionPose() {
        PhotonPipelineResult result = camera.getLatestResult();
    
        if (!result.hasTargets()) {
            return Optional.empty();
        }
    
        return photonEstimator
            .update(result)
            .map(pose -> pose.estimatedPose.toPose2d());
    }
    

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerve.setChassisSpeeds(speeds);
  }
}
