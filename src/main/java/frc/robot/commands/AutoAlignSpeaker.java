package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;

public class AutoAlignSpeaker extends Command {

  private final SwerveSub swerve;

  private final PIDController xController = new PIDController(5, 0, 0);
  private final PIDController yController = new PIDController(5, 0, 0);
  private final PIDController thetaController = new PIDController(4.0, 0, 0);

  private static final double TAG_OFFSET_METERS = 0.20;

  public AutoAlignSpeaker(SwerveSub swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(0.20);
    yController.setTolerance(0.05);
    thetaController.setTolerance(Math.toRadians(2));
  }

  @Override
  public void execute() {

    Optional<Pose2d> tagPoseOpt = swerve.getVisionPose();
    if (tagPoseOpt.isEmpty()) {
      return;
    }
  
    Pose2d robotPose = swerve.getPose();
    Pose2d tagPose = tagPoseOpt.get();
  
    Transform2d error = robotPose.minus(tagPose);

    double yawError =
    tagPose.getRotation()
        .minus(robotPose.getRotation())
        .getRadians();

    double xSpeed = xController.calculate(error.getX(), TAG_OFFSET_METERS);
    double ySpeed = yController.calculate(error.getY(), 0.0);
    double rotSpeed = thetaController.calculate(yawError, 0.0);

    if (Math.abs(yawError) < Math.toRadians(2.0)) {
        rotSpeed = 0.0;
    }
  
   ySpeed *= -1;
   
    ChassisSpeeds speeds = new ChassisSpeeds(
        xSpeed,
        ySpeed,
        rotSpeed
    );
  
    swerve.setChassisSpeeds(speeds);
  }

  @Override
  public boolean isFinished() {
 return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(new ChassisSpeeds());
  }
}
