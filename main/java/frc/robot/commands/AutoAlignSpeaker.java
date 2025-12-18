package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;

public class AutoAlignSpeaker extends Command {

  private final SwerveSub swerve;

  // PID controllers
  private final PIDController xController =
      new PIDController(2.5, 0.0, 0.0);

  private final PIDController yController =
      new PIDController(2.5, 0.0, 0.0);

  private final PIDController thetaController =
      new PIDController(4.0, 0.0, 0.0);

  // Distância desejada da tag (20 cm)
  private static final double TAG_OFFSET_METERS = 0.20;

  public AutoAlignSpeaker(SwerveSub swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    // Configurações dos PIDs
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(0.20); // 20 cm
    yController.setTolerance(0.05); // 5 cm
    thetaController.setTolerance(Math.toRadians(2)); // 2 graus
  }

  @Override
  public void execute() {

    Optional<Pose2d> tagPoseOpt = swerve.getLimelightPose();
    if (tagPoseOpt.isEmpty()) {
      swerve.setChassisSpeeds(new ChassisSpeeds());
      return;
    }

    Pose2d tagPose = tagPoseOpt.get();

    Pose2d targetPose =
        tagPose.transformBy(
            new Transform2d(
                new Translation2d(-TAG_OFFSET_METERS, 0.0),
                new Rotation2d()
            )
        );

    Pose2d currentPose = swerve.getPose();

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xController.calculate(
                currentPose.getX(),
                targetPose.getX()
            ),
            yController.calculate(
                currentPose.getY(),
                targetPose.getY()
            ),
            thetaController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
            ),
            currentPose.getRotation()
        );

    swerve.setChassisSpeeds(speeds);
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint()
        && yController.atSetpoint()
        && thetaController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    // Para o robô ao finalizar ou ser interrompido
    swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }
}
