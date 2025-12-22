// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.SwerveSub;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;




public class Loc extends Command {
  private SwerveSub swerve;
  
      private PS5Controller ps5;
      private ClimbSub climbSub;
      private double y,x;
      private double MAX_SPEED = 3; // meters per second
      

      private final PIDController headingPID =
      new PIDController(4.0, 0.0, 0.2);
  
    public Loc(SwerveSub swerve, PS5Controller ps5) {
       this.swerve = swerve;
       this.ps5 = ps5;

       headingPID.enableContinuousInput(-Math.PI, Math.PI);
    
      addRequirements(swerve);
    }
  
    @Override
    public void initialize() {

  }

  public void MainControll(){
    this.y = applyDeadband(-ps5.getLeftY());
    this.x = applyDeadband(-ps5.getLeftX());
    double rot = applyDeadband(-ps5.getRightX());
    rot *= 2;

    SwerveModuleState[] states;
 
    swerve.drive( new Translation2d(y * MAX_SPEED,x * MAX_SPEED),rot,true,true);

    int pov = ps5.getPOV();

    if (pov != -1) {
      rotateToAngle(pov);
    }
  

    if (Robot.isSimulation()){
      Logger.recordOutput("Swerve/Pose:", swerve.getPose());
      Logger.recordOutput("Swerve/ModuleStates",swerve.getModuleStates());
    }
}
      
private void rotateToAngle(double targetDegrees) {
  Rotation2d current = swerve.getHeading();

  double rotOutput = headingPID.calculate(
      current.getRadians(),
      Math.toRadians(targetDegrees)
  );

  swerve.drive(
      new Translation2d(0, 0),
      rotOutput,
      true,
      true
  );
}

      
        @Override
  public void execute() {
    MainControll();
    SmartdashBoard();
  }

  public void POV0(){
    swerve.drive(new Translation2d(0, 0), 0, true, true);
  }

  public void POV90(){
    swerve.drive(new Translation2d(0, 0), 90, true, true);
  }

  public void POV180(){
    swerve.drive(new Translation2d(0, 0), 180, true, true);
  }

  public void POV270(){ 
    swerve.drive(new Translation2d(0,0), 270, isFinished(), isScheduled());
  }
    public void SmartdashBoard() {
    SmartDashboard.putNumber("X", x);
    SmartDashboard.putNumber("Y", y);
  }

  @Override
  public void end(boolean interrupted) {}

 
  @Override
  public boolean isFinished() {
    return false;
  }

  private double applyDeadband(double value) {
    return Math.abs(value) < 0.05 ? 0.0 : value;
}
}
