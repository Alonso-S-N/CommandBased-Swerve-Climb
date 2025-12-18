// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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


public class Loc extends Command {
  private SwerveSub swerve;
  
      private PS5Controller ps5;
      private ClimbSub climbSub;
      private AutoAlignSpeaker autoAlignCommand;
      private double y,x;
      private double MAX_SPEED = 3; // meters per second
  
    public Loc(SwerveSub swerve, PS5Controller ps5) {
       this.swerve = swerve;
       this.ps5 = ps5;
       this.autoAlignCommand = autoAlignCommand;
    
      addRequirements(swerve);
    }
  
    @Override
    public void initialize() {

  }

  public void MainControll(){
    this.y = applyDeadband(-ps5.getLeftY());
    this.x = applyDeadband(-ps5.getLeftX());
    double rot = applyDeadband(-ps5.getRightX());

    if (CommandScheduler.getInstance().isScheduled(autoAlignCommand)) {
      return;
    }

    SwerveModuleState[] states;
 
    swerve.drive( new Translation2d(y * MAX_SPEED,x * MAX_SPEED),rot,true,true);
     
    if (ps5.getTriangleButton() && swerve.hasTarget()) {

      double rotLL = swerve.limelightAimProportional();
      double fwdLL = swerve.limelight_range_proportional();
    
      Translation2d translation =
          new Translation2d(fwdLL * MAX_SPEED, 0);
    
      swerve.drive(
          translation,
          rotLL * MAX_SPEED,
          false,   // field-relative OFF
          true
      );
    }
}
      
      
        @Override
  public void execute() {
    MainControll();
    SmartdashBoard();
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
