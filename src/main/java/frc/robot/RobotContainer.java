// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autonomous;
import frc.robot.commands.AutoAlignSpeaker;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.Loc;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.SwerveSub;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;


public class RobotContainer {

 private final SwerveSub swerve = new SwerveSub();
 private final Loc loc;
 private final PS5Controller ps5 = new PS5Controller(0);
 private final ClimbSub ClimbSub = new ClimbSub();

private final SendableChooser<Command> autoChooser;
private final AutoAlignSpeaker autoAlignSpeaker =
    new AutoAlignSpeaker(swerve);
    
  public RobotContainer() {

    NamedCommands.registerCommand("Tests", Commands.runOnce(() -> System.out.println("Funfou")));//new ClimbCommand(ClimbSub, 0.5)));

     autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

         FollowPathCommand.warmupCommand().schedule();
         
    
    loc = new Loc(swerve,ps5);
    swerve.setDefaultCommand(loc);

    configurationBindings();
  }

  public void configurationBindings() {
   
    new Trigger(ps5::getR2Button)
    .whileTrue(
        Commands.startEnd(
            () -> ClimbSub.setMotor(-0.5),
            () -> ClimbSub.STOP(),
            ClimbSub
        )
    );
new Trigger(ps5::getL2Button)
.whileTrue(
  Commands.startEnd(
      () -> ClimbSub.setMotor(0.5),
      () -> ClimbSub.STOP(),
      ClimbSub
  )
);

new Trigger(ps5::getCircleButton)
.onTrue(new ClimbCommand(ClimbSub, 0.50));

new Trigger(ps5::getCrossButton)
.onTrue(new ClimbCommand(ClimbSub, 0.0));

new Trigger(ps5::getTriangleButton)
.whileTrue(autoAlignSpeaker);
  }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
}
}
