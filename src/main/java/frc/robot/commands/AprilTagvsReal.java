// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import org.photonvision.targeting.PhotonPipelineResult;

import java.sql.Array;
import java.util.ArrayList;
/** An example command that uses an example subsystem. */
public class AprilTagvsReal extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  ArrayList<Double> realPoses;
  ArrayList<Double> AprilTagPoses;
  public Chassis m_chassis;
  public Limelight m_limelight;
  Double time;
  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AprilTagvsReal( Chassis chassis, Limelight limelight) {
    m_chassis = chassis;
    m_limelight = limelight;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AprilTagPoses = new ArrayList<Double>();
    realPoses = new ArrayList<Double>();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult latestResult = m_limelight.getLatestResult();
    if (latestResult != null) {
      if (time != latestResult.getTimestampSeconds()) {
        realPoses.add(m_chassis.getPose2d().getX());
        AprilTagPoses.add(m_limelight.getX());
        time = latestResult.getTimestampSeconds();
      }
    }
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int i = 0;
    while (i < realPoses.size()) {
      System.out.print(realPoses.get(i));
      System.out.print(", ");
      System.out.print(AprilTagPoses.get(i));
      System.out.print("\n");
      i++;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
