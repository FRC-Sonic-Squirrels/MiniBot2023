/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWMPorts;


public class blinkinSubsystem extends SubsystemBase {

  /*
   * Rev Robotics blinkinSubsystem takes a PWM signal from 1000-2000us This is identical to
   * a SparkMax motor. -1 corresponds to 1000us 0 corresponds to 1500us +1
   * corresponds to 2000us
   */
  private static Spark m_blinkinSubsystem = null;

  /**
   * Creates a new blinkinSubsystem LED controller.
   * 
   * @param pwmPort The PWM port the blinkinSubsystem is connected to.
   */
  public blinkinSubsystem() {
    m_blinkinSubsystem = new Spark(PWMPorts.kBlinkin);
  }

  public void set(final double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkinSubsystem.set(val);
    }
  }

  public void solidBlue() {
    set(0.87);
  }

  public void solidRed() {
    set(0.61);
  }

  public void flashingBlue() {
    set(-0.23);
  }

  public void strobeBlue() {
    set(-0.09);
  }

  public void solidOrange() {
    set(0.63);
  }
  public void off() {
    set(0);
  }

  public Command solidBlueCommand(){
    return new InstantCommand(
 ()->{solidBlue();});
  }
  public Command solidRedCommand(){
    return new InstantCommand(
 ()->{solidRed();});
  }
  public Command flashingBlueCommand(){
    return new InstantCommand(
 ()->{flashingBlue();});
  }
  public Command strobeBlueCommand(){
    return new InstantCommand(
 ()->{strobeBlue();});
  }
  public Command solidOrangeCommand(){
    return new InstantCommand(
 ()->{solidOrange();});
  }
  public Command offCommand(){
    return new InstantCommand(
 ()->{off();});
  }
  
}
