// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//#include <frc/motorcontrol/Spark.h> // For intake
#include <frc/Timer.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

//#define USE_SPARKMAX
#define USE_SPARKFLEX
#ifdef USE_SPARKMAX
#include <rev/SparkMax.h>
#endif

#ifdef USE_SPARKFLEX
#include <rev/SparkFlex.h>
#endif

class ExampleSubsystem : public frc2::SubsystemBase {
 public:
  ExampleSubsystem();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr ExampleMethodCommand();

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool ExampleCondition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;
  void RunMotors();
  void StopMotors();
  void RunIntake();
  void StopIntake();

  void SpeedAdj(double adjAmt)
  {
      double voltage = frc::SmartDashboard::GetNumber("voltage", 5.0);
      voltage += adjAmt;
      if (voltage <= 12.0 && voltage >= -12.0)
      {
        frc::SmartDashboard::PutNumber("voltage", voltage);
      }
  }

 private:
#ifdef USE_SPARKMAX
  rev::spark::SparkMax m_leadmotor;
  rev::spark::SparkRelativeEncoder m_leadEnc = m_leadmotor.GetEncoder();
#endif

#ifdef USE_SPARKFLEX
  rev::spark::SparkFlex m_leadmotor;
  rev::spark::SparkRelativeEncoder m_leadEnc = m_leadmotor.GetEncoder();
#endif

#ifdef TWO_MOTORS
  rev::spark::SparkMax m_followmotor;
  rev::spark::SparkRelativeEncoder m_followEnc = m_followmotor.GetEncoder();
#endif

  //frc::Spark m_intakeMotor;

  frc::Timer m_timer;
  bool m_timerStarted = false;
  bool m_motorStarted = false;
};
