// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExampleSubsystem.h"

#ifdef USE_SPARKMAX
#include <rev/config/SparkMaxConfig.h>
#endif

#ifdef USE_SPARKFLEX
#include <rev/config/SparkFlexConfig.h>
#endif

ExampleSubsystem::ExampleSubsystem() 
#ifdef USE_SPARKMAX
  : m_leadmotor(12, rev::spark::SparkMax::MotorType::kBrushless)
#endif

#ifdef USE_SPARKFLEX
  : m_leadmotor(12, rev::spark::SparkFlex::MotorType::kBrushless)
#endif
#ifdef TWO_MOTORS
  , m_followmotor(13, rev::spark::SparkMax::MotorType::kBrushless)
#endif
  //, m_intakeMotor(1)
{
//#define USE_FOLLOW
#ifdef USE_FOLLOW
  //m_followmotor.Follow(m_leadmotor, true);
  m_followmotor.Follow(m_leadmotor, false);
#else
    //m_followmotor.RestoreFactoryDefaults();
  //m_followmotor.Follow(rev::CANSparkBase::kFollowerDisabled);
  frc::SmartDashboard::PutNumber("diff factor", 1.0);
#endif

  m_leadmotor.ClearFaults();
#ifdef TWO_MOTORS
  m_followmotor.ClearFaults();
#endif

#ifdef USE_SPARKMAX
  rev::spark::SparkMaxConfig config;
#endif

#ifdef USE_SPARKFLEX
  rev::spark::SparkFlexConfig config;
#endif

  config.encoder.PositionConversionFactor(1)
                .VelocityConversionFactor(1);

  config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast)
      .Inverted(false)
      .VoltageCompensation(12.0)
      .closedLoop
        .OutputRange(-1.0, 1.0)
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .P(0.1)
        .I(0)
        .D(0)
        .OutputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .P(0.0001, rev::spark::ClosedLoopSlot::kSlot1)
        .I(0, rev::spark::ClosedLoopSlot::kSlot1)
        .D(0, rev::spark::ClosedLoopSlot::kSlot1);        
  m_leadmotor.Configure(config, rev::ResetMode::kResetSafeParameters,rev::PersistMode::kPersistParameters);

#ifdef TWO_MOTORS
  config.Inverted(true);
  m_followmotor.Configure(config, rev::ResetMode::kResetSafeParameters,rev::PersistMode::kPersistParameters);
#endif

  //std::vector< uint8_t > 	 GetSerialNumber ()

  //???m_intakeMotor.SetInveted(true);

  frc::SmartDashboard::PutNumber("voltage", 5);
  frc::SmartDashboard::PutNumber("intake level", -0.76);
}

frc2::CommandPtr ExampleSubsystem::ExampleMethodCommand()
{
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool ExampleSubsystem::ExampleCondition()
{
  // Query some boolean state, such as a digital sensor.
  return false;
}

void ExampleSubsystem::Periodic()
{
  
  if (m_timerStarted == false && m_motorStarted == true)
  {
    m_timerStarted = true;
    m_timer.Reset();
    m_timer.Start();
  }
  // else if (m_timer.Get() > 4.0_s)
  // {
  //   m_intakeMotor.Set(0);
  // }
  // else if (m_timer.Get() >= 3.0_s)
  // {
  //   double intakeLevel = frc::SmartDashboard::GetNumber("intake level", -0.76);
  //   m_intakeMotor.Set(intakeLevel);
  // }

  frc::SmartDashboard::PutNumber("Lead RPM", m_leadEnc.GetVelocity());
  frc::SmartDashboard::PutNumber("Lead Ring RPM", m_leadEnc.GetVelocity() / 1.5);  // Gear ratio 1.5
  frc::SmartDashboard::PutNumber("Lead Bus Voltage", m_leadmotor.GetBusVoltage());
  frc::SmartDashboard::PutNumber("Lead Appl Out", m_leadmotor.GetAppliedOutput());
  frc::SmartDashboard::PutNumber("Lead Out Current", m_leadmotor.GetOutputCurrent());

#ifdef TWO_MOTORS
  frc::SmartDashboard::PutNumber("Folow RPM", m_followEnc.GetVelocity());
  frc::SmartDashboard::PutNumber("Folow Ring RPM", m_followEnc.GetVelocity() / 1.5);  // Gear ratio 1.5
  frc::SmartDashboard::PutNumber("Follow Bus Voltage", m_followmotor.GetBusVoltage());
  frc::SmartDashboard::PutNumber("Follow Appl Out", m_followmotor.GetAppliedOutput());
  frc::SmartDashboard::PutNumber("Follow Out Current", m_followmotor.GetOutputCurrent());
#endif
}

void ExampleSubsystem::SimulationPeriodic()
{
  // Implementation of subsystem simulation periodic method goes here.
}

void ExampleSubsystem::RunMotors()
{
  double voltage = frc::SmartDashboard::GetNumber("voltage", 5);
  m_leadmotor.SetVoltage(units::voltage::volt_t{voltage});

  m_motorStarted = true;

#ifdef TWO_MOTORS
#ifndef USE_FOLLOW
  double diffFactor = frc::SmartDashboard::GetNumber("diff factor", 1.0);
  m_followmotor.SetVoltage(units::voltage::volt_t{voltage * diffFactor});
#endif
#endif
}

void ExampleSubsystem::StopMotors()
{
  m_leadmotor.SetVoltage(units::voltage::volt_t{0.0});

#ifdef TWO_MOTORS
#ifndef USE_FOLLOW
  m_followmotor.SetVoltage(units::voltage::volt_t{0.0});
#endif
#endif
  //m_intakeMotor.Set(0.0);
  m_timerStarted = false;
  m_motorStarted = false;
}

void ExampleSubsystem::RunIntake()
{
  double intakeLevel = frc::SmartDashboard::GetNumber("intake level", -0.76);
  //m_intakeMotor.Set(intakeLevel);
}

void ExampleSubsystem::StopIntake()
{
  //m_intakeMotor.Set(0.0);
}

