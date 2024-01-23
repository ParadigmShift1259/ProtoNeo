// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExampleSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ExampleSubsystem::ExampleSubsystem() 
  : m_leadmotor(12, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_followmotor(13, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_intakeMotor(1)
{
//#define USE_FOLLOW
#ifdef USE_FOLLOW
  //m_followmotor.Follow(m_leadmotor, true);
  m_followmotor.Follow(m_leadmotor, false);
#else
    //m_followmotor.RestoreFactoryDefaults();
  //m_followmotor.Follow(rev::CANSparkBase::kFollowerDisabled);
  //frc::SmartDashboard::PutNumber("diff factor", 0.6);
  frc::SmartDashboard::PutNumber("diff factor", 0.95);
#endif

  m_leadmotor.ClearFaults();
  m_followmotor.ClearFaults();

  m_leadmotor.EnableVoltageCompensation(12.0);
  m_followmotor.EnableVoltageCompensation(12.0);

  //std::vector< uint8_t > 	 GetSerialNumber ()

  //???m_intakeMotor.SetInveted(true);

  frc::SmartDashboard::PutNumber("voltage", -5);
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
  frc::SmartDashboard::PutNumber("Lead RPM", m_leadEnc.GetVelocity());
  frc::SmartDashboard::PutNumber("Folow RPM", m_followEnc.GetVelocity());
  
  frc::SmartDashboard::PutNumber("Lead Ring RPM", m_leadEnc.GetVelocity() / 1.5);  // Gear ratio 1.5
  frc::SmartDashboard::PutNumber("Folow Ring RPM", m_followEnc.GetVelocity() / 1.5);  // Gear ratio 1.5

  frc::SmartDashboard::PutNumber("Lead Bus Voltage", m_leadmotor.GetBusVoltage());
  frc::SmartDashboard::PutNumber("Follow Bus Voltage", m_followmotor.GetBusVoltage());
  
  frc::SmartDashboard::PutNumber("Lead Appl Out", m_leadmotor.GetAppliedOutput());
  frc::SmartDashboard::PutNumber("Follow Appl Out", m_followmotor.GetAppliedOutput());
  
  frc::SmartDashboard::PutNumber("Lead Out Current", m_leadmotor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Follow Out Current", m_followmotor.GetOutputCurrent());
}

void ExampleSubsystem::SimulationPeriodic()
{
  // Implementation of subsystem simulation periodic method goes here.
}

void ExampleSubsystem::RunMotors()
{
  //double voltage = frc::SmartDashboard::GetNumber("voltage", 10.8);
  double voltage = frc::SmartDashboard::GetNumber("voltage", -5);
  m_leadmotor.SetVoltage(units::voltage::volt_t{voltage});

#ifndef USE_FOLLOW
  double diffFactor = frc::SmartDashboard::GetNumber("diff factor", 0.95);
  m_followmotor.SetVoltage(units::voltage::volt_t{voltage * -1.0 * diffFactor});
#endif
  double intakeLevel = frc::SmartDashboard::GetNumber("intake level", -0.76);
  m_intakeMotor.Set(intakeLevel);
}

void ExampleSubsystem::StopMotors()
{
  m_leadmotor.SetVoltage(units::voltage::volt_t{0.0});

#ifndef USE_FOLLOW
  m_followmotor.SetVoltage(units::voltage::volt_t{0.0});
#endif
  m_intakeMotor.Set(0.0);
}

void ExampleSubsystem::RunIntake()
{
  double intakeLevel = frc::SmartDashboard::GetNumber("intake level", -0.76);
  m_intakeMotor.Set(intakeLevel);
}

void ExampleSubsystem::StopIntake()
{
  m_intakeMotor.Set(0.0);
}

