// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExampleSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ExampleSubsystem::ExampleSubsystem() 
  : m_leadmotor(12, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_followmotor(13, rev::CANSparkLowLevel::MotorType::kBrushless)
{
//#define USE_FOLLOW
#ifdef USE_FOLLOW
  m_followmotor.Follow(m_leadmotor, true);
#else
  //m_followmotor.Follow(m_leadmotor, false);
  frc::SmartDashboard::PutNumber("diff factor", 0.6);
#endif
  frc::SmartDashboard::PutNumber("voltage", 0.0);
}

frc2::CommandPtr ExampleSubsystem::ExampleMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool ExampleSubsystem::ExampleCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void ExampleSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
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

void ExampleSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void ExampleSubsystem::RunMotors() {
  double voltage = frc::SmartDashboard::GetNumber("voltage", 0.0);
  m_leadmotor.SetVoltage(units::voltage::volt_t{voltage});
#ifndef USE_FOLLOW
  double diffFactor = frc::SmartDashboard::GetNumber("diff factor", 0.6);
  m_followmotor.SetVoltage(units::voltage::volt_t{voltage * -1.0 * diffFactor});
#endif
}

void ExampleSubsystem::StopMotors() {
  m_leadmotor.SetVoltage(units::voltage::volt_t{0.0});
#ifndef USE_FOLLOW
  m_followmotor.SetVoltage(units::voltage::volt_t{0.0});
#endif
}
