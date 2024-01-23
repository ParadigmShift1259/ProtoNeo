// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand.h"

IntakeCommand::IntakeCommand(ExampleSubsystem* subsystem)
    : m_subsystem{subsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void IntakeCommand::Initialize()
{
  m_subsystem->RunIntake();
}

void IntakeCommand::Execute()
{
  
}

bool IntakeCommand::IsFinished()
{
  return false;
}

void IntakeCommand::End(bool interrupted)
{
  m_subsystem->StopIntake();
}
