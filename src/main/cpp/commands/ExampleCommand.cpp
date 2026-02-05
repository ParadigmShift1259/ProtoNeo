// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExampleCommand.h"

ExampleCommand::ExampleCommand(ExampleSubsystem* subsystem)
    : m_subsystem{subsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void ExampleCommand::Initialize()
{
}

void ExampleCommand::Execute()
{
  m_subsystem->RunMotors();
}

bool ExampleCommand::IsFinished()
{
  return false;
}

void ExampleCommand::End(bool interrupted)
{
  m_subsystem->StopMotors();
}
