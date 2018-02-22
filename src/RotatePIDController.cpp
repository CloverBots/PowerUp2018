/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RotatePIDController.h"
#include <iostream>
#include <cmath>
#include <vector>

#include "HAL/HAL.h"
#include "Notifier.h"
#include "PIDOutput.h"
#include "PIDSource.h"

using namespace frc;


/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp     the proportional coefficient
 * @param Ki     the integral coefficient
 * @param Kd     the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
RotatePIDController::RotatePIDController(double Kp, double Ki, double Kd, PIDSource* source,
		PIDOutput* leftoutput, PIDOutput* leftoutput1, PIDOutput* leftoutput2,
		PIDOutput* rightoutput, PIDOutput* rightoutput1,
		PIDOutput* rightoutput2, double period)
    : RotatePIDController(Kp, Ki, Kd, 0.0, source, leftoutput, leftoutput1, leftoutput2, rightoutput, rightoutput1, rightoutput2, period) {}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp     the proportional coefficient
 * @param Ki     the integral coefficient
 * @param Kd     the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
RotatePIDController::RotatePIDController(double Kp, double Ki, double Kd, double Kf,
                             PIDSource* source, PIDOutput* leftoutput,
							 PIDOutput* leftoutput1, PIDOutput* leftoutput2,
							 PIDOutput* rightoutput, PIDOutput* rightoutput1,
							 PIDOutput* rightoutput2,
                             double period) {
  m_controlLoop = std::make_unique<Notifier>(&RotatePIDController::Calculate, this);

  m_P = Kp;
  m_I = Ki;
  m_D = Kd;
  m_F = Kf;

  m_pidInput = source;
  m_pidLeftOutput = leftoutput;
  m_pidLeftOutput1 = leftoutput1;
  m_pidLeftOutput2 = leftoutput2;
  m_pidRightOutput = rightoutput;
  m_pidRightOutput1 = rightoutput1;
  m_pidRightOutput2 = rightoutput2;
  m_period = period;

  m_controlLoop->StartPeriodic(m_period);
  m_setpointTimer.Start();

  static int instances = 0;
  instances++;
  HAL_Report(HALUsageReporting::kResourceType_PIDController, instances);
}

RotatePIDController::~RotatePIDController() {
  // forcefully stopping the notifier so the callback can successfully run.
  m_controlLoop->Stop();
}

/**
 * Read the input, calculate the output accordingly, and write to the output.
 * This should only be called by the Notifier.
 */
void RotatePIDController::Calculate() {
  bool enabled;
  PIDSource* pidInput;
  PIDOutput* pidLeftOutput;
  PIDOutput* pidLeftOutput1;
  PIDOutput* pidLeftOutput2;
  PIDOutput* pidRightOutput;
  PIDOutput* pidRightOutput1;
  PIDOutput* pidRightOutput2;

  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    pidInput = m_pidInput;
    pidLeftOutput = m_pidLeftOutput;
    pidLeftOutput1 = m_pidLeftOutput1;
    pidLeftOutput2 = m_pidLeftOutput2;
    pidRightOutput = m_pidRightOutput;
    pidRightOutput1 = m_pidRightOutput1;
    pidRightOutput2 = m_pidRightOutput2;
    enabled = m_enabled;
  }

  if (pidInput == nullptr) return;
  if (pidLeftOutput == nullptr) return;
  if (pidLeftOutput1 == nullptr) return;
  if (pidLeftOutput2 == nullptr) return;
  if (pidRightOutput == nullptr) return;
  if (pidRightOutput1 == nullptr) return;
  if (pidRightOutput2 == nullptr) return;

  if (enabled) {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    double input = pidInput->PIDGet();
    double result;
    PIDOutput* pidLeftOutput;
    PIDOutput* pidLeftOutput1;
    PIDOutput* pidLeftOutput2;
    PIDOutput* pidRightOutput;
    PIDOutput* pidRightOutput1;
    PIDOutput* pidRightOutput2;

    m_error = GetContinuousError(m_setpoint - input);

    if (m_pidInput->GetPIDSourceType() == PIDSourceType::kRate) {
      if (m_P != 0) {
        double potentialPGain = (m_totalError + m_error) * m_P;
        if (potentialPGain < m_maximumOutput) {
          if (potentialPGain > m_minimumOutput)
            m_totalError += m_error;
          else
            m_totalError = m_minimumOutput / m_P;
        } else {
          m_totalError = m_maximumOutput / m_P;
        }
      }

      m_result = m_D * m_error + m_P * m_totalError + CalculateFeedForward();
    } else {
      if (m_I != 0) {
        double potentialIGain = (m_totalError + m_error) * m_I;
        if (potentialIGain < m_maximumOutput) {
          if (potentialIGain > m_minimumOutput)
            m_totalError += m_error;
          else
            m_totalError = m_minimumOutput / m_I;
        } else {
          m_totalError = m_maximumOutput / m_I;
        }
      }

      m_result = m_P * m_error + m_I * m_totalError +
                 m_D * (m_error - m_prevError) + CalculateFeedForward();
    }
    m_prevError = m_error;

    if (m_result > m_maximumOutput)
      m_result = m_maximumOutput;
    else if (m_result < m_minimumOutput)
      m_result = m_minimumOutput;

    pidLeftOutput = m_pidLeftOutput;
    pidLeftOutput1 = m_pidLeftOutput1;
    pidLeftOutput2 = m_pidLeftOutput2;
    pidRightOutput = m_pidRightOutput;
    pidRightOutput1 = m_pidRightOutput1;
    pidRightOutput2 = m_pidRightOutput2;
    result = m_result;

    pidLeftOutput->PIDWrite(result);
	pidLeftOutput1->PIDWrite(result);
	pidLeftOutput2->PIDWrite(result);
    pidRightOutput->PIDWrite(-result);
	pidRightOutput1->PIDWrite(-result);
	pidRightOutput2->PIDWrite(-result);
	//std::cout << result << std::endl;
    // Update the buffer.
    m_buf.push(m_error);
    m_bufTotal += m_error;
    // Remove old elements when buffer is full.
    if (m_buf.size() > m_bufLength) {
      m_bufTotal -= m_buf.front();
      m_buf.pop();
    }
  }
}

/**
 * Calculate the feed forward term.
 *
 * Both of the provided feed forward calculations are velocity feed forwards.
 * If a different feed forward calculation is desired, the user can override
 * this function and provide his or her own. This function does no
 * synchronization because the PIDController class only calls it in synchronized
 * code, so be careful if calling it oneself.
 *
 * If a velocity PID controller is being used, the F term should be set to 1
 * over the maximum setpoint for the output. If a position PID controller is
 * being used, the F term should be set to 1 over the maximum speed for the
 * output measured in setpoint units per this controller's update period (see
 * the default period in this class's constructor).
 */
double RotatePIDController::CalculateFeedForward() {
  if (m_pidInput->GetPIDSourceType() == PIDSourceType::kRate) {
    return m_F * GetSetpoint();
  } else {
    double temp = m_F * GetDeltaSetpoint();
    m_prevSetpoint = m_setpoint;
    m_setpointTimer.Reset();
    return temp;
  }
}

/**
 * Set the PID Controller gain parameters.
 *
 * Set the proportional, integral, and differential coefficients.
 *
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 */
void RotatePIDController::SetPID(double p, double i, double d) {
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    m_P = p;
    m_I = i;
    m_D = d;
  }
}

/**
 * Set the PID Controller gain parameters.
 *
 * Set the proportional, integral, and differential coefficients.
 *
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 * @param f Feed forward coefficient
 */
void RotatePIDController::SetPID(double p, double i, double d, double f) {
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    m_P = p;
    m_I = i;
    m_D = d;
    m_F = f;
  }

}

/**
 * Get the Proportional coefficient.
 *
 * @return proportional coefficient
 */
double RotatePIDController::GetP() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_P;
}

/**
 * Get the Integral coefficient.
 *
 * @return integral coefficient
 */
double RotatePIDController::GetI() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_I;
}

/**
 * Get the Differential coefficient.
 *
 * @return differential coefficient
 */
double RotatePIDController::GetD() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_D;
}

/**
 * Get the Feed forward coefficient.
 *
 * @return Feed forward coefficient
 */
double RotatePIDController::GetF() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_F;
}

/**
 * Return the current PID result.
 *
 * This is always centered on zero and constrained the the max and min outs.
 *
 * @return the latest calculated output
 */
double RotatePIDController::Get() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_result;
}

/**
 * Set the PID controller to consider the input to be continuous,
 *
 * Rather then using the max and min in as constraints, it considers them to
 * be the same point and automatically calculates the shortest route to
 * the setpoint.
 *
 * @param continuous true turns on continuous, false turns off continuous
 */
void RotatePIDController::SetContinuous(bool continuous) {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_continuous = continuous;
}

/**
 * Sets the maximum and minimum values expected from the input.
 *
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the output
 */
void RotatePIDController::SetInputRange(double minimumInput, double maximumInput) {
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
  }

  SetSetpoint(m_setpoint);
}

/**
 * Sets the minimum and maximum values to write.
 *
 * @param minimumOutput the minimum value to write to the output
 * @param maximumOutput the maximum value to write to the output
 */
void RotatePIDController::SetOutputRange(double minimumOutput, double maximumOutput) {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_minimumOutput = minimumOutput;
  m_maximumOutput = maximumOutput;
}

/**
 * Set the setpoint for the PIDController.
 *
 * Clears the queue for GetAvgError().
 *
 * @param setpoint the desired setpoint
 */
void RotatePIDController::SetSetpoint(double setpoint) {
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);

    if (m_maximumInput > m_minimumInput) {
      if (setpoint > m_maximumInput)
        m_setpoint = m_maximumInput;
      else if (setpoint < m_minimumInput)
        m_setpoint = m_minimumInput;
      else
        m_setpoint = setpoint;
    } else {
      m_setpoint = setpoint;
    }

    // Clear m_buf.
    m_buf = std::queue<double>();
    m_bufTotal = 0;
  }
}

/**
 * Returns the current setpoint of the PIDController.
 *
 * @return the current setpoint
 */
double RotatePIDController::GetSetpoint() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_setpoint;
}

/**
 * Returns the change in setpoint over time of the PIDController.
 *
 * @return the change in setpoint over time
 */
double RotatePIDController::GetDeltaSetpoint() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return (m_setpoint - m_prevSetpoint) / m_setpointTimer.Get();
}

/**
 * Returns the current difference of the input from the setpoint.
 *
 * @return the current error
 */
double RotatePIDController::GetError() const {
  double setpoint = GetSetpoint();
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    return GetContinuousError(setpoint - m_pidInput->PIDGet());
  }
}

/**
 * Sets what type of input the PID controller will use.
 */
void RotatePIDController::SetPIDSourceType(PIDSourceType pidSource) {
  m_pidInput->SetPIDSourceType(pidSource);
}
/**
 * Returns the type of input the PID controller is using.
 *
 * @return the PID controller input type
 */
PIDSourceType RotatePIDController::GetPIDSourceType() const {
  return m_pidInput->GetPIDSourceType();
}

/**
 * Returns the current average of the error over the past few iterations.
 *
 * You can specify the number of iterations to average with SetToleranceBuffer()
 * (defaults to 1). This is the same value that is used for OnTarget().
 *
 * @return the average error
 */
double RotatePIDController::GetAvgError() const {
  double avgError = 0;
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    // Don't divide by zero.
    if (m_buf.size()) avgError = m_bufTotal / m_buf.size();
  }
  return avgError;
}

/*
 * Set the percentage error which is considered tolerable for use with
 * OnTarget.
 *
 * @param percentage error which is tolerable
 */
void RotatePIDController::SetTolerance(double percent) {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_toleranceType = kPercentTolerance;
  m_tolerance = percent;
}

/*
 * Set the absolute error which is considered tolerable for use with
 * OnTarget.
 *
 * @param percentage error which is tolerable
 */
void RotatePIDController::SetAbsoluteTolerance(double absTolerance) {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_toleranceType = kAbsoluteTolerance;
  m_tolerance = absTolerance;
}

/*
 * Set the percentage error which is considered tolerable for use with
 * OnTarget.
 *
 * @param percentage error which is tolerable
 */
void RotatePIDController::SetPercentTolerance(double percent) {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_toleranceType = kPercentTolerance;
  m_tolerance = percent;
}

/*
 * Set the number of previous error samples to average for tolerancing. When
 * determining whether a mechanism is on target, the user may want to use a
 * rolling average of previous measurements instead of a precise position or
 * velocity. This is useful for noisy sensors which return a few erroneous
 * measurements when the mechanism is on target. However, the mechanism will
 * not register as on target for at least the specified bufLength cycles.
 *
 * @param bufLength Number of previous cycles to average. Defaults to 1.
 */
void RotatePIDController::SetToleranceBuffer(int bufLength) {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_bufLength = bufLength;

  // Cut the buffer down to size if needed.
  while (m_buf.size() > static_cast<uint32_t>(bufLength)) {
    m_bufTotal -= m_buf.front();
    m_buf.pop();
  }
}

/*
 * Return true if the error is within the percentage of the total input range,
 * determined by SetTolerance. This asssumes that the maximum and minimum input
 * were set using SetInput.
 *
 * Currently this just reports on target as the actual value passes through the
 * setpoint. Ideally it should be based on being within the tolerance for some
 * period of time.
 *
 * This will return false until at least one input value has been computed.
 */
bool RotatePIDController::OnTarget() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  if (m_buf.size() == 0) return false;
  double error = GetAvgError();
  switch (m_toleranceType) {
    case kPercentTolerance:
      return std::fabs(error) <
             m_tolerance / 100 * (m_maximumInput - m_minimumInput);
      break;
    case kAbsoluteTolerance:
      return std::fabs(error) < m_tolerance;
      break;
    case kNoTolerance:
      return false;
  }
  return false;
}

/**
 * Begin running the PIDController.
 */
void RotatePIDController::Enable() {
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    m_enabled = true;
  }
}

/**
 * Stop running the PIDController, this sets the output to zero before stopping.
 */
void RotatePIDController::Disable() {
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    m_pidLeftOutput->PIDWrite(0);
	m_pidLeftOutput1->PIDWrite(0);
	m_pidLeftOutput2->PIDWrite(0);
    m_pidRightOutput->PIDWrite(0);
	m_pidRightOutput1->PIDWrite(0);
	m_pidRightOutput2->PIDWrite(0);

    m_enabled = false;
  }

}

/**
 * Return true if PIDController is enabled.
 */
bool RotatePIDController::IsEnabled() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_enabled;
}

/**
 * Reset the previous error, the integral term, and disable the controller.
 */
void RotatePIDController::Reset() {
  Disable();

  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_prevError = 0;
  m_totalError = 0;
  m_result = 0;
}


/**
 * Wraps error around for continuous inputs. The original error is returned if
 * continuous mode is disabled. This is an unsynchronized function.
 *
 * @param error The current error of the PID controller.
 * @return Error for continuous inputs.
 */
double RotatePIDController::GetContinuousError(double error) const {
  if (m_continuous) {
    if (std::fabs(error) > (m_maximumInput - m_minimumInput) / 2) {
      if (error > 0) {
        return error - (m_maximumInput - m_minimumInput);
      } else {
        return error + (m_maximumInput - m_minimumInput);
      }
    }
  }

  return error;
}
