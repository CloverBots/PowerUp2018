/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Motor3PIDController.h"

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
Motor3PIDController::Motor3PIDController(double Kp, double Ki, double Kd, PIDSource* source,
                             PIDOutput* output, PIDOutput* output1, PIDOutput* output2, double period)
    : Motor3PIDController(Kp, Ki, Kd, 0.0, source, output, output1, output2, period) {}

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
Motor3PIDController::Motor3PIDController(double Kp, double Ki, double Kd, double Kf,
                             PIDSource* source, PIDOutput* output, 
							 PIDOutput* output1, PIDOutput* output2, 
                             double period) {
  m_controlLoop = std::make_unique<Notifier>(&Motor3PIDController::Calculate, this);

  m_P = Kp;
  m_I = Ki;
  m_D = Kd;
  m_F = Kf;

  m_pidInput = source;
  m_pidOutput = output;
  m_pidOutput1 = output1;
  m_pidOutput2 = output2;
  m_period = period;

  m_controlLoop->StartPeriodic(m_period);
  m_setpointTimer.Start();

  static int instances = 0;
  instances++;
  HAL_Report(HALUsageReporting::kResourceType_PIDController, instances);
}

Motor3PIDController::~Motor3PIDController() {
  // forcefully stopping the notifier so the callback can successfully run.
  m_controlLoop->Stop();
}

/**
 * Read the input, calculate the output accordingly, and write to the output.
 * This should only be called by the Notifier.
 */
void Motor3PIDController::Calculate() {
  bool enabled;
  PIDSource* pidInput;
  PIDOutput* pidOutput;
  PIDOutput* pidOutput1;
  PIDOutput* pidOutput2;

  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    pidInput = m_pidInput;
    pidOutput = m_pidOutput;
    pidOutput1 = m_pidOutput1;
    pidOutput2 = m_pidOutput2;
    enabled = m_enabled;
  }

  if (pidInput == nullptr) return;
  if (pidOutput == nullptr) return;
  if (pidOutput1 == nullptr) return;
  if (pidOutput2 == nullptr) return;

  if (enabled) {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    double input = pidInput->PIDGet();
    double result;
    PIDOutput* pidOutput;
	PIDOutput* pidOutput1;
	PIDOutput* pidOutput2;

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

    pidOutput = m_pidOutput;
    pidOutput1 = m_pidOutput1;
    pidOutput2 = m_pidOutput2;
    result = m_result;

    pidOutput->PIDWrite(result);
	pidOutput1->PIDWrite(result);
	pidOutput2->PIDWrite(result);
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
double Motor3PIDController::CalculateFeedForward() {
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
void Motor3PIDController::SetPID(double p, double i, double d) {
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
void Motor3PIDController::SetPID(double p, double i, double d, double f) {
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
double Motor3PIDController::GetP() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_P;
}

/**
 * Get the Integral coefficient.
 *
 * @return integral coefficient
 */
double Motor3PIDController::GetI() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_I;
}

/**
 * Get the Differential coefficient.
 *
 * @return differential coefficient
 */
double Motor3PIDController::GetD() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_D;
}

/**
 * Get the Feed forward coefficient.
 *
 * @return Feed forward coefficient
 */
double Motor3PIDController::GetF() const {
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
double Motor3PIDController::Get() const {
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
void Motor3PIDController::SetContinuous(bool continuous) {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_continuous = continuous;
}

/**
 * Sets the maximum and minimum values expected from the input.
 *
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the output
 */
void Motor3PIDController::SetInputRange(double minimumInput, double maximumInput) {
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
void Motor3PIDController::SetOutputRange(double minimumOutput, double maximumOutput) {
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
void Motor3PIDController::SetSetpoint(double setpoint) {
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
double Motor3PIDController::GetSetpoint() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_setpoint;
}

/**
 * Returns the change in setpoint over time of the PIDController.
 *
 * @return the change in setpoint over time
 */
double Motor3PIDController::GetDeltaSetpoint() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return (m_setpoint - m_prevSetpoint) / m_setpointTimer.Get();
}

/**
 * Returns the current difference of the input from the setpoint.
 *
 * @return the current error
 */
double Motor3PIDController::GetError() const {
  double setpoint = GetSetpoint();
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    return GetContinuousError(setpoint - m_pidInput->PIDGet());
  }
}

/**
 * Sets what type of input the PID controller will use.
 */
void Motor3PIDController::SetPIDSourceType(PIDSourceType pidSource) {
  m_pidInput->SetPIDSourceType(pidSource);
}
/**
 * Returns the type of input the PID controller is using.
 *
 * @return the PID controller input type
 */
PIDSourceType Motor3PIDController::GetPIDSourceType() const {
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
double Motor3PIDController::GetAvgError() const {
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
void Motor3PIDController::SetTolerance(double percent) {
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
void Motor3PIDController::SetAbsoluteTolerance(double absTolerance) {
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
void Motor3PIDController::SetPercentTolerance(double percent) {
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
void Motor3PIDController::SetToleranceBuffer(int bufLength) {
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
bool Motor3PIDController::OnTarget() const {
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
void Motor3PIDController::Enable() {
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    m_enabled = true;
  }
}

/**
 * Stop running the PIDController, this sets the output to zero before stopping.
 */
void Motor3PIDController::Disable() {
  {
    std::lock_guard<priority_recursive_mutex> sync(m_mutex);
    m_pidOutput->PIDWrite(0);
    m_enabled = false;
  }

}

/**
 * Return true if PIDController is enabled.
 */
bool Motor3PIDController::IsEnabled() const {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  return m_enabled;
}

/**
 * Reset the previous error, the integral term, and disable the controller.
 */
void Motor3PIDController::Reset() {
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
double Motor3PIDController::GetContinuousError(double error) const {
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
