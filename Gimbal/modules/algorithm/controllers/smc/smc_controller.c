#include "controllers/smc/smc_controller.h"

#include <math.h>
#include <string.h>

static float SMC_ControllerSignal(float value) {
  if (value > 0.0f) {
    return 1.0f;
  }
  if (value < 0.0f) {
    return -1.0f;
  }
  return 0.0f;
}

static float SMC_ControllerSat(const SMC_ControllerInstance *controller,
                               float value) {
  if (controller == NULL) {
    return 0.0f;
  }

  if (controller->param.epsilon <= 0.0f) {
    return SMC_ControllerSignal(value);
  }

  float normalized = value / controller->param.epsilon;
  if (fabsf(normalized) <= controller->sat_limit) {
    return normalized;
  }
  return SMC_ControllerSignal(normalized);
}

static void SMC_ControllerApplyOutputContinuation(
    SMC_ControllerInstance *controller) {
  if (controller == NULL) {
    return;
  }

  if (controller->param.K != 0.0f && controller->param_last.K != 0.0f) {
    if (controller->param.c2 != 0.0f && controller->param_last.c2 != 0.0f) {
      controller->error.position_error_integral *=
          (controller->param_last.K / controller->param.K) *
          (controller->param_last.c2 / controller->param.c2);
    }

    if (controller->param.c != 0.0f && controller->param_last.c != 0.0f) {
      controller->error.velocity_error_integral *=
          (controller->param_last.K / controller->param.K) *
          (controller->param_last.c / controller->param.c);
    }
  }

  controller->param_last = controller->param;
}

static float SMC_ControllerGetPeriod(const SMC_ControllerInstance *controller) {
  if (controller == NULL || controller->sample_period <= 0.0f) {
    return SMC_CONTROLLER_DEFAULT_SAMPLE_PERIOD;
  }
  return controller->sample_period;
}

void SMC_ControllerInit(SMC_ControllerInstance *controller) {
  if (controller == NULL) {
    return;
  }

  memset(controller, 0, sizeof(*controller));
  controller->mode = SMC_CONTROLLER_MODE_EXPONENT;
  controller->sample_period = SMC_CONTROLLER_DEFAULT_SAMPLE_PERIOD;
}

void SMC_ControllerInitFromConfig(SMC_ControllerInstance *controller,
                                  const SMC_ControllerInitConfig_s *config) {
  if (controller == NULL) {
    return;
  }

  SMC_ControllerInit(controller);

  if (config == NULL) {
    return;
  }

  if (config->sample_period > 0.0f) {
    controller->sample_period = config->sample_period;
  }

  switch (config->mode) {
  case SMC_CONTROLLER_MODE_EXPONENT:
  case SMC_CONTROLLER_MODE_POWER:
  case SMC_CONTROLLER_MODE_VELSMC:
    SMC_ControllerSetParam(controller, config->J, config->K, config->c,
                           config->epsilon, config->sat_limit,
                           config->output_limit, config->mode,
                           config->position_epsilon);
    break;

  case SMC_CONTROLLER_MODE_TFSMC:
    SMC_ControllerSetTFParam(controller, config->J, config->K, config->p,
                             config->q, config->beta, config->epsilon,
                             config->sat_limit, config->output_limit,
                             config->mode, config->position_epsilon);
    break;

  case SMC_CONTROLLER_MODE_EISMC:
    SMC_ControllerSetEIParam(controller, config->J, config->K, config->c1,
                             config->c2, config->epsilon, config->sat_limit,
                             config->output_limit, config->mode,
                             config->position_epsilon);
    break;

  default:
    SMC_ControllerSetParam(controller, config->J, config->K, config->c,
                           config->epsilon, config->sat_limit,
                           config->output_limit,
                           SMC_CONTROLLER_MODE_EXPONENT,
                           config->position_epsilon);
    break;
  }
}

void SMC_ControllerSetSamplePeriod(SMC_ControllerInstance *controller,
                                   float sample_period) {
  if (controller == NULL || sample_period <= 0.0f) {
    return;
  }

  controller->sample_period = sample_period;
}

void SMC_ControllerSetParam(SMC_ControllerInstance *controller, float J,
                            float K, float c, float epsilon, float sat_limit,
                            float output_limit, SMC_ControllerMode_e mode,
                            float position_epsilon) {
  if (controller == NULL) {
    return;
  }

  memset(&controller->param, 0, sizeof(controller->param));
  controller->param.J = J;
  controller->param.K = K;
  controller->param.c = c;
  controller->param.epsilon = epsilon;

  controller->error.position_error_epsilon = position_epsilon;
  controller->mode = mode;
  controller->output_limit = output_limit;
  controller->sat_limit = sat_limit;

  SMC_ControllerApplyOutputContinuation(controller);
}

void SMC_ControllerSetTFParam(SMC_ControllerInstance *controller, float J,
                              float K, float p, float q, float beta,
                              float epsilon, float sat_limit,
                              float output_limit, SMC_ControllerMode_e mode,
                              float position_epsilon) {
  if (controller == NULL) {
    return;
  }

  memset(&controller->param, 0, sizeof(controller->param));
  controller->param.J = J;
  controller->param.K = K;
  controller->param.p = p;
  controller->param.q = q;
  controller->param.beta = beta;
  controller->param.epsilon = epsilon;

  controller->error.position_error_epsilon = position_epsilon;
  controller->mode = mode;
  controller->output_limit = output_limit;
  controller->sat_limit = sat_limit;

  SMC_ControllerApplyOutputContinuation(controller);
}

void SMC_ControllerSetEIParam(SMC_ControllerInstance *controller, float J,
                              float K, float c1, float c2, float epsilon,
                              float sat_limit, float output_limit,
                              SMC_ControllerMode_e mode,
                              float position_epsilon) {
  if (controller == NULL) {
    return;
  }

  memset(&controller->param, 0, sizeof(controller->param));
  controller->param.J = J;
  controller->param.K = K;
  controller->param.c1 = c1;
  controller->param.c2 = c2;
  controller->param.epsilon = epsilon;

  controller->error.position_error_epsilon = position_epsilon;
  controller->mode = mode;
  controller->output_limit = output_limit;
  controller->sat_limit = sat_limit;

  SMC_ControllerApplyOutputContinuation(controller);
}

void SMC_ControllerUpdatePositionError(SMC_ControllerInstance *controller,
                                       float target, float position_now,
                                       float velocity_now) {
  if (controller == NULL) {
    return;
  }

  float sample_period = SMC_ControllerGetPeriod(controller);

  controller->error.target_now = target;
  controller->error.target_differential =
      (controller->error.target_now - controller->error.target_last) /
      sample_period;

  controller->error.target_second_differential =
      (controller->error.target_differential -
       controller->error.target_differential_last) /
      sample_period;

  controller->error.position_feedback = position_now;
  controller->error.velocity_feedback = velocity_now;

  controller->error.position_error = position_now - target;
  controller->error.velocity_error =
      velocity_now - controller->error.target_differential;

  controller->error.target_last = controller->error.target_now;
  controller->error.position_error_integral +=
      controller->error.position_error * sample_period;
  controller->error.target_differential_last =
      controller->error.target_differential;
}

void SMC_ControllerUpdateVelocityError(SMC_ControllerInstance *controller,
                                       float target, float velocity_now) {
  if (controller == NULL) {
    return;
  }

  float sample_period = SMC_ControllerGetPeriod(controller);

  controller->error.target_now = target;
  controller->error.target_differential =
      (controller->error.target_now - controller->error.target_last) /
      sample_period;

  controller->error.velocity_feedback = velocity_now;
  controller->error.velocity_error = velocity_now - controller->error.target_now;
  controller->error.velocity_error_integral +=
      controller->error.velocity_error * sample_period;

  controller->error.target_last = controller->error.target_now;
}

void SMC_ControllerClear(SMC_ControllerInstance *controller) {
  if (controller == NULL) {
    return;
  }

  memset(&controller->error, 0, sizeof(controller->error));
  controller->output = 0.0f;
  controller->sliding_surface = 0.0f;
}

void SMC_ControllerResetState(SMC_ControllerInstance *controller, float target) {
  if (controller == NULL) {
    return;
  }

  SMC_ControllerClear(controller);
  controller->error.target_now = target;
  controller->error.target_last = target;
}

void SMC_ControllerClearIntegral(SMC_ControllerInstance *controller) {
  if (controller == NULL) {
    return;
  }

  controller->error.velocity_error_integral = 0.0f;
  controller->error.position_error_integral = 0.0f;
}

float SMC_ControllerCalculate(SMC_ControllerInstance *controller) {
  if (controller == NULL) {
    return 0.0f;
  }

  float output = 0.0f;
  float sat_output = 0.0f;

  switch (controller->mode) {
  case SMC_CONTROLLER_MODE_EXPONENT:
    if (fabsf(controller->error.position_error) -
            controller->error.position_error_epsilon <
        0.0f) {
      controller->error.position_error = 0.0f;
      controller->sliding_surface = 0.0f;
      controller->output = 0.0f;
      return 0.0f;
    }

    controller->sliding_surface =
        controller->param.c * controller->error.position_error +
        controller->error.velocity_error;
    sat_output = SMC_ControllerSat(controller, controller->sliding_surface);

    output = controller->param.J *
             ((-controller->param.c * controller->error.velocity_error) -
              controller->param.K * controller->sliding_surface -
              controller->param.epsilon * sat_output);
    break;

  case SMC_CONTROLLER_MODE_POWER:
    if (fabsf(controller->error.position_error) -
            controller->error.position_error_epsilon <
        0.0f) {
      controller->error.position_error = 0.0f;
      controller->sliding_surface = 0.0f;
      controller->output = 0.0f;
      return 0.0f;
    }

    controller->sliding_surface =
        controller->param.c * controller->error.position_error +
        controller->error.velocity_error;
    sat_output = SMC_ControllerSat(controller, controller->sliding_surface);

    output =
        controller->param.J *
        ((-controller->param.c * controller->error.velocity_error) -
         controller->param.K * controller->sliding_surface -
         controller->param.K *
             powf(fabsf(controller->sliding_surface), controller->param.epsilon) *
             sat_output);
    break;

  case SMC_CONTROLLER_MODE_TFSMC: {
    if (fabsf(controller->error.position_error) -
            controller->error.position_error_epsilon <
        0.0f) {
      controller->error.position_error = 0.0f;
      controller->sliding_surface = 0.0f;
      controller->output = 0.0f;
      return 0.0f;
    }

    float position_power = 0.0f;
    if (controller->param.p != 0.0f) {
      position_power = powf(fabsf(controller->error.position_error),
                            controller->param.q / controller->param.p);
    }

    if (controller->error.position_error <= 0.0f) {
      position_power = -position_power;
    }

    controller->sliding_surface =
        controller->param.beta * position_power +
        controller->error.velocity_error;
    sat_output = SMC_ControllerSat(controller, controller->sliding_surface);

    if (controller->error.position_error != 0.0f &&
        controller->param.p != 0.0f) {
      output = controller->param.J *
               (controller->error.target_second_differential -
                controller->param.K * controller->sliding_surface -
                controller->param.epsilon * sat_output -
                controller->error.velocity_error *
                    ((controller->param.q * controller->param.beta) *
                     position_power) /
                    (controller->param.p * controller->error.position_error));
    }
    break;
  }

  case SMC_CONTROLLER_MODE_VELSMC:
    controller->sliding_surface =
        controller->error.velocity_error +
        controller->param.c * controller->error.velocity_error_integral;
    sat_output = SMC_ControllerSat(controller, controller->sliding_surface);

    output = controller->param.J *
             (controller->error.target_differential -
              controller->param.c * controller->error.velocity_error -
              controller->param.K * controller->sliding_surface -
              controller->param.epsilon * sat_output);
    break;

  case SMC_CONTROLLER_MODE_EISMC:
    if (fabsf(controller->error.position_error) -
            controller->error.position_error_epsilon <
        0.0f) {
      controller->error.position_error = 0.0f;
      controller->sliding_surface = 0.0f;
      controller->output = 0.0f;
      return 0.0f;
    }

    controller->sliding_surface =
        controller->param.c1 * controller->error.position_error +
        controller->error.velocity_error +
        controller->param.c2 * controller->error.position_error_integral;
    sat_output = SMC_ControllerSat(controller, controller->sliding_surface);

    output = controller->param.J *
             ((-controller->param.c1 * controller->error.velocity_error) -
              controller->param.c2 * controller->error.position_error -
              controller->param.K * controller->sliding_surface -
              controller->param.epsilon * sat_output);
    break;

  default:
    output = 0.0f;
    break;
  }

  controller->error.error_last = controller->error.position_error;

  if (output > controller->output_limit) {
    output = controller->output_limit;
  }
  if (output < -controller->output_limit) {
    output = -controller->output_limit;
  }

  controller->output = output;
  return output;
}

float SMC_ControllerGetOutput(const SMC_ControllerInstance *controller) {
  if (controller == NULL) {
    return 0.0f;
  }

  return controller->output;
}

void SMC_ControllerSetOutput(SMC_ControllerInstance *controller, float output) {
  if (controller == NULL) {
    return;
  }

  controller->output = output;
}
