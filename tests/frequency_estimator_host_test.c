#include "frequency_estimator.h"
#include "signal_separation_config.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define test_pi 3.14159265358979323846

typedef enum
{
  test_wave_sine = 0,
  test_wave_triangle = 1,
  test_wave_square = 2
} test_wave_t;

static uint32_t noise_state = 0x12345678U;

static float waveform_value(test_wave_t wave, double phase)
{
  if (wave == test_wave_square)
  {
    return (phase < 0.5) ? 1.0f : -1.0f;
  }
  if (wave == test_wave_triangle)
  {
    if (phase < 0.25)
    {
      return (float)(4.0 * phase);
    }
    if (phase < 0.75)
    {
      return (float)(2.0 - (4.0 * phase));
    }
    return (float)((4.0 * phase) - 4.0);
  }
  return (float)sin(2.0 * test_pi * phase);
}

static const char *wave_name(test_wave_t wave)
{
  if (wave == test_wave_triangle)
  {
    return "triangle";
  }
  if (wave == test_wave_square)
  {
    return "square";
  }
  return "sine";
}

static test_wave_t classify_result(const frequency_estimator_result_t *result)
{
  float harmonic3 = result->harmonic3_ratio[0];
  float harmonic5 = result->harmonic5_ratio[0];

  if ((harmonic3 >= SIGSEP_SQUARE_H3_RATIO) ||
      ((harmonic5 >= SIGSEP_SQUARE_H5_RATIO) &&
       (harmonic3 >= (SIGSEP_SQUARE_H3_RATIO * 0.5f))))
  {
    return test_wave_square;
  }
  if ((harmonic3 >= SIGSEP_TRI_H3_RATIO) ||
      ((harmonic5 >= SIGSEP_TRI_H5_RATIO) &&
       (harmonic3 >= (SIGSEP_TRI_H3_RATIO * 0.5f))))
  {
    return test_wave_triangle;
  }
  return test_wave_sine;
}

static int run_case(double frequency_hz, test_wave_t wave,
                    double initial_phase)
{
  uint16_t samples[SIGSEP_ADC_DMA_HALF_LEN];
  frequency_estimator_result_t result;
  double phase = initial_phase;
  double phase_step = frequency_hz / (double)SIGSEP_SAMPLE_RATE_HZ;
  uint32_t block;
  uint8_t ready = 0U;
  double error_hz;
  double tolerance_hz;
  double expected_phase;
  double measured_phase;
  double phase_error_degree;
  double phase_tolerance_degree;
  test_wave_t detected;

  frequency_estimator_reset();
  for (block = 0U; block < 2300U; block++)
  {
    uint32_t index;

    for (index = 0U; index < SIGSEP_ADC_DMA_HALF_LEN; index++)
    {
      float value = waveform_value(wave, phase);
      int32_t noise;
      int32_t code;

      noise_state = (noise_state * 1664525U) + 1013904223U;
      noise = (int32_t)((noise_state >> 24U) & 0x1FU) - 16;
      code = 32768 + (int32_t)(18000.0f * value) + noise;
      if (code < 0)
      {
        code = 0;
      }
      else if (code > 65535)
      {
        code = 65535;
      }
      samples[index] = (uint16_t)code;

      phase += phase_step;
      if (phase >= 1.0)
      {
        phase -= 1.0;
      }
    }

    ready = frequency_estimator_push(samples, SIGSEP_ADC_DMA_HALF_LEN,
                                     1U, &result);
    if (ready != 0U)
    {
      break;
    }
  }

  if (ready == 0U)
  {
    fprintf(stderr, "FAIL no result: %.3f Hz %s\n",
            frequency_hz, wave_name(wave));
    return 1;
  }

  error_hz =
    fabs(((double)result.frequency_millihz[0] * 0.001) - frequency_hz);
  tolerance_hz = (frequency_hz <= 1200.0) ? 0.20 : 1.00;
  expected_phase =
    initial_phase +
    (frequency_hz *
     (double)((((uint64_t)block + 1ULL) *
               SIGSEP_ADC_DMA_HALF_LEN) -
              SIGSEP_ADC_DMA_HALF_LEN) /
     (double)SIGSEP_SAMPLE_RATE_HZ);
  measured_phase = (double)result.phase_q32[0] / 4294967296.0;
  phase_error_degree =
    fabs(remainder(measured_phase - expected_phase, 1.0) * 360.0);
  phase_tolerance_degree =
    (wave == test_wave_square) ? 15.0 : 5.0;
  detected = classify_result(&result);
  if ((error_hz > tolerance_hz) || (detected != wave) ||
      (result.amplitude_adc[0] < 1000.0f) ||
      (phase_error_degree > phase_tolerance_degree))
  {
    fprintf(stderr,
            "FAIL f=%.3f got=%.3f err=%.6f wave=%s/%s "
            "H3=%.4f H5=%.4f low=%u amp=%.1f phase_err=%.3fdeg\n",
            frequency_hz,
            (double)result.frequency_millihz[0] * 0.001,
            error_hz, wave_name(wave), wave_name(detected),
            result.harmonic3_ratio[0], result.harmonic5_ratio[0],
            result.low_frequency_path, result.amplitude_adc[0],
            phase_error_degree);
    return 1;
  }

  if (((frequency_hz <= 1200.0) &&
       (result.low_frequency_path == 0U)) ||
      ((frequency_hz > 1200.0) &&
       (result.low_frequency_path != 0U)))
  {
    fprintf(stderr, "FAIL route: %.3f Hz low=%u\n",
            frequency_hz, result.low_frequency_path);
    return 1;
  }

  printf("PASS f=%9.3f %-8s got=%9.3f H3=%.4f H5=%.4f low=%u phase=%.3fdeg\n",
         frequency_hz, wave_name(wave),
         (double)result.frequency_millihz[0] * 0.001,
         result.harmonic3_ratio[0], result.harmonic5_ratio[0],
         result.low_frequency_path, phase_error_degree);
  return 0;
}

int main(void)
{
  static const double low_and_edge_frequencies[] =
  {
    40.0, 50.0, 100.0, 500.0, 999.0, 1000.0, 1100.0, 1200.0,
    1234.567, 5000.0, 10000.0, 250000.0, 400000.0
  };
  static const double regression_frequencies[] =
  {
    1000.0, 2000.0, 3000.0, 4000.0
  };
  uint32_t wave;
  uint32_t index;
  int failures = 0;

  for (wave = 0U; wave < 3U; wave++)
  {
    for (index = 0U;
         index < (sizeof(low_and_edge_frequencies) /
                  sizeof(low_and_edge_frequencies[0]));
         index++)
    {
      failures +=
        run_case(low_and_edge_frequencies[index],
                 (test_wave_t)wave, 0.173);
    }
    for (index = 0U;
         index < (sizeof(regression_frequencies) /
                  sizeof(regression_frequencies[0]));
         index++)
    {
      failures +=
        run_case(regression_frequencies[index],
                 (test_wave_t)wave, 0.617);
    }
  }

  if (failures != 0)
  {
    fprintf(stderr, "%d host cases failed\n", failures);
    return EXIT_FAILURE;
  }

  puts("All single-signal frequency and waveform cases passed.");
  return EXIT_SUCCESS;
}
