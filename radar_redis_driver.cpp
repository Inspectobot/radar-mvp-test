#include <stdio.h>
#include <string.h>
#include "iostream"
#include <sstream>

#include <unistd.h>

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

#include <msgpack.hpp>

#include <chrono>
#include <thread>

#include <CppLinuxSerial/SerialPort.hpp>

#include "rp.h"
#include <sw/redis++/redis++.h>

using namespace std::chrono;

using namespace std;
using namespace sw::redis;

using namespace mn::CppLinuxSerial;

int64_t startupTimestamp;

static volatile int keepRunning = 1;

SerialPort* rfSource;

void intHandler(int dummy) {
	if (keepRunning == 0) {
		exit(-1);
	}
	keepRunning = 0;
}

void setFrequency(int frequency, int intermediateFrequency) {
  rfSource->Write("C0");
  rfSource->Write("f" + std::to_string(frequency));
  rfSource->Write("C1\n");
  rfSource->Write("f" + std::to_string(frequency + intermediateFrequency));
}

int main (int argc, char **argv) {
  if (rp_Init() != RP_OK) {
    fprintf(stderr, "Red Pitaya API init failed!\n");
    return EXIT_FAILURE;
  }

  rp_AcqReset();

  rp_acq_decimation_t adc_precision = RP_DEC_1;
  rp_acq_trig_src_t trigger_src = RP_TRIG_SRC_NOW;
  rp_acq_sampling_rate_t sampling_rate = RP_SMP_122_880M; 

  rp_AcqSetSamplingRate(sampling_rate);
  rp_AcqSetDecimation(adc_precision);

  int startFrequency   = 1000;
  int stepFrequency    = 20;
  int frequencyCount   = 101;
  int intermediateFreq = 32;
  int transmitPower    = 0;
  int loPower          = 15;
  uint32_t sampleCount = 1024;

  long long int sampleTimeInNs = (1 / ADC_SAMPLE_RATE) * sampleCount * 1000000000;

  rfSource = new SerialPort("/dev/ttyACM0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
  rfSource->SetTimeout(0);
  rfSource->Open();

  rfSource->Write("C0");
  rfSource->Write("W" + std::to_string(transmitPower));
  rfSource->Write("C1");
  rfSource->Write("W" + std::to_string(loPower));

  setFrequency(startFrequency, intermediateFreq);
  
  rfSource->Write("C0");

  rfSource->Write("E1");
  rfSource->Write("r1");

  rfSource->Write("C1");

  rfSource->Write("E1");
  rfSource->Write("r1");

  float *dut_buff = (float *)malloc(sampleCount * frequencyCount * sizeof(float));
  float *ref_buff = (float *)malloc(sampleCount * frequencyCount * sizeof(float));
 
  //uint16_t *dut_buff = (uint16_t *)malloc(sampleCount * frequencyCount * sizeof(uint16_t));
  //uint16_t *ref_buff = (uint16_t *)malloc(sampleCount * frequencyCount * sizeof(uint16_t));
  
  int64_t startTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  for(int i = 0; i < frequencyCount; i++) {
    bool fillState = false;
    
    rp_AcqStart();

    std::this_thread::sleep_for(std::chrono::nanoseconds(sampleTimeInNs));
  
    rp_AcqSetTriggerSrc(trigger_src);
    rp_acq_trig_state_t state = RP_TRIG_STATE_WAITING;

    while(1) {
      rp_AcqGetTriggerState(&state);
      if(state == RP_TRIG_STATE_TRIGGERED) {
        break;
      }
    }

    while(!fillState) {
      rp_AcqGetBufferFillState(&fillState);
    }

    rp_AcqStop();
    
    //rp_AcqGetDataRawV2(0, &sampleCount, &dut_buff[i * sizeof(uint16_t)], &ref_buff[i * sizeof(uint16_t)]);

    rp_AcqGetDataV2(0, &sampleCount, &dut_buff[i * sizeof(float)], &ref_buff[i * sizeof(float)]);
   
    setFrequency(startFrequency + (i * stepFrequency), intermediateFreq);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
  }

  int64_t endTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  
  printf("Sweep Done, took %lld microseconds\n", endTime - startTime);

  free(dut_buff);
  free(ref_buff);

  rfSource->Write("C0");

  rfSource->Write("E0");
  rfSource->Write("r0");

  rfSource->Write("C1");

  rfSource->Write("E0");
  rfSource->Write("r0");

  rp_Release();

  return 0;
}
