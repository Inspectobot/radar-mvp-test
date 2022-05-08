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

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <CppLinuxSerial/SerialPort.hpp>

#include "rp.h"
#include <sw/redis++/redis++.h>

#include "ringbuffer.hpp"

#include <thread>

#define TCP_PORT 1001
#define CHANNEL_COUNT 2
#define PROFILE_BUFFER_SIZE 8

using namespace std::chrono;

using namespace std;
using namespace sw::redis;
using namespace mn::CppLinuxSerial;

const string STARTUP_TIMESTAMP_KEY ="rover_startup_timestamp";
const string PARAMETERS_KEY = "radar_parameters";

int64_t startupTimestamp;

int startFrequency   = 1000;
int stepFrequency    = 10;
int frequencyCount   = 201;

int intermediateFreq = 20;
int transmitPower    = 0;
int loPower          = 15;
uint32_t sampleCount = 16384;
int settlingTime = 5000;

static volatile int keepRunning = 1;

struct ParametersMessage {
  uint32_t timestamp = 0;

  int startFrequency = 1000;
  int stepFrequency = 10;
  int frequencyCount = 201;
  int intermediateFreq = 20;
  int transmitPower = 0;
  int loPower = 15;
  uint32_t sampleCount = 16384;
  int settlingTime = 5000;

  MSGPACK_DEFINE_MAP(
    timestamp,

    startFrequency,
    stepFrequency,
    frequencyCount,
    intermediateFreq,
    transmitPower,
    loPower,
    sampleCount,
    settlingTime
  )
};

ParametersMessage lastParametersMessage;

void updateParameters(ParametersMessage parametersMessage) {
  lastParametersMessage = parametersMessage;

  startFrequency = parametersMessage.startFrequency;
  stepFrequency = parametersMessage.stepFrequency;
  frequencyCount = parametersMessage.frequencyCount;
  intermediateFreq = parametersMessage.intermediateFreq;

  transmitPower = parametersMessage.transmitPower;
  loPower = parametersMessage.loPower;
  sampleCount = parametersMessage.sampleCount;
  settlingTime = parametersMessage.settlingTime;
}

void updateParametersFromString(string parametersString) {
  msgpack::object_handle oh = msgpack::unpack(parametersString.data(), parametersString.size());

  msgpack::object deserialized = oh.get();

  std::cout << "updated parameters: " << deserialized << std::endl;

  ParametersMessage parametersMessage;
  deserialized.convert(parametersMessage);

  updateParameters(parametersMessage);
}

SerialPort* rfSource;
Redis* redis;

struct RadarProfile {
  uint32_t timestamp;
  
  float data[];
};

void setFrequency(int frequency, int intermediateFrequency) {
  rfSource->Write("C0");
  rfSource->Write("f" + std::to_string(float(frequency)));
  rfSource->Write("C1");
  rfSource->Write("f" + std::to_string(float(frequency + intermediateFrequency)));
}

void enableExcitation(int transmitPower, int loPower) {
  rfSource->Write("C0");
  rfSource->Write("W" + std::to_string(float(transmitPower)));
  rfSource->Write("C1");
  rfSource->Write("W" + std::to_string(float(loPower)));

  rfSource->Write("C0");

  rfSource->Write("E1");
  rfSource->Write("r1");

  rfSource->Write("C1");

  rfSource->Write("E1");
  rfSource->Write("r1");
}

void queryFrequency() {
  rfSource->Write("C0");
  rfSource->Write("f?");

  std::string channel0;
  rfSource->Read(channel0);

  std::cout << "Channel 0: " << channel0 << std::endl;
}

void disableExcitation() {
  rfSource->Write("C0");

  rfSource->Write("E0");
  rfSource->Write("r0");

  rfSource->Write("C1");

  rfSource->Write("E0");
  rfSource->Write("r0");
}

void intHandler(int dummy) {
	if (keepRunning == 0) {
		printf("shutting down!\n");
    disableExcitation();
    
    rp_Release();

    exit(-1);
	}
	keepRunning = 0;
}

void setupSweep(
    int startFrequency, 
    int stepFrequency, 
    int frequencyCount, 
    int intermediateFrequency,
    float stepTimeInMs) {

  std::cout << "Step time in ms: " << stepTimeInMs << std::endl;

  rfSource->Write("C0");
  
  rfSource->Write("w2");

  rfSource->Write("l" + std::to_string(startFrequency));
  
  int maxStep = stepFrequency * frequencyCount;  
  
  std::cout << "max step: " << maxStep << std::endl;
  rfSource->Write("u" + std::to_string(startFrequency + maxStep));

  rfSource->Write("s" + std::to_string(float(stepFrequency)));

  std::cout << std::to_string(float(stepFrequency));
  
  rfSource->Write("t" + std::to_string(stepTimeInMs));

  rfSource->Write("k" + std::to_string(float(intermediateFrequency)));

  rfSource->Write("Y1");
  
  rfSource->Write("n2");

  rfSource->Write("X0");

  rfSource->Write("^1");

  //rfSource->Write("c0");
}

void runContinuousSweep() {
  rfSource->Write("C0");
  
  rfSource->Write("c1");
  rfSource->Write("g1");
}

void runSingleSweep() {
  rfSource->Write("C0");

  rfSource->Write("g1");
}

jnk0le::Ringbuffer<RadarProfile*, 4> profileBuffer;
struct RadarProfile* profileBuffers[PROFILE_BUFFER_SIZE];

void tcpDataServerTask() {
  cpu_set_t mask;

  struct sched_param param;

  memset(&param, 0, sizeof(param));
  param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  sched_setscheduler(0, SCHED_FIFO, &param);
  
  CPU_ZERO(&mask);
  CPU_SET(0, &mask);
  sched_setaffinity(0, sizeof(cpu_set_t), &mask);

  int sock_server, sock_client;
  int yes = 1;

  struct sockaddr_in addr; 

  printf("Started tcp server task\n");

  if((sock_server = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    printf("Error opening listening socket\n"); 
    keepRunning = false;
  }

  setsockopt(sock_server, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(TCP_PORT);
  
  if(bind(sock_server, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    printf("Error binding to socket\n");
    keepRunning = false;
  }

  listen(sock_server, 1024);
  
  while(keepRunning) {
    if((sock_client = accept(sock_server, NULL, NULL)) < 0) {
      printf("Error accepting connection on socket\n");
      keepRunning = false;
    }

    size_t len = sizeof(struct RadarProfile) + (sampleCount * frequencyCount * sizeof(float) * CHANNEL_COUNT);

    printf("size in bytes %zu\n", len);

    char *data;
    data = (char *) calloc(1, len);

    while(keepRunning) {

      printf("got here\n");
      
      struct RadarProfile* profile = nullptr;

      while(!profileBuffer.remove(profile)) {
        std::this_thread::sleep_for(std::chrono::microseconds(settlingTime * frequencyCount));
        //sched_yield();
      }
      
      memcpy(data, profile, len);
      
      size_t offset = 0;
      ssize_t result;
      while (offset < len) {
        result = send(sock_client, data + offset, len - offset, 0);
        if (result < 0) {
          printf("Error sending!\n");
        }

        offset += result;
      }
    }
  }
}

int main (int argc, char **argv) {
  signal(SIGABRT, intHandler);
  signal(SIGTERM, intHandler);
  signal(SIGINT, intHandler);

  ConnectionOptions redisConnectionOpts;

  redisConnectionOpts.host = "rover";
  redisConnectionOpts.port = 6379;
  redisConnectionOpts.socket_timeout = std::chrono::milliseconds(5); 
  
  redis = new Redis(redisConnectionOpts);

  auto timestamp = redis->get(STARTUP_TIMESTAMP_KEY);
  if(timestamp) {
    string timestampString = *timestamp;

    startupTimestamp = atoll(timestampString.c_str());
  } else {
    std::cout << "Rover startup timestamp not set or invalid, check key: " << STARTUP_TIMESTAMP_KEY << std::endl;

    exit(0);
  }

  std::cout << "Rover startup timestamp: " << startupTimestamp << std::endl;

  auto parameters = redis->get(PARAMETERS_KEY);

  if(parameters) {
    string parametersString = *parameters;

    updateParametersFromString(parametersString);
  } else {

    ParametersMessage parametersMessage;

    std::stringstream packed;
    msgpack::pack(packed, parametersMessage);

    packed.seekg(0);

    std::string str(packed.str());

    msgpack::object_handle oh =
      msgpack::unpack(str.data(), str.size());

    msgpack::object deserialized = oh.get();
    std::cout << "reset parameters: " << deserialized << std::endl;

    redis->set(PARAMETERS_KEY, packed.str());

    updateParameters(parametersMessage);
  }
  
  thread tcpDataServerThread(tcpDataServerTask);
  
  cpu_set_t mask;

  struct sched_param param;

  memset(&param, 0, sizeof(param));
  param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  sched_setscheduler(0, SCHED_FIFO, &param);
  
  CPU_ZERO(&mask);
  CPU_SET(1, &mask);
  sched_setaffinity(0, sizeof(cpu_set_t), &mask);

  if (rp_Init() != RP_OK) {
    fprintf(stderr, "Red Pitaya API init failed!\n");
    exit(0);
  }

  rp_DpinReset();
  rp_AcqReset();
  rp_AcqSetTriggerDelay(0);

  rp_acq_decimation_t adc_precision = RP_DEC_1;
  rp_acq_sampling_rate_t sampling_rate = RP_SMP_122_880M; 

  rp_AcqSetSamplingRate(sampling_rate);
  rp_AcqSetDecimation(adc_precision);

  rp_dpin_t stepPin = RP_DIO5_N;
  rp_pinDirection_t direction = RP_OUT;

  rp_DpinSetDirection(stepPin, direction);
  rp_DpinSetState(stepPin, RP_LOW);

  long long int sampleTimeInMicro = ((1 / ADC_SAMPLE_RATE) * sampleCount * 1000000);

  std::cout << "sample time in micro: " << sampleTimeInMicro << std::endl;

  rfSource = new SerialPort("/dev/ttyACM0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
  rfSource->SetTimeout(100);
  rfSource->Open();

  setFrequency(startFrequency, intermediateFreq);
  //setupSweep(startFrequency, stepFrequency, frequencyCount, intermediateFreq, (settlingTime / 1000) + (sampleTimeInMicro / 1000));

  enableExcitation(transmitPower, loPower);
 
  for(int i = 0; i < PROFILE_BUFFER_SIZE; i++) {
    struct RadarProfile* profile = (RadarProfile *)calloc(1, sizeof(struct RadarProfile) + (sampleCount * frequencyCount * sizeof(float) * CHANNEL_COUNT));

    profileBuffers[i] = profile;
  }
  
  int currentBufferIndex = 0;
  
  while(keepRunning) {
    int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

    if(currentBufferIndex > (PROFILE_BUFFER_SIZE - 1)) currentBufferIndex = 0;

    profileBuffers[currentBufferIndex]->timestamp = uint32_t(currentMicro - startupTimestamp);

    for(int i = 0; i < frequencyCount; i++) {
      setFrequency(startFrequency + (i * stepFrequency), intermediateFreq);
      std::this_thread::sleep_for(std::chrono::microseconds(settlingTime));

      queryFrequency();

      //int64_t startSampleTimeMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
      
      rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);

      bool fillState = false;
      rp_AcqStart();

      std::this_thread::sleep_for(std::chrono::microseconds(sampleTimeInMicro));
    
      while(!fillState) {
        rp_AcqGetBufferFillState(&fillState);
      }

      rp_AcqStop();
      
      rp_AcqGetDataV2(0, 
        &sampleCount,
        &profileBuffers[currentBufferIndex]->data[i * sampleCount],
        &profileBuffers[currentBufferIndex]->data[(sampleCount * frequencyCount) + (i * sampleCount)]);
      
      //rp_DpinSetState(stepPin, RP_HIGH);
      //rp_DpinSetState(stepPin, RP_LOW);
      
      //queryFrequency();

      //int64_t endSampleTimeMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
      //sampleTimes[i] = (endSampleTimeMicro - startSampleTimeMicro);
    }

    profileBuffer.insert(&profileBuffers[currentBufferIndex]);
     
    currentBufferIndex++; 
    int64_t endTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    
    printf("Sweep Done, took %lld microseconds\n", endTime - currentMicro);
  }

  disableExcitation();
  
  rp_Release();

  return 0;
}
