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

#define Z20 1
#include "rp_cross.h"

extern "C" {
  #include "oscilloscope.h"
}

#include <sw/redis++/redis++.h>

#include "ringbuffer.hpp"

#define NUMCPP_NO_USE_BOOST 1
#include "NumCpp.hpp"

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

float startFrequency   = 1000.00;
float stepFrequency    = 10.00;
int frequencyCount   = 201;

float intermediateFreq = 32.00;
float transmitPower    = 0.00;
float loPower          = 15.00;
uint32_t sampleCount = 2048;
int settlingTimeInMicro = 500000;
int sampleTimeInMicro = 133;
int stepTriggerTimeInMicro = 5;

static volatile int keepRunning = 1;

struct ParametersMessage {
  uint32_t timestamp = 0;

  float startFrequency = 1000.00;
  float stepFrequency = 10.00;
  int frequencyCount = 201;
  float intermediateFreq = 32.00;
  float transmitPower = 0.00;
  float loPower = 15.00;
  uint32_t sampleCount = 2048;
  int settlingTimeInMicro = 200000;
  int stepTriggerTimeInMicro = 5;

  MSGPACK_DEFINE_MAP(
    timestamp,

    startFrequency,
    stepFrequency,
    frequencyCount,
    intermediateFreq,
    transmitPower,
    loPower,
    sampleCount,
    settlingTimeInMicro,
    stepTriggerTimeInMicro
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
  settlingTimeInMicro = parametersMessage.settlingTimeInMicro;
  stepTriggerTimeInMicro = parametersMessage.stepTriggerTimeInMicro;
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

nc::NdArray<float> frequencyRange = nc::linspace<float>(startFrequency, startFrequency + frequencyCount * stepFrequency, frequencyCount);

void setFrequency(float frequency, int intermediateFrequency) {
  std::cout << "setting frequency: " << std::to_string(float(frequency)) << ", " << std::to_string(float(frequency + intermediateFrequency)) << std::endl;

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

  rfSource->Write("C1");
  rfSource->Write("f?");

  std::string channel1;
  rfSource->Read(channel1);

  std::cout << "Channel 1: " << channel1 << std::endl;

  rfSource->Write("C0");
}

void queryPower() {
  rfSource->Write("C0");
  rfSource->Write("W?");

  std::string channel0;
  rfSource->Read(channel0);

  std::cout << "Channel 0 Power: " << channel0 << std::endl;

  rfSource->Write("C1");
  rfSource->Write("W?");

  std::string channel1;
  rfSource->Read(channel1);

  std::cout << "Channel 1 Power: " << channel1 << std::endl;
  
  rfSource->Write("C0");
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
    nc::NdArray<float> sweepRange,
    float intermediateFrequency,
    float stepTimeInMs) {

  std::cout << "Step time in ms: " << stepTimeInMs << std::endl;

  rfSource->Write("C0");
  
  std::stringstream sweepTableSs;
  
  sweepTableSs << "Ld";

  int frequencyCount = sweepRange.size();

  for(int i = 0; i < (frequencyCount * 2); i++) {
    int j = i;

    if(i > (frequencyCount - 1)) {
      j = (frequencyCount - 1) - (i - frequencyCount);
    }

    sweepTableSs
      << "L" 
      << std::to_string(i) 
      << "f" << std::fixed << std::setprecision(7) << sweepRange[j]
      << "L"
      << std::to_string(i)
      << "a" << std::fixed << std::setprecision(2) << transmitPower;
  }

  std::string sweepTableStr = sweepTableSs.str();

  std::cout << sweepTableStr << std::endl;

  rfSource->Write(sweepTableStr);

  // Enable trigger: (0=software, 1=sweep, 2=step, 3=hold all, ..)
  rfSource->Write("w2");

  // Sweep step time (mS)
  rfSource->Write("t" + std::to_string(stepTimeInMs));

  std::stringstream ifSs;

  ifSs << std::fixed << std::setprecision(7) << intermediateFrequency;
  
  // Sweep differential seperation (MHz)
  rfSource->Write("k" + ifSs.str());

  // Trigger polarity high
  rfSource->Write("Y1");
  
  // Sweep differential: (0=off, 1=ChA-DiffFreq, 2=ChA+DiffFreq)
  rfSource->Write("n2");

  // Sweep type (lin=0 / tab=1 / %=2)
  rfSource->Write("X1");
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
        std::this_thread::sleep_for(std::chrono::microseconds(settlingTimeInMicro * sampleTimeInMicro * frequencyCount));
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

  frequencyRange.print();

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

  rp_InitReset(true);
  rp_DpinReset();
  rp_AcqReset();

  // needed to set internal flag `triggerDelayInNs = false`
  rp_AcqSetTriggerDelay(0);
  osc_SetTriggerDelay(sampleCount);  
  
  rp_AcqSetSamplingRate(RP_SMP_122_880M);
  rp_AcqSetDecimation(RP_DEC_1);

  // uint32_t decimation = 1;
  // rp_AcqSetDecimationFactor(&decimation);
  rp_dpin_t stepPin = RP_DIO5_N;
  rp_pinDirection_t direction = RP_OUT;

  rp_DpinSetDirection(stepPin, direction);
  rp_DpinSetState(stepPin, RP_LOW);
  
  sampleTimeInMicro = ((1 / ADC_SAMPLE_RATE) * sampleCount * 1000000);

  std::cout << "ADC sample rate: " << ADC_SAMPLE_RATE << std::endl;

  std::cout << "sample time in micro: " << sampleTimeInMicro << std::endl;

  rfSource = new SerialPort("/dev/ttyACM0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
  rfSource->SetTimeout(100);
  rfSource->Open();

  setFrequency(frequencyRange[0], intermediateFreq);
  setupSweep(frequencyRange, intermediateFreq, ((stepTriggerTimeInMicro + settlingTimeInMicro + sampleTimeInMicro) / 1000));

  enableExcitation(transmitPower, loPower);
 
  for(int i = 0; i < PROFILE_BUFFER_SIZE; i++) {
    struct RadarProfile* profile = (RadarProfile *)calloc(1, sizeof(struct RadarProfile) + (sampleCount * frequencyCount * sizeof(float) * CHANNEL_COUNT));

    profileBuffers[i] = profile;
  }
  
  int currentBufferIndex = 0;
  bool sweepDirectionUp = true;
  
  setFrequency(frequencyRange[0], intermediateFreq);
  std::this_thread::sleep_for(std::chrono::microseconds(settlingTimeInMicro));

  while(keepRunning) {
    int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

    if(currentBufferIndex > (PROFILE_BUFFER_SIZE - 1)) currentBufferIndex = 0;

    profileBuffers[currentBufferIndex]->timestamp = uint32_t(currentMicro - startupTimestamp);

    std::cout << "timestamp is: " << profileBuffers[currentBufferIndex]->timestamp << std::endl;
    
    for(int i = 0; i < frequencyCount; i++) {
      //int64_t startSampleTimeMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

      rp_DpinSetState(stepPin, RP_HIGH);
      std::this_thread::sleep_for(std::chrono::microseconds(stepTriggerTimeInMicro));
      rp_DpinSetState(stepPin, RP_LOW);

      std::this_thread::sleep_for(std::chrono::microseconds(settlingTimeInMicro));

      queryFrequency();
      queryPower();

      rp_AcqStart();
      
      std::this_thread::sleep_for(std::chrono::microseconds(sampleTimeInMicro));

      rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);
       
      bool fillState = false;

      while(!fillState) {
          rp_AcqGetBufferFillState(&fillState);
        }
      
      rp_AcqStop();
      
      uint32_t bufferTriggerPosition;
      rp_AcqGetWritePointerAtTrig(&bufferTriggerPosition);
      
      int idx0, idx1; 
      if(sweepDirectionUp) {
        idx0 = i * sampleCount;
        idx1 = (sampleCount * frequencyCount) + idx0;
      } else {
        idx0 = ((frequencyCount - 1) - i) * sampleCount;
        idx1 = (sampleCount * frequencyCount) + idx0;
      }

      //std::cout << (sweepDirectionUp ? "sweep up " : "sweep down ") << " indexes: " << idx0 << ", " << idx1 << std::endl;

      rp_AcqGetDataV2(bufferTriggerPosition,
        &sampleCount,
        &profileBuffers[currentBufferIndex]->data[idx0],
        &profileBuffers[currentBufferIndex]->data[idx1]);

      /*if(i == 0) {
        printf("insert\n");
        profileBuffer.insert(&profileBuffers[currentBufferIndex]);
        sleep(100000);
      }*/
         
      //int64_t endSampleTimeMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    }

    profileBuffer.insert(&profileBuffers[currentBufferIndex]);
    sweepDirectionUp = !sweepDirectionUp;
     
    currentBufferIndex++; 
    int64_t endTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    
    printf("Sweep Done, took %lld microseconds\n", endTime - currentMicro);
  }

  disableExcitation();
  
  rp_Release();

  return 0;
}
