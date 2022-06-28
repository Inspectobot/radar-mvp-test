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

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <termios.h>

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
using namespace LibSerial;

#define DEBUG_SYNTH_SERIAL 0
class SynthSerialPort : public SerialPort {
  public:
    std::string SendCommand(const std::string& dataString, int responseLineCount = 10) {
      if(DEBUG_SYNTH_SERIAL) std::cout << "Wrote command: " << dataString << std::endl;

      SerialPort::Write(dataString + ";\r");
      SerialPort::DrainWriteBuffer();

      std::stringstream ss;

      while(responseLineCount > 0) {
        responseLineCount--;

        try {

          std::string responseLine;
          SerialPort::ReadLine(responseLine, '\r', 1);
          if(DEBUG_SYNTH_SERIAL) std::cout << "Response: " << responseLine << endl;
          ss << responseLine;

        } catch(ReadTimeout&) {}
      }

      return ss.str();
    }
};

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
int bufferSampleDelay = 8192;
int sampleTimeInMicro = 133;
int stepTriggerTimeInMicro = 5;
int synthWarmupTimeInMicro = 10000000;
int channelCount = 2;

static volatile int keepRunning = 1;

static volatile int runSample = 0;

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
  int bufferSampleDelay = 8192;
  int stepTriggerTimeInMicro = 5;
  int synthWarmupTimeInMicro = 10000000;
  int channelCount = 2;

  MSGPACK_DEFINE_MAP(
    timestamp,

    channelCount,
    startFrequency,
    stepFrequency,
    frequencyCount,
    intermediateFreq,
    transmitPower,
    loPower,
    sampleCount,
    settlingTimeInMicro,
    bufferSampleDelay,
    stepTriggerTimeInMicro,
    synthWarmupTimeInMicro
  )
};

ParametersMessage lastParametersMessage;

void updateParameters(ParametersMessage parametersMessage) {
  lastParametersMessage = parametersMessage;

  channelCount = parametersMessage.channelCount;

  startFrequency = parametersMessage.startFrequency;
  stepFrequency = parametersMessage.stepFrequency;
  frequencyCount = parametersMessage.frequencyCount;
  intermediateFreq = parametersMessage.intermediateFreq;

  transmitPower = parametersMessage.transmitPower;
  loPower = parametersMessage.loPower;
  sampleCount = parametersMessage.sampleCount;
  settlingTimeInMicro = parametersMessage.settlingTimeInMicro;
  bufferSampleDelay = parametersMessage.bufferSampleDelay;
  stepTriggerTimeInMicro = parametersMessage.stepTriggerTimeInMicro;
  synthWarmupTimeInMicro = parametersMessage.synthWarmupTimeInMicro;
}

void updateParametersFromString(string parametersString) {
  msgpack::object_handle oh = msgpack::unpack(parametersString.data(), parametersString.size());

  msgpack::object deserialized = oh.get();

  std::cout << "updated parameters: " << deserialized << std::endl;

  ParametersMessage parametersMessage;
  deserialized.convert(parametersMessage);

  updateParameters(parametersMessage);
}

Redis* redis;

struct RadarProfile {
  uint32_t timestamp;

  float data[];
};

void setFrequency(SynthSerialPort& rfSource, float frequency, float intermediateFreq) {
  std::cout << "setting frequency: " << std::to_string(float(frequency)) << ", " << std::to_string(float(frequency + intermediateFreq)) << std::endl;

  rfSource.SendCommand("Source 1");
  std::stringstream source1FreqSs;
  source1FreqSs << std::fixed << std::setprecision(6) << frequency;
  rfSource.SendCommand("Frequency " + source1FreqSs.str() + " M");

  rfSource.SendCommand("Source 2");
  std::stringstream source2FreqSs;
  source2FreqSs << std::fixed << std::setprecision(6) << (frequency + intermediateFreq);
  rfSource.SendCommand("Frequency " + source2FreqSs.str() + " M");

  rfSource.SendCommand("Source 1");
}

void enableExcitation(SynthSerialPort& rfSource, int transmitPower, int loPower) {
  float transmitAtten = 15.0 - transmitPower;
  float loAtten = 15 - loPower;

  rfSource.SendCommand("Source 1");

  std::stringstream transmitPowerSs;
  transmitPowerSs << std::fixed << std::setprecision(2) << transmitAtten;

  rfSource.SendCommand("ATTenuator " + transmitPowerSs.str());

  rfSource.SendCommand("oen 1");
  rfSource.SendCommand("pdn 1");

  rfSource.SendCommand("Source 2");

  std::stringstream loPowerSs;
  loPowerSs << std::fixed << std::setprecision(2) << loAtten;

  rfSource.SendCommand("ATTenuator " + loPowerSs.str());

  rfSource.SendCommand("oen 1");
  rfSource.SendCommand("pdn 1");

  rfSource.SendCommand("Source 1");
}

void queryFrequency(SynthSerialPort& rfSource) {
  rfSource.SendCommand("Source 1");
  std::string channel0 = rfSource.SendCommand("Frequency?");

  std::cout << "Channel 0 Frequency: " << channel0 << std::endl;

  rfSource.SendCommand("Source 2");
  std::string channel1 = rfSource.SendCommand("Frequency?");

  std::cout << "Channel 1 Frequency: " << channel1 << std::endl;

  rfSource.SendCommand("Source 1");
}

void queryPower(SynthSerialPort& rfSource) {
  rfSource.SendCommand("Source 1");
  std::string channel0 = rfSource.SendCommand("ATTenuator?");

  std::cout << "Channel 0 Attenutation (power +15 dbM): " << channel0 << std::endl;

  rfSource.SendCommand("Source 2");
  std::string channel1 = rfSource.SendCommand("ATTenuator?");

  std::cout << "Channel 1 Attenutation (power +15 dbM): " << channel1 << std::endl;

  rfSource.SendCommand("Source 1");
}


void disableExcitation(SynthSerialPort& rfSource) {
  rfSource.SendCommand("Source 1");
  rfSource.SendCommand("oen 0");
  rfSource.SendCommand("pdn 0");

  rfSource.SendCommand("Source 2");
  rfSource.SendCommand("oen 0");
  rfSource.SendCommand("pdn 0");

  rfSource.SendCommand("Source 1");
}

void intHandler(int dummy) {
	if (keepRunning == 0) {
		printf("shutting down!\n");
    //disableExcitation();

    rp_Release();

    exit(-1);
	}
	keepRunning = 0;
}

void setupSweep(
  SynthSerialPort& rfSource,
  float startFrequency,
  float stepFrequency,
  int frequencyCount,
  float intermediateFreq,
  float stepTimeInMs) {

  std::cout << "Step time in ms: " << stepTimeInMs << std::endl;

  rfSource.SendCommand("Source 1");
  rfSource.SendCommand("Mode SWEEP");

  std::stringstream rateInMsSs;
  rateInMsSs << std::fixed << std::setprecision(3) << stepTimeInMs;
  rfSource.SendCommand("RATE " + rateInMsSs.str());

  rfSource.SendCommand("TMODE EXTSTEP");

  std::stringstream startFreqSs;
  startFreqSs << std::fixed << std::setprecision(6) << startFrequency;
  rfSource.SendCommand("START " + startFreqSs.str());

  std::stringstream stopFreqSs;
  stopFreqSs << std::fixed << std::setprecision(6) << (startFrequency + (stepFrequency * (frequencyCount - 1)));
  rfSource.SendCommand("STOP " + stopFreqSs.str());

  std::stringstream stepFreqSs;
  stepFreqSs << std::fixed << std::setprecision(6) << stepFrequency;
  rfSource.SendCommand("STEP " + stepFreqSs.str() + " MHz");

  std::cout << "Step frequency: " << stepFreqSs.str() << std::endl;

  rfSource.SendCommand("Source 2");
  rfSource.SendCommand("Mode SWEEP");

  std::stringstream rateInMsSs2;
  rateInMsSs2 << std::fixed << std::setprecision(3) << stepTimeInMs;
  rfSource.SendCommand("RATE " + rateInMsSs2.str());

  rfSource.SendCommand("TMODE EXTSTEP");

  std::stringstream startFreqSs2;
  startFreqSs2 << std::fixed << std::setprecision(6) << (startFrequency + intermediateFreq);
  rfSource.SendCommand("START " + startFreqSs2.str());

  std::stringstream stopFreqSs2;
  stopFreqSs2 << std::fixed << std::setprecision(6) << (startFrequency + intermediateFreq) + (stepFrequency * (frequencyCount - 1));
  rfSource.SendCommand("STOP " + stopFreqSs2.str());

  std::stringstream stepFreqSs2;
  stepFreqSs2 << std::fixed << std::setprecision(6) << stepFrequency;
  rfSource.SendCommand("STEP " + stepFreqSs2.str() + " MHz");

  rfSource.SendCommand("Source 1");
}

void setupSweepTable(
    SynthSerialPort& rfSource,
    nc::NdArray<float> sweepRange,
    float intermediateFrequency,
    float stepTimeInMs) {

  std::cout << "Step time in ms: " << stepTimeInMs << std::endl;

  int frequencyCount = sweepRange.size();

  for(int c = 0; c < CHANNEL_COUNT; c++) {

    rfSource.SendCommand("Source " + std::to_string(c + 1));
    rfSource.SendCommand("Mode LIST");

    std::stringstream rateInMsSs;
    rateInMsSs << std::fixed << std::setprecision(1);
    rfSource.SendCommand("RATE " + rateInMsSs.str());

    rfSource.SendCommand("TMODE EXTSTEP");

    float transmitAtten = 15.0 - transmitPower;
    float loAtten = 15.0 - loPower;

    int listIndex = 0;
    for(int i = 0; i < (frequencyCount * 2); i++) {

      listIndex++;

      int j = i;

      if(i > (frequencyCount - 1)) {
        j = (frequencyCount - 1) - (i - frequencyCount);
      }

      std::stringstream sweepTableEntry;

      sweepTableEntry
        << "List"
        << " "
        << std::to_string(listIndex)
        << " "
        << std::fixed << std::setprecision(6) << sweepRange[j] + (c == 1 ? intermediateFrequency : 0.0)
        << " "
        << std::fixed << std::setprecision(1) << (c == 0 ? transmitAtten : loAtten);

      rfSource.SendCommand(sweepTableEntry.str());
    }
  }
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

    bool clientFault = false;
    while(keepRunning && !clientFault) {

      printf("got here\n");

      struct RadarProfile* profile = nullptr;

      while(!profileBuffer.remove(profile)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        //sched_yield();

        int bytes_read = read(sock_client);
        printf("Bytes read from socket %s", bytes_read)

        if (bytes_read > 0) {
          printf("Read bytes, setting run sample flag")
          runSample = 1;
        }
      }

      memcpy(data, profile, len);

      size_t offset = 0;
      ssize_t result;
      while (offset < len) {
        result = send(sock_client, data + offset, len - offset, 0);
        if (result < 0) {
          printf("Error sending!\n");
          clientFault = true;
          break;
        }

        offset += result;
      }
    }

    close(sock_client);
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

  auto timestamp = redis->get(STARTUP_TIMESTAMP_KEY); // This is a DISASTER!  Use local clocks!!!!
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

  nc::NdArray<float> frequencyRange = nc::linspace<float>(startFrequency, startFrequency + ((frequencyCount - 1) * stepFrequency), frequencyCount);
  frequencyRange.print();

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

  rp_DpinSetDirection(stepPin, RP_OUT);
  rp_DpinSetState(stepPin, RP_LOW);

  rp_dpin_t channel0LockDetect = RP_DIO7_N;
  rp_dpin_t channel1LockDetect = RP_DIO7_P;

  rp_DpinSetDirection(channel0LockDetect, RP_IN);
  rp_DpinSetDirection(channel1LockDetect, RP_IN);

  sampleTimeInMicro = ((1 / ADC_SAMPLE_RATE) * sampleCount * 1000000);
  //rp_AcqSetTriggerDelayNs(sampleTimeInMicro * 1000);

  std::cout << "ADC sample rate: " << ADC_SAMPLE_RATE << std::endl;

  std::cout << "sample time in micro: " << sampleTimeInMicro << std::endl;
  std::cout << "settling time in micro: " << settlingTimeInMicro << std::endl;

  SynthSerialPort rfSource;
  rfSource.Open("/dev/ttyUSB0");
  rfSource.SetBaudRate(BaudRate::BAUD_9600);
  rfSource.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
  rfSource.SetParity(Parity::PARITY_NONE);
  rfSource.SetStopBits(StopBits::STOP_BITS_1);
  rfSource.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

  rfSource.SendCommand("BAUD 115200");
  rfSource.Close();

  rfSource.Open("/dev/ttyUSB0");
  rfSource.SetBaudRate(BaudRate::BAUD_115200);
  rfSource.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
  rfSource.SetParity(Parity::PARITY_NONE);
  rfSource.SetStopBits(StopBits::STOP_BITS_1);
  rfSource.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

  sleep(2);
  tcflush(rfSource.GetFileDescriptor(),TCIOFLUSH);

  rfSource.SendCommand("RST");

  std::string status = rfSource.SendCommand("STATUS");
  std::cout << "Synth Status: " << status << std::endl;

  //setFrequency(frequencyRange[0], intermediateFreq);

  /*std::cout << "before sweep setup:" << std::endl;
  enableExcitation(transmitPower, loPower);
  queryFrequency(rfSource);
  queryPower(rfSource);*/

  enableExcitation(rfSource, transmitPower, loPower);
  std::cout << "Warming up RF PLL..." << std::endl;
  setFrequency(rfSource, frequencyRange[0], intermediateFreq);
  std::this_thread::sleep_for(std::chrono::microseconds(synthWarmupTimeInMicro));

  setupSweep(
    rfSource,
    startFrequency,
    stepFrequency,
    frequencyCount,
    intermediateFreq,
    (stepTriggerTimeInMicro + settlingTimeInMicro + sampleTimeInMicro) / 1000
  );

  std::cout << "after sweep setup:" << std::endl;

  //enableExcitation(transmitPower, loPower);
  queryFrequency(rfSource);
  queryPower(rfSource);


  for(int i = 0; i < PROFILE_BUFFER_SIZE; i++) {
    struct RadarProfile* profile = (RadarProfile *)calloc(1, sizeof(struct RadarProfile) + (sampleCount * frequencyCount * sizeof(float) * CHANNEL_COUNT));

    profileBuffers[i] = profile;
  }

  int currentBufferIndex = 0;
  bool sweepDirectionUp = true;

  //setFrequency(frequencyRange[0], intermediateFreq);
  //std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  while(keepRunning) {
    int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

    if (!runSample) {
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
        continue;
    }
    if(currentBufferIndex > (PROFILE_BUFFER_SIZE - 1)) currentBufferIndex = 0;

    profileBuffers[currentBufferIndex]->timestamp = uint32_t(currentMicro - startupTimestamp);

    //std::cout << "timestamp is: " << profileBuffers[currentBufferIndex]->timestamp << std::endl;

    //rp_DpinSetState(stepPin, RP_HIGH);

    for(int i = 0; i < frequencyCount; i++) {
      //setFrequency(rfSource, frequencyRange[i], intermediateFreq);

      //queryFrequency(rfSource);
      //queryPower(rfSource);


      //int64_t startSampleTimeMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

      rp_AcqStart();

      //sleep(0.5);
      //std::this_thread::sleep_for(std::chrono::microseconds(sampleTimeInMicro));

      rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);
      rp_acq_trig_state_t state = RP_TRIG_STATE_WAITING;

      while(1){
        rp_AcqGetTriggerState(&state);
        if(state == RP_TRIG_STATE_TRIGGERED){
          break;
        }
      }

      /*bool fillState = false;
      while(!fillState) {
        rp_AcqGetBufferFillState(&fillState);
      }*/

      //std::cout << "triggered! " << i << std::endl;
      //rp_AcqStart();

      //usleep(settlingTimeInMicro);

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

      /*if(i == 1) {
        printf("insert\n");
        profileBuffer.insert(&profileBuffers[currentBufferIndex]);
        sleep(100000);
      }*/

      rp_DpinSetState(stepPin, RP_HIGH);
      std::this_thread::sleep_for(std::chrono::microseconds(stepTriggerTimeInMicro));
      rp_DpinSetState(stepPin, RP_LOW);

      std::this_thread::sleep_for(std::chrono::microseconds(settlingTimeInMicro));

      //int64_t endSampleTimeMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    }

    profileBuffer.insert(&profileBuffers[currentBufferIndex]);
    //sweepDirectionUp = !sweepDirectionUp;

    currentBufferIndex++;
    int64_t endTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

    runSample = 0; // don't run another sample until tcp socket triggers

    printf("Sweep Done, took %lld microseconds\n", endTime - currentMicro);
  }

  //disableExcitation();

  rp_Release();

  return 0;
}
