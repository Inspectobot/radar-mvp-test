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

#include <fcntl.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <errno.h>

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

#define NUMCPP_NO_USE_BOOST 1
#include "NumCpp.hpp"

#define TCP_PORT 1001
#define MAX_CONNECTIONS 1
#define MAX_EVENTS 32
#define RECEIVE_BUFFER_SIZE 16

#define CHANNEL_COUNT 2

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

rp_dpin_t stepPin = RP_DIO5_N;

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

static void epoll_ctl_add(int epfd, int fd, uint32_t events) {
  struct epoll_event ev;
  
  ev.events = events;
  ev.data.fd = fd;
  
  if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) == -1) {
    perror("epoll_ctl()\n");
    exit(1);
  }
}

static void set_sockaddr(struct sockaddr_in *addr) {
  bzero((char *)addr, sizeof(struct sockaddr_in));
  addr->sin_family = AF_INET;
  addr->sin_addr.s_addr = INADDR_ANY;
  addr->sin_port = htons(TCP_PORT);
}

static int setnonblocking(int sockfd) {
  if (fcntl(sockfd, F_SETFD, fcntl(sockfd, F_GETFD, 0) | O_NONBLOCK) == -1) {
    return -1;
  }

  return 0;
}

bool sweepDirectionUp = true;
void captureProfile(RadarProfile* profile) {
  bzero(profile, sizeof(struct RadarProfile) + (sampleCount * frequencyCount * sizeof(float) * CHANNEL_COUNT));

  int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  profile->timestamp = uint32_t(currentMicro - startupTimestamp);

  for(int i = 0; i < frequencyCount; i++) {
    rp_AcqStart();

    rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);
    rp_acq_trig_state_t state = RP_TRIG_STATE_WAITING;

    while(1){
      rp_AcqGetTriggerState(&state);
      if(state == RP_TRIG_STATE_TRIGGERED){
        break;
      }
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

    rp_AcqGetDataV2(bufferTriggerPosition,
      &sampleCount,
      &profile->data[idx0],
      &profile->data[idx1]);

    rp_DpinSetState(stepPin, RP_HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds(stepTriggerTimeInMicro));
    rp_DpinSetState(stepPin, RP_LOW);

    std::this_thread::sleep_for(std::chrono::microseconds(settlingTimeInMicro));
  }

  int64_t endTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  printf("Sweep Done, took %lld microseconds\n", endTime - currentMicro);
}

int main (int argc, char **argv) {
  signal(SIGABRT, intHandler);
  signal(SIGTERM, intHandler);
  signal(SIGINT, intHandler);

  ConnectionOptions redisConnectionOpts;

  redisConnectionOpts.host = "rover";
  redisConnectionOpts.port = 6379;
  redisConnectionOpts.socket_timeout = std::chrono::milliseconds(5);

  bool connectedToRedis = false;

  while(!connectedToRedis) {
    try {
      redis = new Redis(redisConnectionOpts);

      auto timestamp = redis->get(STARTUP_TIMESTAMP_KEY);
      if(timestamp) {
        string timestampString = *timestamp;

        startupTimestamp = atoll(timestampString.c_str());
        connectedToRedis = true;

      } else {
        std::cout << "Rover startup timestamp not set or invalid, check key: " << STARTUP_TIMESTAMP_KEY << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    } catch (const Error &err) {
      std::cout << "Could not connect to redis " << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
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
  tcflush(rfSource.GetFileDescriptor(), TCIOFLUSH);

  rfSource.SendCommand("RST");

  std::string status = rfSource.SendCommand("STATUS");
  std::cout << "Synth Status: " << status << std::endl;

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

  queryFrequency(rfSource);
  queryPower(rfSource);

  size_t len = sizeof(struct RadarProfile) + (sampleCount * frequencyCount * sizeof(float) * CHANNEL_COUNT);
  RadarProfile* profileBuffer = (RadarProfile *)calloc(1, len);
  char *profileData;
  profileData = (char *) calloc(1, len);

  char buf[RECEIVE_BUFFER_SIZE];
  struct sockaddr_in srv_addr;
  struct sockaddr_in cli_addr;
  struct epoll_event events[MAX_EVENTS]; 
  
  int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
  set_sockaddr(&srv_addr);
  bind(listen_sock, (struct sockaddr *)&srv_addr, sizeof(srv_addr));

  setnonblocking(listen_sock);
  listen(listen_sock, MAX_CONNECTIONS);

  std::cout << "Ready to accept connections on port: " << std::to_string(TCP_PORT) << std::endl;

  int epfd = epoll_create(1);
  epoll_ctl_add(epfd, listen_sock, EPOLLIN | EPOLLOUT | EPOLLET);

  socklen_t socklen = sizeof(cli_addr);
  
  while (keepRunning) {
    int nfds = epoll_wait(epfd, events, MAX_EVENTS, -1);
    
    for (int i = 0; i < nfds; i++) {
      
      if (events[i].data.fd == listen_sock) {
        
        int conn_sock = accept(listen_sock, (struct sockaddr *)&cli_addr, &socklen);

        inet_ntop(AF_INET, (char *)&(cli_addr.sin_addr), buf, sizeof(cli_addr));
        
        printf("client connected with %s:%d\n", buf, ntohs(cli_addr.sin_port));

        setnonblocking(conn_sock);
        epoll_ctl_add(epfd, conn_sock, EPOLLIN | EPOLLET | EPOLLRDHUP | EPOLLHUP);
      } else if (events[i].events & EPOLLIN) {
        
        while (keepRunning) {
          
          bzero(buf, sizeof(buf));
          int n = read(events[i].data.fd, buf, sizeof(buf));

          if (n <= 0 /* || errno == EAGAIN */ ) {
            break;
          } else {
            printf("got trigger request: %s\n", buf);

            printf("profile size in bytes %zu\n", len);
            
            int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

            captureProfile(profileBuffer);
            
            bzero(profileData, len);
            memcpy(profileData, profileBuffer, len);
            write(events[i].data.fd, profileData, len);

            int64_t endTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
            printf("Total time to send data, took %lld microseconds\n", endTime - currentMicro);
          }
        }
      } else {
        printf("client connection unexpected, only a single client is supported\n");
      }

      if (events[i].events & (EPOLLRDHUP | EPOLLHUP)) {
        
        printf("client connection closed\n");
        epoll_ctl(epfd, EPOLL_CTL_DEL, events[i].data.fd, NULL);
        close(events[i].data.fd);
        
        continue;
      }
    }
  }

  //disableExcitation();

  rp_Release();

  return 0;
}
