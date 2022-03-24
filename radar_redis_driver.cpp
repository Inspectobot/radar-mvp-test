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

#include "rp.h"
#include <sw/redis++/redis++.h>

using namespace std::chrono;

using namespace std;
using namespace sw::redis;

int64_t startupTimestamp;

static volatile int keepRunning = 1;

void intHandler(int dummy) {
	if (keepRunning == 0) {
		exit(-1);
	}
	keepRunning = 0;
}

/*int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  startupTimestamp = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  printf("Startup timestamp: %s \n", std::to_string(startupTimestamp).c_str());

	return 0;
}*/


int main (int argc, char **argv) {
  int unsigned period = 1000000; // uS
  int unsigned led;

  // index of blinking LED can be provided as an argument
  if (argc > 1) {
    led = atoi(argv[1]);
    // otherwise LED 0 will blink
  } else {
    led = 0;
  }
  printf("Blinking LED[%u]\n", led);
  led += RP_LED0;

  // Initialization of API
  if (rp_Init() != RP_OK) {
    fprintf(stderr, "Red Pitaya API init failed!\n");
    return EXIT_FAILURE;
  }

  int unsigned retries = 1000;
  
  rp_dpin_t ledPin = RP_LED0; 
  
  while (retries--){
    rp_DpinSetState(ledPin, RP_HIGH);
    usleep(period/2);
    rp_DpinSetState(ledPin, RP_LOW);
    usleep(period/2);
  }

  // Releasing resources
  rp_Release();

  return EXIT_SUCCESS;
}
