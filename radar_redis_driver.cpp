#include <stdio.h>
#include <string.h>
#include "iostream"
#include <sstream>

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

#include <msgpack.hpp>

#include <chrono>
#include <thread>

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

int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  startupTimestamp = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  printf("Startup timestamp: %s \n", std::to_string(startupTimestamp).c_str());

	return 0;
}
