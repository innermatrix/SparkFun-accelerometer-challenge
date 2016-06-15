// BEGIN TEST CODE
// Stuff needed to run this test outside the Arduino environment
#include <stdint.h>
#include <stdio.h>
#include <math.h>
uint32_t millis();

#include <fstream>
using std::ifstream;

#include <sstream>
using std::istringstream;

#include <string>
using std::string;
using std::getline;
// END TEST CODE


/* These are at-rest readings for X, Y, and Z from the accelerometer. This will vary on the type of your accelerometer.
 *	
 *	With a typical digital accelerometer, X and Y baseline are zero and Z baseline is -ADCRange / AccelRange, where 
 *	ADCRange is the range of the ADC built into the IMU (for a 12-bit ADC, ADCRange is 4096), and AccelRange is the
 *	range of Gs detected by the accelerometer (for a +-2g accelerometer, AccelRange is 4).
 *	
 *	SparkFun data appears to be using a +-4g accelerometer with a 12bit ADC, so zBaseline = -4096/8 = -512
 *	
 *	For an analog accelerometer, all three of these would be non-zero; X and Y baseline would normally be the middle of
 *	the microcontroller's ADC range (512 for most Arduinos), and Z baseline would be (ADCRange / 2 - ADCRange / AccelRange)
 */

const int xBaseline = 0;
const int yBaseline = 0;
const int zBaseline = -512;

/* This is the size of the window used for running peak analysis
 * You want your window to be able to fit between the peaks in your data. In SparkFun's case, we are seeing approximately 
 * 500 samples per second, so a 50-sample window will work for detecting peaks that occur no more than 10 times per second.
 * This seems reasonable for the boxing situation (note that in that situation there are 4 peaks per hit: one from fist-bag
 * impact and three from bag-plate impact)
 */
const int dataWindowSize = 50;

/* In boxing, a single hit results either in a sequence of 4 peaks, or in a sequence of <4 peaks followed by a gap */
const int peaksPerHit = 4;
const int delayAfterHit = 250; // in milliseconds

/* Sensitivity of peak detector. 0 = high sensitivity (too many hits), 20 = low sensitivity (too few hits). Calibrate this to your own needs. */
const float sensitivity = 4;

uint32_t getAccel();
void peakDetected(uint32_t timestamp);
void noPeakDetected(uint32_t timestamp);

void setup() {
}

int windowIndex = 0;
bool startup = true;
uint32_t dataWindow[dataWindowSize];
uint32_t windowSum = 0;
uint32_t windowSumSq = 0;

uint32_t threshold = 0;
uint32_t hitCount = 0;
bool peak = false;

void loop() {
	uint32_t timestamp = millis();
	uint32_t accel = getAccel();
	
	// Don't attempt peak detection during startup period
	if (!startup) {
		bool newPeak = accel > threshold;
		if (!peak && newPeak) {
			peakDetected(timestamp);
		} else {
			noPeakDetected(timestamp);
		}
		peak = newPeak;
	}

	/* Threshold calculation is where half the magic is. We maintain a mean and std dev for the non-peak readings in our window.
	 * Threshold is set to be mean + sensitivity*stddev
	 */
	 if (startup) {
		dataWindow[windowIndex] = accel;
		windowSum += accel;
		windowSumSq += accel * accel;

		windowIndex = (windowIndex + 1) % dataWindowSize;

		if (windowIndex == 0) {
			startup = false;
			uint32_t mean = windowSum / dataWindowSize;
			uint32_t stddev = sqrt(windowSumSq / dataWindowSize);
			threshold = mean + sensitivity * stddev;
		}
	 } else {
		// Don't update threshold while we are mid-peak. This means that slow drift in IMU noise will lead to a threshold change,
		// but peaks will leave it alone
		if (!peak) {
			uint32_t evictedReading = dataWindow[windowIndex];
			windowSum -= evictedReading;
			windowSumSq -= evictedReading * evictedReading;

			dataWindow[windowIndex] = accel;
			windowSum += accel;
			windowSumSq += accel * accel;

			uint32_t mean = windowSum / dataWindowSize;
			uint32_t stddev = sqrt(windowSumSq / dataWindowSize);
			threshold = mean + sensitivity * stddev;

			windowIndex = (windowIndex + 1) % dataWindowSize;
		}
	 }
}

uint32_t lastPeak = 0;
bool inPeakTrain = false;
int peakCount = 0;

/*
 * Hit detection is the other half of the magic. We call it a hit every time we see a train of four peaks,
 * or a peak followed by prolonged lack of peak
 */
void peakDetected(uint32_t timestamp) {
	peakCount = (peakCount + 1) % peaksPerHit;
	if (peakCount == 0) {
		hitCount += 1;
	}
	inPeakTrain = (peakCount > 0);
	lastPeak = timestamp;
}

void noPeakDetected(uint32_t timestamp) {
	if (inPeakTrain && timestamp > lastPeak + delayAfterHit) {
		hitCount += 1;
		inPeakTrain = false;
	}
}

uint32_t testX;
uint32_t testY;
uint32_t testZ;
uint32_t testMillis;

uint32_t getAccel() {
	// BEGIN TEST CODE
	// If you were doing this for real, you would replace this with code to get X, Y, and Z readings from an actual accelerometer
	uint32_t x = testX;
	uint32_t y = testY;
	uint32_t z = testZ;
	// END TEST CODE

	x -= xBaseline;
	y -= yBaseline;
	z -= zBaseline;

	uint32_t accel = x * x + y * y + z * z;
	return sqrt(accel);
}


// BEGIN TEST CODE
int main(int argc, char** argv) {
	if (argc != 2) {
		printf("Need an argument.\n");
		return -1;
	}
	
	ifstream inputFile(argv[1]);
	
	setup();
	
	std::string line;
	while(std::getline(inputFile, line)) {
		istringstream inputLine(line);
		
		string token;

		getline(inputLine, token, ',');
		testMillis = atoi(token.c_str());

		getline(inputLine, token, ',');
		testX = atoi(token.c_str());

		getline(inputLine, token, ',');
		testY = atoi(token.c_str());

		getline(inputLine, token, ',');
		testZ = atoi(token.c_str());
		
		loop();
	}
	
	printf("Test done; %d hits detected\n", hitCount);
}

uint32_t millis() {
	return testMillis;
}

// END TEST CODE
