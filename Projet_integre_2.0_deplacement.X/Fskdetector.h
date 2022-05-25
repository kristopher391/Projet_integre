#define DEBUG_FSK_DETECTOR

#define MESSAGE_LENGTH		10
#define SAMPLING_FREQ		15000
#define BIT_FREQ			10
#define OSR					(SAMPLING_FREQ/BIT_FREQ)
#define FSK_MIN_SAMPLES_NB	(2*OSR/3)

int FskDetector(int detLow, int detHigh);
