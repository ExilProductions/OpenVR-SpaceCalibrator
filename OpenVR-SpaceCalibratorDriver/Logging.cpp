#include "Logging.h"
#include <chrono>

FILE *LogFile;

__attribute__((constructor)) void OpenLogFile()
{
	LogFile = fopen("space_calibrator_driver.log", "a");
	if (LogFile == nullptr)
	{
		LogFile = stderr;
	}
}

tm TimeForLog()
{
	auto now = std::chrono::system_clock::now();
	auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value;
	localtime_r(&nowTime, &value);
	return value;
}

void LogFlush()
{
	fflush(LogFile);
}