/*
 * test_logging.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: rdu
 */

#include <memory>

#include "logging/logger.h"

using namespace acel;

int main(int argc, char* argv[])
{
	CtrlLogger& ctrl_logger = CtrlLogger::GetLogger("ctrl_log.txt", "/home/rdu");
	ctrl_logger.AddItemNameToEntryHead("x1");
	ctrl_logger.AddItemNameToEntryHead("x2");
	ctrl_logger.AddItemNameToEntryHead("x3");
	ctrl_logger.AddItemNameToEntryHead("x4");
	ctrl_logger.PassEntryHeaderToLogger();

	for(int i = 0; i < 500; i++) {
		ctrl_logger.AddItemDataToEntry("x1", i+1);
		ctrl_logger.AddItemDataToEntry("x2", i+2);
		ctrl_logger.AddItemDataToEntry("x3", i+3);
		ctrl_logger.AddItemDataToEntry("x4", i+4);

		ctrl_logger.PassEntryDataToLogger();
	}

	CsvLogger csv_logger("csv_log", "/home/rdu");

	for(int i = 0; i < 500; i++)
	{
		csv_logger.LogData(i+1, i+0.5, i*100.5);

		// global csv logger can be called anywhere within the process 
		GlobalCsvLogger::GetLogger("global_csv_log", "/home/rdu").LogData(21,34,56);
	}

	EventLogger event_logger("event_log", "/home/rdu");

	for(int i = 0; i < 500; i++)
		event_logger.LogEvent("event1", "event2", i*100.5);
}
