/*
 * logger.h
 *
 *  Created on: Oct 6, 2016
 *      Author: rdu
 */

#ifndef UTILITY_LOGGING_LOGGER_H_
#define UTILITY_LOGGING_LOGGER_H_

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <atomic>

#include "spdlog/spdlog.h"

#define ENABLE_LOGGING

namespace acel {

namespace LoggerHelper{
	std::string GetDefaultLogPath();
}

class CtrlLogger {
private:
	CtrlLogger() = delete;
	CtrlLogger(std::string log_name_prefix, std::string log_save_path);

	// prevent copy or assignment
	CtrlLogger(const CtrlLogger&) = delete;
	CtrlLogger& operator= (const CtrlLogger &) = delete;

public:
	static CtrlLogger& GetLogger(std::string log_name_prefix = "", std::string log_save_path = "");

	// basic functions
	void AddItemNameToEntryHead(std::string name);
	void AddItemDataToEntry(std::string item_name, std::string data_str);
	void AddItemDataToEntry(uint64_t item_id, std::string data_str);

	void PassEntryHeaderToLogger();
	void PassEntryDataToLogger();

	// extra helper functions
	void AddItemDataToEntry(std::string item_name, double data);
	void AddItemDataToEntry(uint64_t item_id, double data);

private:
	std::shared_ptr<spdlog::logger> logger_;

	bool head_added_;
	std::string log_name_prefix_;
	std::string log_save_path_;

	std::map<uint64_t, std::string> entry_names_;
	std::map<std::string, uint64_t> entry_ids_;
	std::atomic<uint64_t> item_counter_;
	std::vector<std::string> item_data_;
};

class CsvLogger {
public:
	CsvLogger() = delete;
	CsvLogger(std::string log_name_prefix, std::string log_save_path);

	// prevent copy or assignment
	CsvLogger(const CsvLogger&) = delete;
	CsvLogger& operator= (const CsvLogger &) = delete;

public:
	// basic functions
	void AddItemNameToEntryHead(std::string name);
	void PassEntryHeaderToLogger();

	template<class... T>
	void LogData(const T&... value)
	{
#ifdef ENABLE_LOGGING
		std::ostringstream o;
		build_string(o, value...);

		std::string data = o.str();
		data.pop_back();

		logger_->info(data);
#else
		return;
#endif
	}

private:
	std::shared_ptr<spdlog::logger> logger_;

	std::string log_name_prefix_;
	std::string log_save_path_;

	std::map<uint64_t, std::string> entry_names_;

	inline void build_string (std::ostream& o) {(void)o; }
	template<class First, class... Rest> inline void build_string (std::ostream& o, const First& value, const Rest&... rest)
	{
		o << std::to_string(value) + ",";
		build_string(o, rest...);
	}
};

class GlobalCsvLogger: public CsvLogger
{
private:
	GlobalCsvLogger() = delete;
	GlobalCsvLogger(std::string prefix, std::string path):CsvLogger(prefix,path){};

	// prevent copy or assignment
	GlobalCsvLogger(const GlobalCsvLogger&) = delete;
	GlobalCsvLogger& operator= (const GlobalCsvLogger &) = delete;

public:
	static GlobalCsvLogger& GetLogger(std::string log_name_prefix = "", std::string log_save_path = "");
};

class EventLogger {
public:
	EventLogger() = delete;
	EventLogger(std::string log_name_prefix, std::string log_save_path);

	// prevent copy or assignment
	EventLogger(const EventLogger&) = delete;
	EventLogger& operator= (const EventLogger &) = delete;

public:
	// basic functions
	template<class... T>
	void LogEvent(const T&... value)
	{
#ifdef ENABLE_LOGGING
		std::ostringstream o;
		build_string(o, value...);

		std::string data = o.str();
		data.pop_back();

		logger_->info(o.str());
#else
		return;
#endif
	}

	// logger wrapper functions
	void LogInfo(std::string msg)
	{
#ifdef ENABLE_LOGGING
		logger_->info(msg);
#endif
	}

	void LogWarn(std::string msg)
	{
#ifdef ENABLE_LOGGING
		logger_->warn(msg);
#endif
	}

	void LogFatal(std::string msg)
	{
#ifdef ENABLE_LOGGING
		logger_->critical(msg);
#endif
	}

private:
	std::shared_ptr<spdlog::logger> logger_;

	inline void build_string (std::ostream& o) {(void)o; }
	template<class First, class... Rest> inline void build_string (std::ostream& o, const First& value, const Rest&... rest)
	{
		o << value;
		o << ",";
		build_string(o, rest...);
	}
};

}

#endif /* UTILITY_LOGGING_LOGGER_H_ */
