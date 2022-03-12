/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <cstdint>
#include <string>
#include <utility>

// main spdlog header
#include <tijlogger/spdlog/spdlog.h>

// for user defined types; needs to be included after spdlog.h
#include <tijlogger/spdlog/fmt/ostr.h>

#define DEBUG(...)                                                                                 \
  {                                                                                                \
    ::logger::instance().debug(__PRETTY_FUNCTION__, __LINE__, __VA_ARGS__);                        \
  };
#define INFO(...)                                                                                  \
  {                                                                                                \
    ::logger::instance().info(__PRETTY_FUNCTION__, __LINE__, __VA_ARGS__);                         \
  };
#define WARNING(...)                                                                               \
  {                                                                                                \
    ::logger::instance().warning(__PRETTY_FUNCTION__, __LINE__, __VA_ARGS__);                      \
  };
#define ERROR(...)                                                                                 \
  {                                                                                                \
    ::logger::instance().error(__PRETTY_FUNCTION__, __LINE__, __VA_ARGS__);                        \
  };

namespace logger
{
enum class Level
{
  Debug,
  Info,
  Warning,
  Error
};

class LoggerSingleton
{
public:
  LoggerSingleton()
  {
    spdlog::set_pattern("[%H:%M:%S:%e] %^[%L] %t: %v%$");
  }

  void setLevel(const Level level) const
  {
    switch (level)
    {
      case Level::Debug:
        spdlog::set_level(spdlog::level::debug);
        break;
      case Level::Info:
        spdlog::set_level(spdlog::level::info);
        break;
      case Level::Warning:
        spdlog::set_level(spdlog::level::warn);
        break;
      case Level::Error:
        spdlog::set_level(spdlog::level::err);
        break;
    }
  }

  template <typename... T>
  void debug(const std::string_view filename, const int line, T&&... t)
  {
    spdlog::debug(std::forward<T>(t)...);
  }

  template <typename... T>
  void info(const std::string_view filename, const int line, T&&... t)
  {
    spdlog::info(std::forward<T>(t)...);
  }

  template <typename... T>
  void warning(const std::string_view filename, const int line, T&&... t)
  {
    spdlog::warn(std::forward<T>(t)...);
  }

  template <typename... T>
  void error(const std::string_view filename, const int line, T&&... t)
  {
    spdlog::error(std::forward<T>(t)...);
  }
};

inline LoggerSingleton& instance()
{
  static LoggerSingleton instance{};
  return instance;
}

}  // namespace logger
