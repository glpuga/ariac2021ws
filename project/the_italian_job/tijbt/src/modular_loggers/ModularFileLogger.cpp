/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>

// project
#include <tijbt/modular_loggers/ModularFileLogger.hpp>

namespace tijbt
{
ModularFileLogger::ModularFileLogger(const char* filename, uint16_t buffer_size)
  : filename_{ filename }, buffer_size_{ buffer_size }
{
}

void ModularFileLogger::flush()
{
  logger_->flush();
}

void ModularFileLogger::attach(BT::Tree& tree)
{
  logger_ = std::make_unique<BT::FileLogger>(tree, filename_.c_str(), buffer_size_);
}

}  // namespace tijbt
