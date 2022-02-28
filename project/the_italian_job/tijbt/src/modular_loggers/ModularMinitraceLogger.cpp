/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>

// project
#include <tijbt/modular_loggers/ModularMinitraceLogger.hpp>

namespace tijbt
{
ModularMinitraceLogger::ModularMinitraceLogger(const char* filename_json)
  : filename_json_{ filename_json }
{
}

void ModularMinitraceLogger::flush()
{
  logger_->flush();
}

void ModularMinitraceLogger::attach(BT::Tree& tree)
{
  logger_ = std::make_unique<BT::MinitraceLogger>(tree, filename_json_.c_str());
}

}  // namespace tijbt
