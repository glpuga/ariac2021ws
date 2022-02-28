/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>

// project
#include <tijbt/modular_loggers/ModularStdCoutLogger.hpp>

namespace tijbt
{
void ModularStdCoutLogger::flush()
{
  logger_->flush();
}

void ModularStdCoutLogger::attach(BT::Tree& tree)
{
  logger_ = std::make_unique<BT::StdCoutLogger>(tree);
}

}  // namespace tijbt
