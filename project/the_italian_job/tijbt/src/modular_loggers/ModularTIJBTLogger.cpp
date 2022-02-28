/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <memory>

// project
#include <tijbt/loggers/TIJBTLogger.hpp>
#include <tijbt/modular_loggers/ModularTIJBTLogger.hpp>

namespace tijbt
{
void ModularTIJBTLogger::flush()
{
  logger_->flush();
}

void ModularTIJBTLogger::attach(BT::Tree& tree)
{
  logger_ = std::make_unique<TIJBTLogger>(tree);
}

}  // namespace tijbt
