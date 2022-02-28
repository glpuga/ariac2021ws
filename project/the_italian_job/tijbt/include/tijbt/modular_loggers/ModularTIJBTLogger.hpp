/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#ifndef BT_MODULAR_TIJ_BT_LOGGER_H
#define BT_MODULAR_TIJ_BT_LOGGER_H

// standard library
#include <memory>

// bt
#include <tijbt/modular_loggers/ModularAbstractLogger.hpp>

// project
#include <tijbt/loggers/TIJBTLogger.hpp>
namespace tijbt
{
/**
 * @brief Injectable StdCoutLogger logger that can be attached to a tree after creation. Allows
 *        dependency injection-ish mode of adding loggers to trees.
 */
class ModularTIJBTLogger : public ModularStatusChangeLogger
{
public:
  /** @brief Attaches the logger to a tree and complete initialization. Needs to be called before
   *         calling any other member function. */
  void attach(BT::Tree& tree) override;

  /** @brief Flushes the logger output. Behavior undefined if the logger has not been initialized
   *         before calling.*/
  void flush() override;

private:
  std::unique_ptr<BT::StatusChangeLogger> logger_;
};

}  // namespace tijbt

#endif  // BT_COUT_LOGGER_H
