/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#ifndef BT_MODULAR_FILE_LOGGER_H
#define BT_MODULAR_FILE_LOGGER_H

// standard library
#include <memory>
#include <string>

// bt
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

// project
#include <tijbt/modular_loggers/ModularAbstractLogger.hpp>

namespace tijbt
{
/**
 * @brief Injectable FileLogger logger that can be attached to a tree after creation. Allows
 *        dependency injection-ish mode of adding loggers to trees.
 */
class ModularFileLogger : public ModularStatusChangeLogger
{
public:
  explicit ModularFileLogger(const char* filename, uint16_t buffer_size = 10);

  /** @brief Attaches the logger to a tree and complete initialization. Needs to be called before
   *         calling any other member function. */
  void attach(BT::Tree& tree) override;

  /** @brief Flushes the logger output. Behavior undefined if the logger has not been initialized
   *         before calling.*/
  void flush() override;

private:
  std::string filename_;
  uint16_t buffer_size_;

  std::unique_ptr<BT::StatusChangeLogger> logger_;
};

}  // namespace tijbt

#endif  // BT_MODULAR_FILE_LOGGER_H
