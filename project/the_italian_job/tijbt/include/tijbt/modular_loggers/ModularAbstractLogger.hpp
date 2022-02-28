/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

#ifndef MODULAR_ABSTRACT_LOGGER_H
#define MODULAR_ABSTRACT_LOGGER_H

// standard library
#include <memory>

// bt
#include <behaviortree_cpp_v3/bt_factory.h>

namespace tijbt
{
class ModularStatusChangeLogger
{
public:
  using Ptr = std::unique_ptr<ModularStatusChangeLogger>;

  virtual ~ModularStatusChangeLogger() = default;

  /** @brief Attaches the logger to a tree and complete initialization. Needs to be called before
   *         calling any other member function. */
  virtual void attach(BT::Tree& tree) = 0;

  /** @brief Flushes the logger output. Behavior undefined if the logger has not been initialized
   *         before calling.*/
  virtual void flush() = 0;
};

}  // namespace tijbt

#endif  // ABSTRACT_LOGGER_H
