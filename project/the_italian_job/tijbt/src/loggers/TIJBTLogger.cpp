/* Copyright [2022] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <algorithm>

// project
#include <tijbt/loggers/TIJBTLogger.hpp>
#include <tijlogger/logger.hpp>

namespace tijbt
{
TIJBTLogger::TIJBTLogger(const BT::Tree& tree) : StatusChangeLogger(tree.rootNode())
{
}

void TIJBTLogger::callback(BT::Duration timestamp, const BT::TreeNode& node,
                           BT::NodeStatus prev_status, BT::NodeStatus status)
{
  using namespace std::chrono;  // NOLINT(build/namespaces)

  constexpr const char* whitespaces = "                         ";
  constexpr const size_t ws_count = 25;

  double since_epoch = duration<double>(timestamp).count();
  DEBUG("[{:.3f}]: {}{} {} -> {}", since_epoch, node.name(),
        &whitespaces[std::min(ws_count, node.name().size())], toStr(prev_status, true),
        toStr(status, true));
}

void TIJBTLogger::flush()
{
}

}  // namespace tijbt
