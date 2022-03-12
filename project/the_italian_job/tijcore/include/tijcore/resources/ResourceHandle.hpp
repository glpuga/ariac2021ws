/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <iostream>
#include <memory>
#include <utility>

namespace tijcore
{
template <typename ResourceType>
class ResourceHandle
{
public:
  using ResourceUniquePtr = std::unique_ptr<ResourceType>;
  using ResourceSharedPtr = std::shared_ptr<ResourceType>;

  explicit ResourceHandle(ResourceUniquePtr&& resource)
    : ResourceHandle{ std::shared_ptr(std::move(resource)) }
  {
  }

  explicit ResourceHandle(ResourceSharedPtr resource) : resource_{ std::move(resource) }
  {
    if (!resource_)
    {
      throw std::invalid_argument("ResourceHandle instances need a valid resource pointer");
    }

    if (resource_.use_count() > 1)
    {
      throw std::invalid_argument(
          "The resource pointer cannot be already "
          "shared when creating the ResourceHandle");
    }
  }

  ResourceType* resource()
  {
    return resource_.get();
  }

  const ResourceType* resource() const
  {
    return resource_.get();
  }

  bool allocated() const
  {
    return (resource_.use_count() > 1);
  }

  // TODO(glpuga) test this
  std::size_t allocationCount() const
  {
    return resource_.use_count();
  }

private:
  ResourceSharedPtr resource_;
};

}  // namespace tijcore
