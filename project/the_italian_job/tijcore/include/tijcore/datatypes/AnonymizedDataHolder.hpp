/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>

namespace tijcore
{
class AnonymizedDataHolder
{
public:
  AnonymizedDataHolder() = default;

  AnonymizedDataHolder(AnonymizedDataHolder&&) = default;
  AnonymizedDataHolder& operator=(AnonymizedDataHolder&&) = default;

  AnonymizedDataHolder(const AnonymizedDataHolder& original)
  {
    if (original.contents_)
    {
      contents_ = original.contents_->clone();
    }
  }

  AnonymizedDataHolder& operator=(const AnonymizedDataHolder& original)
  {
    if (!contents_)
    {
      throw std::invalid_argument{ "Invalid AnonymizedDataHolder assignment" };
    }
    contents_ = original.contents_->clone();
    return *this;
  }

  template <typename T>
  AnonymizedDataHolder(const T& part_info)  // NOLINT(runtime/explicit)
    : contents_{ std::make_unique<DataHolderImpl<T>>(part_info) }
  {
  }

  template <typename T>
  AnonymizedDataHolder& operator=(const T& data)
  {
    contents_ = std::make_unique<DataHolderImpl<T>>(data);
    return *this;
  }

  template <typename T>
  const bool is() const
  {
    return nullptr != dynamic_cast<DataHolderImpl<T>*>(contents_.get());
  }

  template <typename T>
  const T& as() const
  {
    if (!is<T>())
    {
      throw std::invalid_argument{ "Invalid AnonymizedDataHolder cast" };
    }
    return *dynamic_cast<DataHolderImpl<T>*>(contents_.get());
  }

  template <typename T>
  T& as()
  {
    return *dynamic_cast<DataHolderImpl<T>*>(contents_.get());
  }

private:
  struct DataHolderBase
  {
    using Ptr = std::unique_ptr<DataHolderBase>;
    virtual ~DataHolderBase() = default;
    virtual Ptr clone() const = 0;
  };

  template <typename T>
  struct DataHolderImpl : public DataHolderBase, public T
  {
    DataHolderImpl(const T& data) : T{ data }  // NOLINT(runtime/explicit)
    {
    }

    Ptr clone() const override
    {
      return std::make_unique<DataHolderImpl<T>>(*this);
    }
  };

  DataHolderBase::Ptr contents_;
};

}  // namespace tijcore
