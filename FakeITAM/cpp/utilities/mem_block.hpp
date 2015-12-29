//
//  mem_block.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/6.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_UTILITIES_MEM_BLOCK_HPP_
#define FAKEITAM_CPP_UTILITIES_MEM_BLOCK_HPP_

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <istream>

namespace fakeitam {
namespace utility {

enum MemDevice { NONE, MEM_CPU, MEM_METAL, MEM_CPU_AND_METAL };
enum BlockStatus { UNALLOCATED, ALLOCATED };

template<typename T>
class MemBlock {
 public:
  MemBlock();
  MemBlock(const MemBlock& other);
  MemBlock(int element_n, MemDevice type);
  ~MemBlock();

  int element_n() const { return element_n_; }
  int byte_capacity() const { return byte_capacity_; }
  MemDevice device() const { return device_; }
  BlockStatus status() const { return status_; }

  T* GetData() throw(std::runtime_error);
  const T* GetData() const throw(std::runtime_error);
  void ResetData(unsigned char value = 0);

  int CopyBytesFrom(const void* src, int byte_n) throw(std::runtime_error);
  int CopyBytesFromStream(std::istream& is, int byte_n) throw(std::runtime_error);
  int CopyBytesTo(void* dest, int byte_n) const throw(std::runtime_error);

  T& operator[](int id) throw(std::runtime_error);
  const T& operator[](int id) const throw(std::runtime_error);

  static bool Allocate(MemBlock* dest, int element_n, MemDevice type);
  static bool Deallocate(MemBlock* block);

 private:
  MemDevice device_;
  BlockStatus status_;
  int element_n_;
  int byte_capacity_;
  T* data_cpu_;
  T* data_metal_;

  MemBlock& operator=(const MemBlock&);
};

template<typename T>
MemBlock<T>::MemBlock()
    : device_(NONE), status_(UNALLOCATED),
      element_n_(0), byte_capacity_(0),
      data_cpu_(nullptr), data_metal_(nullptr) {}

template<typename T>
MemBlock<T>::MemBlock(int element_n, MemDevice type)
    : device_(NONE), status_(UNALLOCATED),
      element_n_(0), byte_capacity_(0),
      data_cpu_(nullptr), data_metal_(nullptr) {
  Allocate(this, element_n, type);
}

template<typename T>
MemBlock<T>::MemBlock(const MemBlock& other)
    : MemBlock(other.element_n_, other.device_) {
  CopyBytesFrom(other.GetData(), other.byte_capacity_);
}

template<typename T>
MemBlock<T>::~MemBlock() {
  if (status_ == ALLOCATED)
    Deallocate(this);
}

/* TODO Metal */
template<typename T>
bool MemBlock<T>::Allocate(MemBlock* dest, int element_n, MemDevice type) {
  switch (type) {
    case MEM_CPU:
      dest->data_cpu_ = new T[element_n];
      dest->status_ = ALLOCATED;
      break;
    
    case MEM_METAL:
    case MEM_CPU_AND_METAL:
    default: return false;
  }

  dest->element_n_ = element_n;
  dest->byte_capacity_ = element_n * sizeof(T);
  dest->device_ = type;

  return true;
}

/* TODO Metal */
template<typename T>
bool MemBlock<T>::Deallocate(MemBlock* block) {
  if (block->status_ == UNALLOCATED)
    return true;

  switch (block->device_) {
    case MEM_CPU:
      delete [] block->data_cpu_;
      block->data_cpu_ = nullptr;
      block->status_ = UNALLOCATED;
      break;
    
    case MEM_METAL:
    case MEM_CPU_AND_METAL:
    default: return false;
  }

  block->element_n_ = 0;
  block->byte_capacity_ = 0;
  block->device_ = NONE;

  return true;
}

template<typename T>
T* MemBlock<T>::GetData() throw(std::runtime_error) {
  if (status_ == UNALLOCATED)
    throw std::runtime_error("Memory block unallocated!");

  if (device_ == MEM_CPU_AND_METAL || device_ == MEM_CPU)
    return data_cpu_;
  else if (device_ == MEM_METAL)
    return data_metal_;
  else
    throw std::runtime_error("Unknown meomry device type!");
}

template<typename T>
const T* MemBlock<T>::GetData() const throw(std::runtime_error) {
  if (status_ == UNALLOCATED)
    throw std::runtime_error("Memory block unallocated!");

  if (device_ == MEM_CPU_AND_METAL || device_ == MEM_CPU)
    return data_cpu_;
  else if (device_ == MEM_METAL)
    return data_metal_;
  else
    throw std::runtime_error("Unknown meomry device type!");
}

/* TODO Metal */
template<typename T>
int MemBlock<T>::CopyBytesFrom(const void* src, int byte_n)
    throw(std::runtime_error) {
  if (status_ == UNALLOCATED)
    throw std::runtime_error("Memory block unallocated!");
  if (byte_n > byte_capacity_)
    throw std::runtime_error("Memory block overflowed");

  switch (device_) {
    case MEM_CPU:
      memset(data_cpu_, 0, byte_capacity_);
      memcpy(data_cpu_, src, byte_n);
      break;
    
    case MEM_METAL:
    case MEM_CPU_AND_METAL:
    default:
      throw std::runtime_error("Unknown meomry device type!");
  }

  return byte_n;
}

/* TODO Metal */
template<typename T>
int MemBlock<T>::CopyBytesFromStream(std::istream& is, int byte_n)
    throw(std::runtime_error) {
  if (status_ == UNALLOCATED)
    throw std::runtime_error("Memory block unallocated!");
  if (byte_n > byte_capacity_)
    throw std::runtime_error("Memory block overflowed");

  switch (device_) {
    case MEM_CPU:
      memset(data_cpu_, 0, byte_capacity_);
      is.read((char*)data_cpu_, byte_n);
      break;
    
    case MEM_METAL:
    case MEM_CPU_AND_METAL:
    default:
      throw std::runtime_error("Unknown meomry device type!");
  }

  return byte_n;
}

/* TODO Metal */
template<typename T>
int MemBlock<T>::CopyBytesTo(void* dest, int byte_n) const
    throw(std::runtime_error) {
  if (status_ == UNALLOCATED)
    throw std::runtime_error("Memory block unallocated!");
  if (byte_n > byte_capacity_)
    throw std::runtime_error("Memory block overflowed");

  switch (device_) {
    case MEM_CPU:
      memcpy(dest, data_cpu_, byte_n);
      break;
    
    case MEM_METAL:
    case MEM_CPU_AND_METAL:
    default:
      throw std::runtime_error("Unknown meomry device type!");
  }

  return byte_n;
}

/* TODO Metal */
template<typename T>
void MemBlock<T>::ResetData(unsigned char value) {
  if (status_ == UNALLOCATED)
    throw std::runtime_error("Memory block unallocated!");

  switch (device_) {
    case MEM_CPU:
      memset(data_cpu_, value, byte_capacity_);
      break;
    
    case MEM_METAL:
    case MEM_CPU_AND_METAL:
    default:
      throw std::runtime_error("Unknown meomry device type!");
  }
}

/* TODO Metal */
template<typename T>
T& MemBlock<T>::operator[](int id) throw(std::runtime_error) {
  if (status_ == UNALLOCATED)
    throw std::runtime_error("Memory block unallocated!");
  if (id >= element_n_)
    throw std::runtime_error("Memory block overflowed!");

  switch (device_) {
    case MEM_CPU:
      return data_cpu_[id];
    
    case MEM_METAL:
    case MEM_CPU_AND_METAL:
    default:
      throw std::runtime_error("Unknown meomry device type!");
  }
}

/* TODO Metal */
template<typename T>
const T& MemBlock<T>::operator[](int id) const throw(std::runtime_error) {
  if (status_ == UNALLOCATED)
    throw std::runtime_error("Memory block unallocated!");
  if (id >= element_n_)
    throw std::runtime_error("Memory block overflowed!");

  switch (device_) {
    case MEM_CPU:
      return data_cpu_[id];
    
    case MEM_METAL:
    case MEM_CPU_AND_METAL:
    default:
      throw std::runtime_error("Unknown meomry device type!");
  }
}

}
}

#endif  /* FAKEITAM_CPP_UTILITIES_MEM_BLOCK_HPP_ */
