// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from phasespace_msgs:msg/Rigids.idl
// generated code does not contain a copyright notice

#ifndef PHASESPACE_MSGS__MSG__DETAIL__RIGIDS__STRUCT_HPP_
#define PHASESPACE_MSGS__MSG__DETAIL__RIGIDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'rigids'
#include "phasespace_msgs/msg/detail/rigid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__phasespace_msgs__msg__Rigids __attribute__((deprecated))
#else
# define DEPRECATED__phasespace_msgs__msg__Rigids __declspec(deprecated)
#endif

namespace phasespace_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Rigids_
{
  using Type = Rigids_<ContainerAllocator>;

  explicit Rigids_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Rigids_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _rigids_type =
    std::vector<phasespace_msgs::msg::Rigid_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<phasespace_msgs::msg::Rigid_<ContainerAllocator>>>;
  _rigids_type rigids;

  // setters for named parameter idiom
  Type & set__rigids(
    const std::vector<phasespace_msgs::msg::Rigid_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<phasespace_msgs::msg::Rigid_<ContainerAllocator>>> & _arg)
  {
    this->rigids = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    phasespace_msgs::msg::Rigids_<ContainerAllocator> *;
  using ConstRawPtr =
    const phasespace_msgs::msg::Rigids_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<phasespace_msgs::msg::Rigids_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<phasespace_msgs::msg::Rigids_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      phasespace_msgs::msg::Rigids_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<phasespace_msgs::msg::Rigids_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      phasespace_msgs::msg::Rigids_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<phasespace_msgs::msg::Rigids_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<phasespace_msgs::msg::Rigids_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<phasespace_msgs::msg::Rigids_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__phasespace_msgs__msg__Rigids
    std::shared_ptr<phasespace_msgs::msg::Rigids_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__phasespace_msgs__msg__Rigids
    std::shared_ptr<phasespace_msgs::msg::Rigids_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Rigids_ & other) const
  {
    if (this->rigids != other.rigids) {
      return false;
    }
    return true;
  }
  bool operator!=(const Rigids_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Rigids_

// alias to use template instance with default allocator
using Rigids =
  phasespace_msgs::msg::Rigids_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace phasespace_msgs

#endif  // PHASESPACE_MSGS__MSG__DETAIL__RIGIDS__STRUCT_HPP_
