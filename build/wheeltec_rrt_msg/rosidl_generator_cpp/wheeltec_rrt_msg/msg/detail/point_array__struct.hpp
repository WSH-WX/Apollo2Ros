// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wheeltec_rrt_msg:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__STRUCT_HPP_
#define WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__msg__PointArray __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__msg__PointArray __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PointArray_
{
  using Type = PointArray_<ContainerAllocator>;

  explicit PointArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit PointArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _points_type points;

  // setters for named parameter idiom
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__msg__PointArray
    std::shared_ptr<wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__msg__PointArray
    std::shared_ptr<wheeltec_rrt_msg::msg::PointArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PointArray_ & other) const
  {
    if (this->points != other.points) {
      return false;
    }
    return true;
  }
  bool operator!=(const PointArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PointArray_

// alias to use template instance with default allocator
using PointArray =
  wheeltec_rrt_msg::msg::PointArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wheeltec_rrt_msg

#endif  // WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__STRUCT_HPP_
