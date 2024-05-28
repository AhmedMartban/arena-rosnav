// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from map_distance_server:srv/GetDistanceMap.idl
// generated code does not contain a copyright notice

#ifndef MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__STRUCT_HPP_
#define MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__map_distance_server__srv__GetDistanceMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__map_distance_server__srv__GetDistanceMap_Request __declspec(deprecated)
#endif

namespace map_distance_server
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetDistanceMap_Request_
{
  using Type = GetDistanceMap_Request_<ContainerAllocator>;

  explicit GetDistanceMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetDistanceMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__map_distance_server__srv__GetDistanceMap_Request
    std::shared_ptr<map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__map_distance_server__srv__GetDistanceMap_Request
    std::shared_ptr<map_distance_server::srv::GetDistanceMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetDistanceMap_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetDistanceMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetDistanceMap_Request_

// alias to use template instance with default allocator
using GetDistanceMap_Request =
  map_distance_server::srv::GetDistanceMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace map_distance_server


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'info'
#include "nav_msgs/msg/detail/map_meta_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__map_distance_server__srv__GetDistanceMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__map_distance_server__srv__GetDistanceMap_Response __declspec(deprecated)
#endif

namespace map_distance_server
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetDistanceMap_Response_
{
  using Type = GetDistanceMap_Response_<ContainerAllocator>;

  explicit GetDistanceMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    info(_init)
  {
    (void)_init;
  }

  explicit GetDistanceMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _info_type =
    nav_msgs::msg::MapMetaData_<ContainerAllocator>;
  _info_type info;
  using _data_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__info(
    const nav_msgs::msg::MapMetaData_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__map_distance_server__srv__GetDistanceMap_Response
    std::shared_ptr<map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__map_distance_server__srv__GetDistanceMap_Response
    std::shared_ptr<map_distance_server::srv::GetDistanceMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetDistanceMap_Response_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->info != other.info) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetDistanceMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetDistanceMap_Response_

// alias to use template instance with default allocator
using GetDistanceMap_Response =
  map_distance_server::srv::GetDistanceMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace map_distance_server

namespace map_distance_server
{

namespace srv
{

struct GetDistanceMap
{
  using Request = map_distance_server::srv::GetDistanceMap_Request;
  using Response = map_distance_server::srv::GetDistanceMap_Response;
};

}  // namespace srv

}  // namespace map_distance_server

#endif  // MAP_DISTANCE_SERVER__SRV__DETAIL__GET_DISTANCE_MAP__STRUCT_HPP_
