// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wheeltec_rrt_msg:action/PickColouredBox.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__ACTION__DETAIL__PICK_COLOURED_BOX__STRUCT_H_
#define WHEELTEC_RRT_MSG__ACTION__DETAIL__PICK_COLOURED_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'box_colour'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/PickColouredBox in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_Goal
{
  rosidl_runtime_c__String box_colour;
} wheeltec_rrt_msg__action__PickColouredBox_Goal;

// Struct for a sequence of wheeltec_rrt_msg__action__PickColouredBox_Goal.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_Goal__Sequence
{
  wheeltec_rrt_msg__action__PickColouredBox_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__action__PickColouredBox_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
#include "std_msgs/msg/detail/empty__struct.h"

/// Struct defined in action/PickColouredBox in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_Result
{
  std_msgs__msg__Empty result;
} wheeltec_rrt_msg__action__PickColouredBox_Result;

// Struct for a sequence of wheeltec_rrt_msg__action__PickColouredBox_Result.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_Result__Sequence
{
  wheeltec_rrt_msg__action__PickColouredBox_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__action__PickColouredBox_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/PickColouredBox in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} wheeltec_rrt_msg__action__PickColouredBox_Feedback;

// Struct for a sequence of wheeltec_rrt_msg__action__PickColouredBox_Feedback.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_Feedback__Sequence
{
  wheeltec_rrt_msg__action__PickColouredBox_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__action__PickColouredBox_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"

/// Struct defined in action/PickColouredBox in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  wheeltec_rrt_msg__action__PickColouredBox_Goal goal;
} wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request;

// Struct for a sequence of wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__Sequence
{
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PickColouredBox in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response;

// Struct for a sequence of wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__Sequence
{
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PickColouredBox in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request;

// Struct for a sequence of wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__Sequence
{
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"

/// Struct defined in action/PickColouredBox in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response
{
  int8_t status;
  wheeltec_rrt_msg__action__PickColouredBox_Result result;
} wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response;

// Struct for a sequence of wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__Sequence
{
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"

/// Struct defined in action/PickColouredBox in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  wheeltec_rrt_msg__action__PickColouredBox_Feedback feedback;
} wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage;

// Struct for a sequence of wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage.
typedef struct wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__Sequence
{
  wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WHEELTEC_RRT_MSG__ACTION__DETAIL__PICK_COLOURED_BOX__STRUCT_H_
