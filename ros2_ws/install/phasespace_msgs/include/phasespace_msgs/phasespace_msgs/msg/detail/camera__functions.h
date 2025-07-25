// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from phasespace_msgs:msg/Camera.idl
// generated code does not contain a copyright notice

#ifndef PHASESPACE_MSGS__MSG__DETAIL__CAMERA__FUNCTIONS_H_
#define PHASESPACE_MSGS__MSG__DETAIL__CAMERA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "phasespace_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "phasespace_msgs/msg/detail/camera__struct.h"

/// Initialize msg/Camera message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * phasespace_msgs__msg__Camera
 * )) before or use
 * phasespace_msgs__msg__Camera__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
bool
phasespace_msgs__msg__Camera__init(phasespace_msgs__msg__Camera * msg);

/// Finalize msg/Camera message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
void
phasespace_msgs__msg__Camera__fini(phasespace_msgs__msg__Camera * msg);

/// Create msg/Camera message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * phasespace_msgs__msg__Camera__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
phasespace_msgs__msg__Camera *
phasespace_msgs__msg__Camera__create();

/// Destroy msg/Camera message.
/**
 * It calls
 * phasespace_msgs__msg__Camera__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
void
phasespace_msgs__msg__Camera__destroy(phasespace_msgs__msg__Camera * msg);

/// Check for msg/Camera message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
bool
phasespace_msgs__msg__Camera__are_equal(const phasespace_msgs__msg__Camera * lhs, const phasespace_msgs__msg__Camera * rhs);

/// Copy a msg/Camera message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
bool
phasespace_msgs__msg__Camera__copy(
  const phasespace_msgs__msg__Camera * input,
  phasespace_msgs__msg__Camera * output);

/// Initialize array of msg/Camera messages.
/**
 * It allocates the memory for the number of elements and calls
 * phasespace_msgs__msg__Camera__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
bool
phasespace_msgs__msg__Camera__Sequence__init(phasespace_msgs__msg__Camera__Sequence * array, size_t size);

/// Finalize array of msg/Camera messages.
/**
 * It calls
 * phasespace_msgs__msg__Camera__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
void
phasespace_msgs__msg__Camera__Sequence__fini(phasespace_msgs__msg__Camera__Sequence * array);

/// Create array of msg/Camera messages.
/**
 * It allocates the memory for the array and calls
 * phasespace_msgs__msg__Camera__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
phasespace_msgs__msg__Camera__Sequence *
phasespace_msgs__msg__Camera__Sequence__create(size_t size);

/// Destroy array of msg/Camera messages.
/**
 * It calls
 * phasespace_msgs__msg__Camera__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
void
phasespace_msgs__msg__Camera__Sequence__destroy(phasespace_msgs__msg__Camera__Sequence * array);

/// Check for msg/Camera message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
bool
phasespace_msgs__msg__Camera__Sequence__are_equal(const phasespace_msgs__msg__Camera__Sequence * lhs, const phasespace_msgs__msg__Camera__Sequence * rhs);

/// Copy an array of msg/Camera messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_phasespace_msgs
bool
phasespace_msgs__msg__Camera__Sequence__copy(
  const phasespace_msgs__msg__Camera__Sequence * input,
  phasespace_msgs__msg__Camera__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PHASESPACE_MSGS__MSG__DETAIL__CAMERA__FUNCTIONS_H_
