#ifndef MESH_UTILS_H__
#define MESH_UTILS_H__

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "flash_config.h"
#include "PN102Series.h"

#define MESH_ASSERT(cond)               \
	do{ 										   \
		 if (!(cond))							   \
		 {                                         \
             printf("assert[%s:%d]\n",__func__,__LINE__); \
		 }										   \
	 } while (0)	

#define MESH_ERR_CHECK(err) MESH_ASSERT(err == PAN_SUCCESS)

#define PAN_MESH_STATIC_ASSERT(cond) typedef char static_assert[(cond) ? 1 : -1]
    
#define __MESH_LOG(...)  printf

#define __INTERNAL_EVENT_PUSH(...)

typedef uint32_t timestamp_t;

typedef void (*bearer_start_cb_t)(timestamp_t start_time, void* p_args);

typedef struct
{
    bearer_start_cb_t          start_cb;          /**< Start of action-callback for the action. */
    void*                      p_args;            /**< Arguments pointer provided to the callbacks. */
} bearer_action_t;

#define CONCAT_2(p1, p2)      CONCAT_2_(p1, p2)
/** Auxiliary macro used by @ref CONCAT_2 */
#define CONCAT_2_(p1, p2)     p1##p2

/**@brief Concatenates three parameters.
 *
 * It realizes two level expansion to make it sure that all the parameters
 * are actually expanded before gluing them together.
 *
 * @param p1 First parameter to concatenating
 * @param p2 Second parameter to concatenating
 * @param p3 Third parameter to concatenating
 *
 * @return Three parameters glued together.
 *         They have to create correct C mnemonic in other case
 *         preprocessor error would be generated.
 *
 * @sa CONCAT_2
 */
#define CONCAT_3(p1, p2, p3)  CONCAT_3_(p1, p2, p3)
/** Auxiliary macro used by @ref CONCAT_3 */
#define CONCAT_3_(p1, p2, p3) p1##p2##p3

#define STRINGIFY_(val) #val
/** Converts a macro argument into a character constant.
 */
#define STRINGIFY(val)  STRINGIFY_(val)

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define CEILING(dividend,divisor) ((dividend)/(divisor) + (((dividend)%(divisor))?1:0))
/**
 * @defgroup UTILS Utility functions
 * @ingroup MESH_CORE
 * This library provides implementations of common utility functions.
 *
 * @{
 */

#   define BE2LE16(n) __REV16(n) /**< Converts a 16-bit big-endian number to little-endian. */
#   define LE2BE16(n) __REV16(n) /**< Converts a 16-bit little-endian number to big endian. */
#   define BE2LE32(n) __REV(n)   /**< Converts a 32-bit big-endian number to little-endian. */
#   define LE2BE32(n) __REV(n)   /**< Converts a 32-bit little-endian number to big-endian. */


/** Converts a 24-bit little-endian number to big-endian. */
#define LE2BE24(n) ((((n) & 0x00FF0000) >> 16) |        \
                    ( (n) & 0x0000FF00)        |        \
                    (((n) & 0x000000FF) << 16))

/** Converts a 24-bit big-endian number to little-endian. */
#define BE2LE24(n) LE2BE24(n)

/** Size of a word on the current hardware platform. */
#define WORD_SIZE (sizeof(int))




//lint -emacro(572,MSB_32) // Suppress warning 572 "Excessive shift value"
#define MSB_32(a) (((a) & 0xFF000000) >> 24)
/** The lower 8 bits (of a 32 bit value) */
#define LSB_32(a) ((a) & 0x000000FF)

/** The upper 8 bits of a 16 bit value */
//lint -emacro(572,MSB_16) // Suppress warning 572 "Excessive shift value"
#define MSB_16(a) (((a) & 0xFF00) >> 8)
/** The lower 8 bits (of a 16 bit value) */
#define LSB_16(a) ((a) & 0x00FF)

/** Leaves the minimum of the two 32-bit arguments */
/*lint -emacro(506, MIN) */ /* Suppress "Constant value Boolean */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
/** Leaves the maximum of the two 32-bit arguments */
/*lint -emacro(506, MAX) */ /* Suppress "Constant value Boolean */
#define MAX(a, b) ((a) < (b) ? (b) : (a))

/** Returns an equivalent for a number (X) aligned to given word size (A). Value of A must be 2^n. */
#define ALIGN_VAL(X, A)          (((X)+((A)-1))&~((A)-1))
/** Checks whether the given value is power of two. */
#define IS_POWER_OF_2(VALUE) ((VALUE) && (((VALUE) & ((VALUE) - 1)) == 0))

/**@brief Macro for performing rounded integer division (as opposed to truncating the result).
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Rounded (integer) result of dividing A by B.
 */
#ifndef ROUNDED_DIV
#define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))
#endif

/**
 * Converts hours to seconds.
 * @param t The number of hours.
 * @return  The number of seconds corresponding to the specified number of hours.
 */
#define HOURS_TO_SECONDS(t) ((t) * 60 * 60)

/**
 * Converts minutes to milliseconds.
 * @param t The number of minutes.
 * @return  The number of milliseconds corresponding to the specified number of minutes.
 */
#define MIN_TO_MS(t) ((t) * 60000ul)

/**
 * Converts seconds to microseconds.
 * @param t The number of seconds.
 * @return  The number of microseconds corresponding to the specified number of seconds.
 */
#define SEC_TO_US(t) ((t) * 1000000ul)

/**
 * Converts seconds to milliseconds.
 * @param t The number of seconds.
 * @return  The number of milliseconds corresponding to the specified number of seconds.
 */
#define SEC_TO_MS(t) ((t) * 1000)

/**
 * Converts milliseconds to microseconds.
 * @param t The number of milliseconds.
 * @return  The number of microseconds corresponding to the specified number of milliseconds.
 */
#define MS_TO_US(t) ((t) * 1000)

/**
 * Converts milliseconds to seconds.
 * @param t The number of milliseconds.
 * @return  The number of seconds corresponding to the specified number of milliseconds.
 */
#define MS_TO_SEC(t) (ROUNDED_DIV(t, 1000))

/**
 * Converts milliseconds to minutes.
 * @param t The number of milliseconds.
 * @return  The number of minutes corresponding to the specified number of milliseconds.
 */
#define MS_TO_MIN(t) ((t) / 60000ul)

/**
 * Converts microseconds to milliseconds.
 * @param t The number of microseconds.
 * @return  The number of milliseconds corresponding to the specified number of microseconds.
 */
#define US_TO_MS(t) (ROUNDED_DIV(t, 1000))

/**
 * Converts microseconds to seconds.
 * @param t The number of microseconds.
 * @return  The number of seconds corresponding to the specified number of microseconds.
 */
#define US_TO_SEC(t) (ROUNDED_DIV(t, 1000000ul))

/**
 * Macro for checking if min <= val <= max.
 *
 * @param[in] val Value to check.
 * @param[in] min Range minimum.
 * @param[in] max Range maximum.
 *
 * @retval true  If the value is within the range.
 * @retval false Otherwise.
 */
#define IS_IN_RANGE(val, min, max) (((min) <= (val) && (val) <= (max)))

/**
 * Get pointer to the start of a structure given a pointer to one of the structure's fields.
 *
 * @param[in] STRUCT_TYPE   Type of structure.
 * @param[in] FIELD_NAME    Name of field.
 * @param[in] FIELD_POINTER Pointer to field inside the structure.
 *
 * @return Pointer to start of structure.
 */
#define __offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#define PARENT_BY_FIELD_GET(STRUCT_TYPE, FIELD_NAME, FIELD_POINTER) \
    ((STRUCT_TYPE *) (((uint8_t *)FIELD_POINTER) - __offsetof(STRUCT_TYPE, FIELD_NAME)))

/**@brief Macro for performing integer division, making sure the result is rounded up.
 *
 * @details One typical use for this is to compute the number of objects with size B is needed to
 *          hold A number of bytes.
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Integer result of dividing A by B, rounded up.
 */
#ifndef CEIL_DIV
#define CEIL_DIV(A, B)   (((A) + (B) - 1) / (B))
#endif

/**
 * Check if a value is power of two without evaluating the value multiple times.
 *
 * See https://graphics.stanford.edu/~seander/bithacks.html#DetermineIfPowerOf2 for details.
 *
 * @param[in] value Value to check for.
 *
 * @returns Whether value is power of two.
 */
static inline bool is_power_of_two(uint32_t value)
{
    return IS_POWER_OF_2(value);
}

/**
 * Reverse memcpy.
 *
 * Writes size bytes from p_src to p_dst in reverse order.
 *
 * @param p_dst Destination address.
 * @param p_src Source address.
 * @param size  Number of bytes to write.
 */
static inline void utils_reverse_memcpy(uint8_t * p_dst, const uint8_t * p_src, uint16_t size)
{
    p_src += size;
    while (size--)
    {
        *((uint8_t *) p_dst++) = *((uint8_t *) --p_src);
    }
}

/**
 * Reverses an array in-place.
 *
 * @param p_array The array to reverse.
 * @param size    The length of the array to reverse.
 */
static inline void utils_reverse_array(uint8_t * p_array, uint16_t size)
{
    for(uint16_t i = 0; i < size / 2; ++i)
    {
        uint8_t temp = p_array[i];
        p_array[i] = p_array[size - i - 1];
        p_array[size - i - 1] = temp;
    }
}

/**
 * Bytewise XOR for an array.
 *
 * XORs size amount of data from p_src1 and p_src2 and stores it in p_dst.
 *
 * @note p_dst may be equal to one or more of the sources.
 *
 * @param p_dst  Destination address.
 * @param p_src1 First source address.
 * @param p_src2 Secound source address.
 * @param size   Number of bytes to XOR.
 */
static inline void utils_xor(uint8_t * p_dst, const uint8_t * p_src1, const uint8_t * p_src2, uint16_t size)
{
    while (0 != size)
    {
        size--;
        p_dst[size] = p_src1[size] ^ p_src2[size];
    }
}

/**
 * Left shift an array of bytes one bit. p_dst and p_src may be the same.
 *
 * @param p_dst Destination address.
 * @param p_src Source address.
 * @param size  Size of p_dst and p_src.
 */
static inline void utils_lshift(uint8_t * p_dst, const uint8_t * p_src, uint16_t size)
{
    for (uint16_t i = 0; i < size - 1; ++i)
    {
        p_dst[i] = (p_src[i] << 1);
        p_dst[i] |= !!(p_src[i + 1] & 0x80);
    }
    p_dst[size - 1] = (p_src[size - 1] << 1);
}

/**
 * Pads an array according to AES-CMAC RFC 4493.
 *
 * For an input string x of r-octets, where 0 <= r < 16, the padding
 * function, padding(x), is defined as follows:
 *
 * @verbatim padding(x) = x || 10^i      where i is 128-8*r-1 @endverbatim
 *
 * That is, padding(x) is the concatenation of x and a single '1',
 * followed by the minimum number of '0's, so that the total length is
 * equal to 128 bits.
 *
 * @param p_dst Destination address.
 * @param p_src Source address.
 * @param size  Size of p_dst and p_src arrays.
 */
static inline void utils_pad(uint8_t * p_dst, const uint8_t * p_src, uint16_t size)
{
    memcpy(p_dst, p_src, size);
    p_dst[size] = 0x80;

    for (int i = size+1; i < 16; ++i)
        p_dst[i] = 0x00;
}

/**
 * Gets the integer value of the binary logarithm of a number.
 *
 * @param[in] value Input value for the calculation.
 *
 * @returns Returns the binary logarithm of the input number.
 */
static inline uint8_t log2_get(uint32_t value)
{
    uint8_t log_val = 0;
    while ((value >>= 1) != 0)
    {
        log_val++;
    }

    return log_val;
}


extern  inline uint8_t calculat_bits(uint32_t data)
{
	uint8_t i = 0;

	while(data )
	{
		if(data & 0x00000001)
			i++;
		data>>=1;
	}
	return i;
}
/**
 * @}
 */
#endif

