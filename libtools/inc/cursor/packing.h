#ifndef PACKING_H
#define PACKING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void
unpack_le_u8(uint8_t * dst, void const * src);

void
unpack_le_i8(int8_t * dst, void const * src);

void
unpack_le_u16(uint16_t * dst, void const * src);

void
unpack_le_i16(int16_t * dst, void const * src);

void
unpack_le_u32(uint32_t * dst, void const * src);

void
unpack_le_i32(int32_t * dst, void const * src);

void
unpack_le_u32(uint32_t * dst, void const * src);

void
unpack_le_i32(int32_t * dst, void const * src);

void
unpack_le_u64(uint64_t * dst, void const * src);

void
unpack_le_i64(int64_t * dst, void const * src);

void
unpack_le_f(float * dst, void const * src);

void
unpack_le_d(double * dst, void const * src);


#define unpack_le(_DST, _SRC)                                                  \
    _Generic((_DST),                                                           \
             uint8_t*:  unpack_le_u8,                                          \
             int8_t*:   unpack_le_i8,                                          \
             uint16_t*: unpack_le_u16,                                         \
             int16_t*:  unpack_le_i16,                                         \
             uint32_t*: unpack_le_u32,                                         \
             int32_t*:  unpack_le_i32,                                         \
             uint64_t*: unpack_le_u64,                                         \
             int64_t*:  unpack_le_i64,                                         \
             float*:    unpack_le_f,                                           \
             double*:   unpack_le_d                                            \
             )(_DST, _SRC)

void
unpack_be_u8(uint8_t * dst, void const * src);

void
unpack_be_i8(int8_t * dst, void const * src);

void
unpack_be_u16(uint16_t * dst, void const * src);

void
unpack_be_i16(int16_t * dst, void const * src);

void
unpack_be_u32(uint32_t * dst, void const * src);

void
unpack_be_i32(int32_t * dst, void const * src);

void
unpack_be_u32(uint32_t * dst, void const * src);

void
unpack_be_i32(int32_t * dst, void const * src);

void
unpack_be_u64(uint64_t * dst, void const * src);

void
unpack_be_i64(int64_t * dst, void const * src);

void
unpack_be_f(float * dst, void const * src);

void
unpack_be_d(double * dst, void const * src);

#define unpack_be(_DST, _SRC)                                                  \
    _Generic((_DST),                                                           \
             uint8_t*:  unpack_be_u8,                                          \
             int8_t*:   unpack_be_i8,                                          \
             uint16_t*: unpack_be_u16,                                         \
             int16_t*:  unpack_be_i16,                                         \
             uint32_t*: unpack_be_u32,                                         \
             int32_t*:  unpack_be_i32,                                         \
             uint64_t*: unpack_be_u64,                                         \
             int64_t*:  unpack_be_i64,                                         \
             float*:    unpack_be_f,                                           \
             double*:   unpack_be_d                                            \
             )(_DST, _SRC)

void
pack_le_u8(void * dst, uint8_t val);

void
pack_le_i8(void * dst, int8_t val);

void
pack_le_u16(void * dst, uint16_t val);

void
pack_le_i16(void * dst, int16_t val);

void
pack_le_u32(void * dst, uint32_t val);

void
pack_le_i32(void * dst, int32_t val);

void
pack_le_u64(void * dst, uint64_t val);

void
pack_le_i64(void * dst, int64_t val);

void
pack_le_f(void * dst, float val);

void
pack_le_d(void * dst, double val);

#define pack_le(_DST, _VAL)                                                    \
    _Generic((_VAL), uint8_t                                                   \
             : pack_le_u8, int8_t                                              \
             : pack_le_i8, uint16_t                                            \
             : pack_le_u16, int16_t                                            \
             : pack_le_i16, uint32_t                                           \
             : pack_le_u32, int32_t                                            \
             : pack_le_i32, uint64_t                                           \
             : pack_le_u64, int64_t                                            \
             : pack_le_i64, float                                              \
             : pack_le_f, double                                               \
             : pack_le_d)(_DST, _VAL)

void
pack_be_u8(void * dst, uint8_t val);

void
pack_be_i8(void * dst, int8_t val);

void
pack_be_u16(void * dst, uint16_t val);

void
pack_be_i16(void * dst, int16_t val);

void
pack_be_u32(void * dst, uint32_t val);

void
pack_be_i32(void * dst, int32_t val);

void
pack_be_u64(void * dst, uint64_t val);

void
pack_be_i64(void * dst, int64_t val);

void
pack_be_f(void * dst, float val);

void
pack_be_d(void * dst, double val);

#define pack_be(_DST, _VAL)                                                    \
    _Generic((_VAL), uint8_t                                                   \
             : pack_be_u8, int8_t                                              \
             : pack_be_i8, uint16_t                                            \
             : pack_be_u16, int16_t                                            \
             : pack_be_i16, uint32_t                                           \
             : pack_be_u32, int32_t                                            \
             : pack_be_i32, uint64_t                                           \
             : pack_be_u64, int64_t                                            \
             : pack_be_i64, float                                              \
             : pack_be_f, double                                               \
             : pack_be_d)(_DST, _VAL)

#ifdef __cplusplus
}
#endif

#endif /* PACKING_H */
