#ifndef CURSOR_H
#define CURSOR_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A context for [de]serializing data from a buffer */
struct cursor {
    uint8_t * buf;
    size_t    len;
    size_t    pos;
};

enum cursor_res {
    /* The unpack/pack operation was successful */
    cursor_res_ok,
    /* No more bytes left in the buffer */
    cursor_res_err_buf_exhausted,
};

/**
 * Creates and returns a new cursor object
 */
struct cursor
cursor_new(void * buf, size_t buflen);

size_t
cursor_remaining(struct cursor const * csr);

enum cursor_res
cursor_take(struct cursor * csr, size_t n, uint8_t * dst);

size_t
cursor_take_remaining(struct cursor * csr, uint8_t * dst);

enum cursor_res
cursor_put(struct cursor * csr, void const * src, size_t src_len);

enum cursor_res
cursor_unpack_le_u8(struct cursor * csr, uint8_t * dst);

enum cursor_res
cursor_unpack_le_i8(struct cursor * csr, int8_t * dst);

enum cursor_res
cursor_unpack_le_u16(struct cursor * csr, uint16_t * dst);

enum cursor_res
cursor_unpack_le_i16(struct cursor * csr, int16_t * dst);

enum cursor_res
cursor_unpack_le_u32(struct cursor * csr, uint32_t * dst);

enum cursor_res
cursor_unpack_le_i32(struct cursor * csr, int32_t * dst);

enum cursor_res
cursor_unpack_le_u64(struct cursor * csr, uint64_t * dst);

enum cursor_res
cursor_unpack_le_i64(struct cursor * csr, int64_t * dst);

enum cursor_res
cursor_unpack_le_f(struct cursor * csr, float * dst);

enum cursor_res
cursor_unpack_le_d(struct cursor * csr, double * dst);

#define cursor_unpack_le(_CSR, _DST)                                           \
    _Generic((_DST),                                                           \
             uint8_t*:  cursor_unpack_le_u8,                                     \
             uint16_t*: cursor_unpack_le_u16,                                    \
             uint32_t*: cursor_unpack_le_u32,                                    \
             uint64_t*: cursor_unpack_le_u64,                                    \
             int8_t*:   cursor_unpack_le_i8,                                     \
             int16_t*:  cursor_unpack_le_i16,                                    \
             int32_t*:  cursor_unpack_le_i32,                                    \
             int64_t*:  cursor_unpack_le_i64,                                    \
             float*:    cursor_unpack_le_f,                                      \
             double*:   cursor_unpack_le_d                                       \
             )(_CSR, _DST)

enum cursor_res
cursor_unpack_be_u8(struct cursor * csr, uint8_t * dst);

enum cursor_res
cursor_unpack_be_i8(struct cursor * csr, int8_t * dst);

enum cursor_res
cursor_unpack_be_u16(struct cursor * csr, uint16_t * dst);

enum cursor_res
cursor_unpack_be_i16(struct cursor * csr, int16_t * dst);

enum cursor_res
cursor_unpack_be_u32(struct cursor * csr, uint32_t * dst);

enum cursor_res
cursor_unpack_be_i32(struct cursor * csr, int32_t * dst);

enum cursor_res
cursor_unpack_be_u64(struct cursor * csr, uint64_t * dst);

enum cursor_res
cursor_unpack_be_i64(struct cursor * csr, int64_t * dst);

enum cursor_res
cursor_unpack_be_f(struct cursor * csr, float * dst);

enum cursor_res
cursor_unpack_be_d(struct cursor * csr, double * dst);

#define cursor_unpack_be(_CSR, _DST)                                           \
    _Generic((_DST),                                                           \
             uint8_t*:  cursor_unpack_be_u8,                                   \
             uint16_t*: cursor_unpack_be_u16,                                  \
             uint32_t*: cursor_unpack_be_u32,                                  \
             uint64_t*: cursor_unpack_be_u64,                                  \
             int8_t*:   cursor_unpack_be_i8,                                   \
             int16_t*:  cursor_unpack_be_i16,                                  \
             int32_t*:  cursor_unpack_be_i32,                                  \
             int64_t*:  cursor_unpack_be_i64,                                  \
             float*:    cursor_unpack_be_f,                                    \
             double*:   cursor_unpack_be_d                                     \
             )(_CSR, _DST)

enum cursor_res
cursor_pack_le_u8(struct cursor * csr, uint8_t val);

enum cursor_res
cursor_pack_le_i8(struct cursor * csr, int8_t val);

enum cursor_res
cursor_pack_le_u16(struct cursor * csr, uint16_t val);

enum cursor_res
cursor_pack_le_i16(struct cursor * csr, int16_t val);

enum cursor_res
cursor_pack_le_u32(struct cursor * csr, uint32_t val);

enum cursor_res
cursor_pack_le_i32(struct cursor * csr, int32_t val);

enum cursor_res
cursor_pack_le_u64(struct cursor * csr, uint64_t val);

enum cursor_res
cursor_pack_le_i64(struct cursor * csr, int64_t val);

enum cursor_res
cursor_pack_le_f(struct cursor * csr, float val);

enum cursor_res
cursor_pack_le_d(struct cursor * csr, double val);

#define cursor_pack_le(_CSR, _DST)                                             \
    _Generic((_DST), uint8_t                                                   \
             : cursor_pack_le_u8, uint16_t                                     \
             : cursor_pack_le_u16, uint32_t                                    \
             : cursor_pack_le_u32, uint64_t                                    \
             : cursor_pack_le_u64, int8_t                                      \
             : cursor_pack_le_i8, int16_t                                      \
             : cursor_pack_le_i16, int32_t                                     \
             : cursor_pack_le_i32, int64_t                                     \
             : cursor_pack_le_i64, float                                       \
             : cursor_pack_le_f, double                                        \
             : cursor_pack_le_d)(_CSR, _DST)

enum cursor_res
cursor_pack_be_u8(struct cursor * csr, uint8_t val);

enum cursor_res
cursor_pack_be_i8(struct cursor * csr, int8_t val);

enum cursor_res
cursor_pack_be_u16(struct cursor * csr, uint16_t val);

enum cursor_res
cursor_pack_be_i16(struct cursor * csr, int16_t val);

enum cursor_res
cursor_pack_be_u32(struct cursor * csr, uint32_t val);

enum cursor_res
cursor_pack_be_i32(struct cursor * csr, int32_t val);

enum cursor_res
cursor_pack_be_u64(struct cursor * csr, uint64_t val);

enum cursor_res
cursor_pack_be_i64(struct cursor * csr, int64_t val);

enum cursor_res
cursor_pack_be_f(struct cursor * csr, float val);

enum cursor_res
cursor_pack_be_d(struct cursor * csr, double val);

#define cursor_pack_be(_CSR, _DST)                                             \
    _Generic((_DST), uint8_t                                                   \
             : cursor_pack_be_u8, uint16_t                                     \
             : cursor_pack_be_u16, uint32_t                                    \
             : cursor_pack_be_u32, uint64_t                                    \
             : cursor_pack_be_u64, int8_t                                      \
             : cursor_pack_be_i8, int16_t                                      \
             : cursor_pack_be_i16, int32_t                                     \
             : cursor_pack_be_i32, int64_t                                     \
             : cursor_pack_be_i64, float                                       \
             : cursor_pack_be_f, double                                        \
             : cursor_pack_be_d)(_CSR, _DST)

#ifdef __cplusplus
}
#endif

#endif /* CURSOR_H */
