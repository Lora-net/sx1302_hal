#include <cursor/packing.h>

#define SRC ((uint8_t *)src)
#define DST ((uint8_t *)dst)

#ifdef HAVE_ATTRIBUTE_MAY_ALIAS
typedef uint32_t __attribute__((__may_alias__)) uint32_a;
typedef uint64_t __attribute__((__may_alias__)) uint64_a;
#else
typedef uint32_t uint32_a;
typedef uint64_t uint64_a;
#endif

#define GENERATE_UNPACK_LE_8(_PRIM_TYPE, _SHORT_NAME)                          \
    void unpack_le_##_SHORT_NAME(_PRIM_TYPE * dst, void const * src) {         \
        *dst = SRC[0];                                                         \
    }

#define GENERATE_UNPACK_LE_16(_PRIM_TYPE, _SHORT_NAME)                         \
    void unpack_le_##_SHORT_NAME(_PRIM_TYPE * dst, void const * src) {         \
        *dst = (SRC[1] << 8) | SRC[0];                                         \
    }

#define GENERATE_UNPACK_LE_32(_PRIM_TYPE, _SHORT_NAME)                         \
    void unpack_le_##_SHORT_NAME(_PRIM_TYPE * dst, void const * src) {         \
        uint32_t val = ((uint32_t)SRC[3] << 24) | ((uint32_t)SRC[2] << 16)     \
                       | ((uint32_t)SRC[1] << 8) | SRC[0];                     \
        *((uint32_t *)dst) = val;                                              \
    }

#define GENERATE_UNPACK_LE_64(_PRIM_TYPE, _SHORT_NAME)                         \
    void unpack_le_##_SHORT_NAME(_PRIM_TYPE * dst, void const * src) {         \
        uint64_t val = ((uint64_t)SRC[7] << 56) | ((uint64_t)SRC[6] << 48)     \
                       | ((uint64_t)SRC[5] << 40) | ((uint64_t)SRC[4] << 32)   \
                       | ((uint64_t)SRC[3] << 24) | ((uint64_t)SRC[2] << 16)   \
                       | ((uint64_t)SRC[1] << 8) | (uint64_t)SRC[0];           \
        *((uint64_t *)dst) = val;                                              \
    }

#define GENERATE_UNPACK_BE_8(_PRIM_TYPE, _SHORT_NAME)                          \
    void unpack_be_##_SHORT_NAME(_PRIM_TYPE * dst, void const * src) {         \
        *dst = SRC[0];                                                         \
    }

#define GENERATE_UNPACK_BE_16(_PRIM_TYPE, _SHORT_NAME)                         \
    void unpack_be_##_SHORT_NAME(_PRIM_TYPE * dst, void const * src) {         \
        *dst = (SRC[0] << 8) | SRC[1];                                         \
    }

#define GENERATE_UNPACK_BE_32(_PRIM_TYPE, _SHORT_NAME)                         \
    void unpack_be_##_SHORT_NAME(_PRIM_TYPE * dst, void const * src) {         \
        *((uint32_t *)dst) = ((uint32_t)SRC[0] << 24)                          \
                             | ((uint32_t)SRC[1] << 16)                        \
                             | ((uint32_t)SRC[2] << 8) | SRC[3];               \
    }

#define GENERATE_UNPACK_BE_64(_PRIM_TYPE, _SHORT_NAME)                         \
    void unpack_be_##_SHORT_NAME(_PRIM_TYPE * dst, void const * src) {         \
        *((uint64_t *)dst) =                                                   \
            ((uint64_t)SRC[0] << 56) | ((uint64_t)SRC[1] << 48)                \
            | ((uint64_t)SRC[2] << 40) | ((uint64_t)SRC[3] << 32)              \
            | ((uint64_t)SRC[4] << 24) | ((uint64_t)SRC[5] << 16)              \
            | ((uint64_t)SRC[6] << 8) | (uint64_t)SRC[7];                      \
    }

#define GENERATE_PACK_LE_8(_PRIM_TYPE, _SHORT_NAME)                            \
    void pack_le_##_SHORT_NAME(void * dst, _PRIM_TYPE val) {                   \
        DST[0] = val;                                                          \
    }
#define GENERATE_PACK_LE_16(_PRIM_TYPE, _SHORT_NAME)                           \
    void pack_le_##_SHORT_NAME(void * dst, _PRIM_TYPE val) {                   \
        DST[0] = val & 0xff;                                                   \
        DST[1] = (val >> 8) & 0xff;                                            \
    }

#define GENERATE_PACK_LE_32(_PRIM_TYPE, _SHORT_NAME)                           \
    void pack_le_##_SHORT_NAME(void * dst, _PRIM_TYPE val) {                   \
        uint32_t val_cast = *((uint32_a *)&val);                               \
        DST[0]            = val_cast & 0xff;                                   \
        DST[1]            = (val_cast >> 8) & 0xff;                            \
        DST[2]            = (val_cast >> 16) & 0xff;                           \
        DST[3]            = (val_cast >> 24) & 0xff;                           \
    }

#define GENERATE_PACK_LE_64(_PRIM_TYPE, _SHORT_NAME)                           \
    void pack_le_##_SHORT_NAME(void * dst, _PRIM_TYPE val) {                   \
        uint64_t val_cast = *((uint64_a *)&val);                               \
        DST[0]            = val_cast & 0xff;                                   \
        DST[1]            = (val_cast >> 8) & 0xff;                            \
        DST[2]            = (val_cast >> 16) & 0xff;                           \
        DST[3]            = (val_cast >> 24) & 0xff;                           \
        DST[4]            = (val_cast >> 32) & 0xff;                           \
        DST[5]            = (val_cast >> 40) & 0xff;                           \
        DST[6]            = (val_cast >> 48) & 0xff;                           \
        DST[7]            = (val_cast >> 56) & 0xff;                           \
    }

#define GENERATE_PACK_BE_8(_PRIM_TYPE, _SHORT_NAME)                            \
    void pack_be_##_SHORT_NAME(void * dst, _PRIM_TYPE val) {                   \
        DST[0] = val;                                                          \
    }
#define GENERATE_PACK_BE_16(_PRIM_TYPE, _SHORT_NAME)                           \
    void pack_be_##_SHORT_NAME(void * dst, _PRIM_TYPE val) {                   \
        DST[0] = (val >> 8) & 0xff;                                            \
        DST[1] = val & 0xff;                                                   \
    }

#define GENERATE_PACK_BE_32(_PRIM_TYPE, _SHORT_NAME)                           \
    void pack_be_##_SHORT_NAME(void * dst, _PRIM_TYPE val) {                   \
        uint32_t val_cast = *((uint32_a *)&val);                               \
        DST[0]            = (val_cast >> 24) & 0xff;                           \
        DST[1]            = (val_cast >> 16) & 0xff;                           \
        DST[2]            = (val_cast >> 8) & 0xff;                            \
        DST[3]            = val_cast & 0xff;                                   \
    }

#define GENERATE_PACK_BE_64(_PRIM_TYPE, _SHORT_NAME)                           \
    void pack_be_##_SHORT_NAME(void * dst, _PRIM_TYPE val) {                   \
        uint64_t val_cast = *((uint64_a *)&val);                               \
        DST[0]            = (val_cast >> 56) & 0xff;                           \
        DST[1]            = (val_cast >> 48) & 0xff;                           \
        DST[2]            = (val_cast >> 40) & 0xff;                           \
        DST[3]            = (val_cast >> 32) & 0xff;                           \
        DST[4]            = (val_cast >> 24) & 0xff;                           \
        DST[5]            = (val_cast >> 16) & 0xff;                           \
        DST[6]            = (val_cast >> 8) & 0xff;                            \
        DST[7]            = val_cast & 0xff;                                   \
    }

GENERATE_UNPACK_LE_8(uint8_t, u8)
GENERATE_UNPACK_LE_8(int8_t, i8)
GENERATE_UNPACK_LE_16(uint16_t, u16)
GENERATE_UNPACK_LE_16(int16_t, i16)
GENERATE_UNPACK_LE_32(uint32_t, u32)
GENERATE_UNPACK_LE_32(int32_t, i32)
GENERATE_UNPACK_LE_64(uint64_t, u64)
GENERATE_UNPACK_LE_64(int64_t, i64)
GENERATE_UNPACK_LE_32(float, f)
GENERATE_UNPACK_LE_64(double, d)

GENERATE_PACK_LE_8(uint8_t, u8)
GENERATE_PACK_LE_8(int8_t, i8)
GENERATE_PACK_LE_16(uint16_t, u16)
GENERATE_PACK_LE_16(int16_t, i16)
GENERATE_PACK_LE_32(uint32_t, u32)
GENERATE_PACK_LE_32(int32_t, i32)
GENERATE_PACK_LE_64(uint64_t, u64)
GENERATE_PACK_LE_64(int64_t, i64)
GENERATE_PACK_LE_32(float, f)
GENERATE_PACK_LE_64(double, d)

GENERATE_UNPACK_BE_8(uint8_t, u8)
GENERATE_UNPACK_BE_8(int8_t, i8)
GENERATE_UNPACK_BE_16(uint16_t, u16)
GENERATE_UNPACK_BE_16(int16_t, i16)
GENERATE_UNPACK_BE_32(uint32_t, u32)
GENERATE_UNPACK_BE_32(int32_t, i32)
GENERATE_UNPACK_BE_64(uint64_t, u64)
GENERATE_UNPACK_BE_64(int64_t, i64)
GENERATE_UNPACK_BE_32(float, f)
GENERATE_UNPACK_BE_64(double, d)

GENERATE_PACK_BE_8(uint8_t, u8)
GENERATE_PACK_BE_8(int8_t, i8)
GENERATE_PACK_BE_16(uint16_t, u16)
GENERATE_PACK_BE_16(int16_t, i16)
GENERATE_PACK_BE_32(uint32_t, u32)
GENERATE_PACK_BE_32(int32_t, i32)
GENERATE_PACK_BE_64(uint64_t, u64)
GENERATE_PACK_BE_64(int64_t, i64)
GENERATE_PACK_BE_32(float, f)
GENERATE_PACK_BE_64(double, d)
