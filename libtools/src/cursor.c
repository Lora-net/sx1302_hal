#include <assert.h>
#include <cursor/cursor.h>
#include <cursor/packing.h>
#include <string.h>

struct cursor
cursor_new(void * buf, size_t buflen) {
    assert(buf);
    return (struct cursor){
        .buf = buf,
        .len = buflen,
        .pos = 0,
    };
}

/* Check that buffer has enough bytes left to unpack the desired type out */
#define SIZE_CHECK(_CSR, _PRIM_TYPE)                                           \
    do {                                                                       \
        if ((_CSR->pos + sizeof(_PRIM_TYPE)) > _CSR->len) {                    \
            return cursor_res_err_buf_exhausted;                               \
        }                                                                      \
    } while (0)

#define INC_POS(_CSR, _PRIM_TYPE)                                              \
    do {                                                                       \
        _CSR->pos += sizeof(_PRIM_TYPE);                                       \
    } while (0)

#define GENERATE(_PRIM_TYPE, _SHORT_NAME)                                      \
    enum cursor_res cursor_unpack_le_##_SHORT_NAME(struct cursor * csr,        \
                                                   _PRIM_TYPE *    dst) {         \
        SIZE_CHECK(csr, *dst);                                                 \
        unpack_le_##_SHORT_NAME(dst, &csr->buf[csr->pos]);                     \
        INC_POS(csr, *dst);                                                    \
        return cursor_res_ok;                                                  \
    }                                                                          \
    enum cursor_res cursor_pack_le_##_SHORT_NAME(struct cursor * csr,          \
                                                 _PRIM_TYPE      val) {             \
        SIZE_CHECK(csr, val);                                                  \
        pack_le_##_SHORT_NAME(&csr->buf[csr->pos], val);                       \
        INC_POS(csr, val);                                                     \
        return cursor_res_ok;                                                  \
    }                                                                          \
    enum cursor_res cursor_unpack_be_##_SHORT_NAME(struct cursor * csr,        \
                                                   _PRIM_TYPE *    dst) {         \
        SIZE_CHECK(csr, *dst);                                                 \
        unpack_be_##_SHORT_NAME(dst, &csr->buf[csr->pos]);                     \
        INC_POS(csr, *dst);                                                    \
        return cursor_res_ok;                                                  \
    }                                                                          \
    enum cursor_res cursor_pack_be_##_SHORT_NAME(struct cursor * csr,          \
                                                 _PRIM_TYPE      val) {             \
        SIZE_CHECK(csr, val);                                                  \
        pack_be_##_SHORT_NAME(&csr->buf[csr->pos], val);                       \
        INC_POS(csr, val);                                                     \
        return cursor_res_ok;                                                  \
    }


GENERATE(uint8_t, u8)
GENERATE(int8_t, i8)
GENERATE(uint16_t, u16)
GENERATE(int16_t, i16)
GENERATE(uint32_t, u32)
GENERATE(int32_t, i32)
GENERATE(uint64_t, u64)
GENERATE(int64_t, i64)
GENERATE(float, f)
GENERATE(double, d)

enum cursor_res
cursor_take(struct cursor * csr, size_t n, uint8_t * dst) {
    if (csr->pos + n > csr->len) {
        return cursor_res_err_buf_exhausted;
    }
    memmove(dst, csr->buf + csr->pos, n);
    csr->pos += n;
    return cursor_res_ok;
}

size_t
cursor_remaining(struct cursor const * csr) {
    return csr->len - csr->pos;
}

size_t
cursor_take_remaining(struct cursor * csr, uint8_t * dst) {
    size_t          remaining = cursor_remaining(csr);
    enum cursor_res res       = cursor_take(csr, remaining, dst);
    assert(cursor_res_ok == res);
    (void)res;
    assert(0 == cursor_remaining(csr));
    return remaining;
}

enum cursor_res
cursor_put(struct cursor * csr, void const * src, size_t src_len) {
    if (cursor_remaining(csr) < src_len) {
        return cursor_res_err_buf_exhausted;
    }
    memcpy(&csr->buf[csr->pos], src, src_len);
    csr->pos += src_len;
    return cursor_res_ok;
}
