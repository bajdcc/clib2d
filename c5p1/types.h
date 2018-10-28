//
// Project: cliblisp
// Author: bajdcc
//

#ifndef CLIBLISP_TYPES_H
#define CLIBLISP_TYPES_H

#include <string>
#include <unordered_map>

using string_t = std::string;
template<class K, class V> using map_t = std::unordered_map<K, V>;

namespace clib {
#if __APPLE__ && __MACH__
    using int8 = int8_t;
    using uint8 = uint8_t;
    using int16 = int16_t;
    using uint16 = uint16_t;
    using int32 = int32_t;
    using uint32 = uint32_t;
    using int64 = int64_t;
    using uint64 = uint64_t;
#else
    using int8 = signed __int8;
    using uint8 = unsigned __int8;
    using int16 = signed __int16;
    using uint16 = unsigned __int16;
    using int32 = signed __int32;
    using uint32 = unsigned __int32;
    using int64 = signed __int64;
    using uint64 = unsigned __int64;
#endif

    // 这里暂不支持64位程序
#if __x86_64__
#warning "Not support x86_64"
#endif
#if 1
    using sint = int32;
    using uint = uint32;
    using slong = long long;
    using ulong = unsigned long long;
#else
    using sint = int64;
    using uint = uint64;
    using slong = long;
    using ulong = unsigned long;
#endif
    using byte = uint8;
    using size_t = uint;

    enum lexer_t {
        l_none,
        l_error,
        l_char,
        l_uchar,
        l_short,
        l_ushort,
        l_int,
        l_uint,
        l_long,
        l_ulong,
        l_float,
        l_double,
        l_operator,
        l_identifier,
        l_string,
        l_space,
        l_newline,
        l_end,
    };

    enum operator_t {
        op__start,
        op_lbrace,
        op_rbrace,
        op_lsquare,
        op_rsquare,
        op_lparan,
        op_rparan,
        op_quote,
        op_comma,
        op_colon,
        op_lambda,
        op__end,
    };

    enum error_t {
        e__start,
        e_invalid_char,
        e_invalid_operator,
        e_invalid_digit,
        e_invalid_string,
        e__end
    };

    template<lexer_t>
    struct base_t {
        using type = void *;
    };
    template<class T>
    struct base_lexer_t {
        static const lexer_t type = l_none;
    };

#define DEFINE_BASE_TYPE(t, obj) \
template<> \
struct base_t<t> \
{ \
    using type = obj; \
    static const int size = sizeof(obj); \
};

    DEFINE_BASE_TYPE(l_char, char)
    DEFINE_BASE_TYPE(l_uchar, unsigned char)
    DEFINE_BASE_TYPE(l_short, short)
    DEFINE_BASE_TYPE(l_ushort, unsigned short)
    DEFINE_BASE_TYPE(l_int, int)
    DEFINE_BASE_TYPE(l_uint, unsigned int)
    DEFINE_BASE_TYPE(l_long, slong)
    DEFINE_BASE_TYPE(l_ulong, ulong)
    DEFINE_BASE_TYPE(l_float, float)
    DEFINE_BASE_TYPE(l_double, double)
    DEFINE_BASE_TYPE(l_operator, operator_t)
    DEFINE_BASE_TYPE(l_identifier, string_t)
    DEFINE_BASE_TYPE(l_string, string_t)
    DEFINE_BASE_TYPE(l_space, uint)
    DEFINE_BASE_TYPE(l_newline, uint)
    DEFINE_BASE_TYPE(l_error, error_t)
#undef DEFINE_BASE_TYPE

#define DEFINE_CONV_TYPE(t, obj) \
template<> \
struct base_lexer_t<obj> \
{ \
    static const lexer_t type = t; \
};

    DEFINE_CONV_TYPE(l_char, char)
    DEFINE_CONV_TYPE(l_uchar, unsigned char)
    DEFINE_CONV_TYPE(l_short, short)
    DEFINE_CONV_TYPE(l_ushort, unsigned short)
    DEFINE_CONV_TYPE(l_int, int)
    DEFINE_CONV_TYPE(l_uint, unsigned int)
    DEFINE_CONV_TYPE(l_long, slong)
    DEFINE_CONV_TYPE(l_ulong, ulong)
    DEFINE_CONV_TYPE(l_float, float)
    DEFINE_CONV_TYPE(l_double, double)
    DEFINE_CONV_TYPE(l_string, string_t)
    DEFINE_CONV_TYPE(l_error, error_t)
#undef DEFINE_CONV_TYPE

    const string_t &lexer_typestr(lexer_t);
    const string_t &lexer_opstr(operator_t);
    const string_t &lexer_opnamestr(operator_t);
    const string_t &lexer_errstr(error_t);

    extern string_t keyword_string_list[];

#define LEX_T(t) base_t<l_##t>::type
#define LEX_CONV_T(t) base_lexer_t<t>::type
#define LEX_SIZEOF(t) base_t<l_##t>::size
#define LEX_STRING(t) lexer_typestr(t)

#define OPERATOR_STRING(t) lexer_opnamestr(t)
#define OP_STRING(t) lexer_opstr(t)
#define ERROR_STRING(t) lexer_errstr(t)
}

#endif //CLIBLISP_TYPES_H
