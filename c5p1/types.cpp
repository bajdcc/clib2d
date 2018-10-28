//
// Project: cliblisp
// Author: bajdcc
//

#include <cassert>
#include "types.h"

namespace clib {
    string_t lexer_string_list[] = {
        "none",
        "error",
        "char",
        "uchar",
        "short",
        "ushort",
        "int",
        "uint",
        "long",
        "ulong",
        "float",
        "double",
        "operator",
        "string",
        "space",
        "newline",
        "END"
    };

    const string_t &lexer_typestr(lexer_t type) {
        assert(type >= l_none && type < l_end);
        return lexer_string_list[type];
    }

    std::tuple<operator_t, string_t, string_t> operator_string_list[] = {
        std::make_tuple(op__start, "@START", "@START"),
        std::make_tuple(op_lbrace, "{", "lbrace"),
        std::make_tuple(op_rbrace, "}", "rbrace"),
        std::make_tuple(op_lsquare, "[", "lsquare"),
        std::make_tuple(op_rsquare, "]", "rsquare"),
        std::make_tuple(op_lparan, "(", "lparan"),
        std::make_tuple(op_rparan, ")", "rparan"),
        std::make_tuple(op_quote, "`", "quote"),
        std::make_tuple(op_comma, ",", "comma"),
        std::make_tuple(op_colon, ":", "colon"),
        std::make_tuple(op_lambda, "\\", "lambda"),
        std::make_tuple(op__end, "??? unknown op", "unknown op"),
    };

    const string_t &lexer_opstr(operator_t type) {
        assert(type > op__start && type <= op__end);
        return std::get<1>(operator_string_list[type]);
    }

    const string_t &lexer_opnamestr(operator_t type) {
        assert(type > op__start && type <= op__end);
        return std::get<2>(operator_string_list[type]);
    }

    string_t err_string_list[] = {
        "@START",
        "#E !char!",
        "#E !operator!",
        "#E !digit!",
        "#E !string!",
        "@END",
    };

    const string_t &lexer_errstr(error_t type) {
        assert(type > e__start && type < e__end);
        return err_string_list[type];
    }
}
