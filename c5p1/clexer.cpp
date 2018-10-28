//
// Project: cliblisp
// Author: bajdcc
//

#include <cassert>
#include <climits>
#include <sstream>
#include <cstring>
#include "clexer.h"

namespace clib {

    clexer::clexer(string_t str) : str(str) {
        length = str.length();
        assert(length > 0);
        initMap();
    }

    clexer::~clexer() = default;

#define DEFINE_LEXER_GETTER(t) \
LEX_T(t) clexer::get_##t() const \
{ \
    return bags._##t; \
}

    DEFINE_LEXER_GETTER(char)
    DEFINE_LEXER_GETTER(uchar)
    DEFINE_LEXER_GETTER(short)
    DEFINE_LEXER_GETTER(ushort)
    DEFINE_LEXER_GETTER(int)
    DEFINE_LEXER_GETTER(uint)
    DEFINE_LEXER_GETTER(long)
    DEFINE_LEXER_GETTER(ulong)
    DEFINE_LEXER_GETTER(float)
    DEFINE_LEXER_GETTER(double)
    DEFINE_LEXER_GETTER(operator)
    DEFINE_LEXER_GETTER(identifier)
    DEFINE_LEXER_GETTER(string)
    DEFINE_LEXER_GETTER(space)
    DEFINE_LEXER_GETTER(newline)
    DEFINE_LEXER_GETTER(error)
#undef DEFINE_LEXER_GETTER

#define DEFINE_LEXER_GETTER(t) \
LEX_T(t) clexer::get_store_##t(int index) const \
{ \
    return storage._##t[index]; \
}

    DEFINE_LEXER_GETTER(char)
    DEFINE_LEXER_GETTER(uchar)
    DEFINE_LEXER_GETTER(short)
    DEFINE_LEXER_GETTER(ushort)
    DEFINE_LEXER_GETTER(int)
    DEFINE_LEXER_GETTER(uint)
    DEFINE_LEXER_GETTER(long)
    DEFINE_LEXER_GETTER(ulong)
    DEFINE_LEXER_GETTER(float)
    DEFINE_LEXER_GETTER(double)
    DEFINE_LEXER_GETTER(identifier)
    DEFINE_LEXER_GETTER(string)
#undef DEFINE_LEXER_GETTER

    // 记录错误
    lexer_t clexer::record_error(error_t error, uint skip) {
        err_record_t err{};
        err.line = line; // 起始行
        err.column = column; // 起始列
        err.start_idx = index; // 文本起始位置
        err.end_idx = index + skip; // 文本结束位置
        err.err = error; // 错误类型
        err.str = str.substr(err.start_idx, err.end_idx - err.start_idx); // 错误字符
        records.push_back(err);
        bags._error = error;
        move(skip); // 略过错误文本
        return l_error;
    }

    const clexer::err_record_t &clexer::recent_error() const {
        return records.back();
    }

    lexer_t clexer::next() {
        auto c = local();
        if (c == -1) {
            type = l_end;
            return l_end;
        }
        type = l_error;
        if (isalpha(c) || c == '_') { // 变量名或关键字
            type = next_alpha();
        } else if (c == '\"') { // 字符串
            type = next_string();
        } else if (isdigit(c) || (c == '-' && isdigit(local(1)))) { // 数字
            if (c == '-') {
                move(1);
                type = next_digit();
                if (type == l_error)
                    return type;
                switch (type) {
                    case l_char:
                        bags._char = -bags._char;
                        break;
                    case l_short:
                        bags._short = -bags._short;
                        break;
                    case l_int:
                        bags._int = -bags._int;
                        break;
                    case l_long:
                        bags._long = -bags._long;
                        break;
                    case l_float:
                        bags._float = -bags._float;
                        break;
                    case l_double:
                        bags._double = -bags._double;
                        break;
                    default:
                        break;
                }
                return type;
            } else {
                type = next_digit();
            }
        } else if (isspace(c)) { // 空白字符
            type = next_space();
        } else if (c == '\'') { // 字符
            type = next_char();
        } else { // 最后才检查操作符
            type = next_operator();
        }
        return type;
    }

    lexer_t clexer::get_type() const {
        return type;
    }

    int clexer::get_line() const {
        return line;
    }

    int clexer::get_column() const {
        return column;
    }

    int clexer::get_last_line() const {
        return last_line;
    }

    int clexer::get_last_column() const {
        return last_column;
    }

    string_t clexer::current() const {
        switch (type) {
            case l_operator:
                return str.substr(last_index, index - last_index) + "\t[" + OPERATOR_STRING(bags._operator) + "]";
            default:
                break;
        }
        return str.substr(last_index, index - last_index);
    }

    bool clexer::is_type(lexer_t type) const {
        return get_type() == type;
    }

    bool clexer::is_operator(operator_t type) const {
        return get_type() == l_operator && get_operator() == type;
    }

    bool clexer::is_operator(operator_t type1, operator_t type2) const {
        return get_type() == l_operator && (get_operator() == type1 || get_operator() == type2);
    }

    bool clexer::is_number() const {
        return get_type() >= l_char && get_type() <= l_double;
    }

    bool clexer::is_integer() const {
        return get_type() >= l_char && get_type() <= l_ulong;
    }

    LEX_T(int) clexer::get_integer() const {
        assert(is_integer());
        switch (type) {
#define DEFINE_LEXER_CASE(t) case l_##t: return get_##t();
            DEFINE_LEXER_CASE(char)
            DEFINE_LEXER_CASE(uchar)
            DEFINE_LEXER_CASE(short)
            DEFINE_LEXER_CASE(ushort)
            DEFINE_LEXER_CASE(int)
            DEFINE_LEXER_CASE(uint)
            DEFINE_LEXER_CASE(long)
            DEFINE_LEXER_CASE(ulong)
#undef DEFINE_LEXER_CASE
            default:
                break;
        }
        return 0;
    }



    void clexer::move(uint idx, int inc) {
        last_index = index;
        last_line = line;
        last_column = column;
        if (inc < 0) {
            column += idx;
        } else {
            column = 1;
            line += inc;
        }
        index += idx;
    }

    // 计算幂
    template<class T>
    static T calc_exp(T d, int e) {
        if (e == 0)
            return d;
        else if (e > 0)
            for (int i = 0; i < e; i++)
                d *= 10;
        else
            for (int i = e; i < 0; i++)
                d /= 10;
        return d;
    }

    // 转无符号类型
    static lexer_t unsigned_type(lexer_t t) {
        switch (t) {
            case l_char:
                return l_uchar;
            case l_short:
                return l_ushort;
            case l_int:
                return l_uint;
            case l_long:
                return l_ulong;
            default:
                return t;
        }
    }

    // 数字类型后缀
    static lexer_t digit_type_postfix(char c) {
        switch (c) {
            case 'C':
            case 'c':
                return l_char;
            case 'S':
            case 's':
                return l_short;
            case 'I':
            case 'i':
                return l_int;
            case 'L':
            case 'l':
                return l_long;
            case 'F':
            case 'f':
                return l_float;
            case 'D':
            case 'd':
                return l_double;
            default:
                return l_error;
        }
    }

    // 数字类型后缀（带无符号）
    lexer_t clexer::digit_type(lexer_t t, uint &i) {
        if (i == length) {
            return l_error;
        }
        if (str[i] == 'U' || str[i] == 'u') {
            if (++i == length) {
                return unsigned_type(t);
            }
            if ((t = unsigned_type(digit_type_postfix(str[i]))) == l_error) {
                return l_error;
            }
            ++i;
            return t;
        } else {
            if ((t = digit_type_postfix(str[i])) == l_error) {
                return l_error;
            }
            ++i;
            return t;
        }
    }

    bool clexer::digit_from_integer(lexer_t t, LEX_T(ulong) n) {
        switch (t) {
#define DEFINE_LEXER_CONV_INTEGER(t) case l_##t: bags._##t = (LEX_T(t)) n; break;
            DEFINE_LEXER_CONV_INTEGER(char)
            DEFINE_LEXER_CONV_INTEGER(uchar)
            DEFINE_LEXER_CONV_INTEGER(short)
            DEFINE_LEXER_CONV_INTEGER(ushort)
            DEFINE_LEXER_CONV_INTEGER(int)
            DEFINE_LEXER_CONV_INTEGER(uint)
            DEFINE_LEXER_CONV_INTEGER(long)
            DEFINE_LEXER_CONV_INTEGER(ulong)
            DEFINE_LEXER_CONV_INTEGER(float)
            DEFINE_LEXER_CONV_INTEGER(double)
#undef DEFINE_LEXER_CONV_INTEGER
            default:
                return false;
        }
        return true;
    }

    bool clexer::digit_from_double(lexer_t t, LEX_T(double) d) {
        switch (t) {
#define DEFINE_LEXER_CONV_INTEGER(t) case l_##t: bags._##t = (LEX_T(t)) d; break;
            DEFINE_LEXER_CONV_INTEGER(char)
            DEFINE_LEXER_CONV_INTEGER(uchar)
            DEFINE_LEXER_CONV_INTEGER(short)
            DEFINE_LEXER_CONV_INTEGER(ushort)
            DEFINE_LEXER_CONV_INTEGER(int)
            DEFINE_LEXER_CONV_INTEGER(uint)
            DEFINE_LEXER_CONV_INTEGER(long)
            DEFINE_LEXER_CONV_INTEGER(ulong)
            DEFINE_LEXER_CONV_INTEGER(float)
            DEFINE_LEXER_CONV_INTEGER(double)
#undef DEFINE_LEXER_CONV_INTEGER
            default:
                return false;
        }
        return true;
    }

    // 返回数字（依照目前识别的类型）
    lexer_t clexer::digit_return(lexer_t t, LEX_T(ulong) n, LEX_T(double) d, uint i) {
        if (t == l_int) {
            bags._int = (int) n;
        } else if (t == l_double) {
            bags._double = d;
        } else if (t == l_long) {
            bags._long = n;
        } else {
            bags._double = d;
        }
        move(i - index);
        return t;
    }

    // 十六进制字符转十进制
    static int hex2dec(char c) {
        if (c >= '0' && c <= '9') {
            return c - '0';
        } else if (c >= 'a' && c <= 'f') {
            return c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            return c - 'A' + 10;
        } else {
            return -1;
        }
    }

    // 参考自：https://github.com/bajdcc/CEval/blob/master/CEval/CEval.cpp#L105
    lexer_t clexer::next_digit() {
        // 假定这里的数字规则是以0-9开头
        // 正则：^((?:\d+(\.)?\d*)(?:[eE][+-]?\d+)?)([uU])?([fFdCcSsDiIlL])?$
        // 正则：^0[Xx][0-9A-Fa-f]+$
        // 手动实现atof/atoi，并类型转换
        // 其他功能：int溢出转double，e科学记数法
        // 注意：这里不考虑负数，因为估计到歧义（可能是减法呢？）
        auto _type = l_int; // 默认是整型
        auto _postfix = l_none;
        auto i = index;
        auto n = 0ULL, _n = 0ULL;
        auto d = 0.0;
        if (local() == '0' && (local(1) == 'x' || local(1) == 'x')) {
            auto cc = 0;
            // 预先判断十六进制
            for (i += 2; i < length && ((cc = hex2dec(str[i])) != -1); i++) { // 解析整数部分
                if (_type == l_double) { // 小数加位，溢出后自动转换
                    d *= 16.0;
                    d += cc;
                } else { // 整数加位
                    _n = n;
                    n <<= 4;
                    n += cc;
                }
                if (_type == l_int) { // 超过int范围，转为long
                    if (n > INT_MAX)
                        _type = l_long;
                } else if (_type == l_long) { // 超过long范围，转为double
                    if (n >> 4 != _n) {
                        d = (double) _n;
                        d *= 16.0;
                        d += cc;
                        _type = l_double;
                    }
                }
            }
            return digit_return(_type, n, d, i);
        }
        // 判断整数部分
        for (; i < length && (isdigit(str[i])); i++) { // 解析整数部分
            if (_type == l_double) { // 小数加位，溢出后自动转换
                d *= 10.0;
                d += str[i] - '0';
            } else { // 整数加位
                _n = n;
                n *= 10;
                n += str[i] - '0';
            }
            if (_type == l_int) { // 超过int范围，转为long
                if (n > INT_MAX) {
                    _type = l_long;
                }
            } else if (_type == l_long) { // 超过long范围，转为double
                if (n / 10 != _n) {
                    d = (double) _n;
                    d *= 10.0;
                    d += str[i] - '0';
                    _type = l_double;
                }
            }
        }
        if (i == length) { // 只有整数部分
            return digit_return(_type, n, d, i);
        }
        if ((_postfix = digit_type(_type, i)) != l_error) { // 判断有无后缀
            move(i - index);
            return digit_from_integer(_postfix, n) ? _postfix : _type;
        }
        if (str[i] == '.') { // 解析小数部分
            sint l = ++i;
            for (; i < length && (isdigit(str[i])); i++) {
                d *= 10.0;
                d += str[i] - '0';
            }
            l = i - l;
            if (l > 0) {
                d = (double) n + calc_exp(d, -l);
                _type = l_double;
            }
        }
        if (i == length) { // 只有整数部分和小数部分
            return digit_return(_type, n, d, i);
        }
        if ((_postfix = digit_type(_type, i)) != l_error) { // 判断有无后缀
            move(i - index);
            if (_type == l_int)
                return digit_from_integer(_postfix, n) ? _postfix : _type;
            else
                return digit_from_double(_postfix, d) ? _postfix : _type;
        }
        if (str[i] == 'e' || str[i] == 'E') { // 科学计数法强制转成double
            auto neg = false;
            auto e = 0;
            if (_type != l_double) {
                _type = l_double;
                d = (double) n;
            }
            if (++i == length) {
                return digit_return(_type, n, d, i);
            }
            if (!isdigit(str[i])) {
                if (str[i] == '-') { // 1e-1
                    if (++i == length)
                        return digit_return(_type, n, d, i);
                    neg = true;
                } else if (str[i] == '+') { // 1e+1
                    if (++i == length)
                        return digit_return(_type, n, d, i);
                } else {
                    return digit_return(_type, n, d, i);
                }
            }
            for (; i < length && (isdigit(str[i])); i++) { // 解析指数部分
                e *= 10;
                e += str[i] - '0';
            }
            d = calc_exp(d, neg ? -e : e);
        }
        if ((_postfix = digit_type(_type, i)) != l_error) { // 判断有无后缀
            move(i - index);
            if (_type == l_int)
                return digit_from_integer(_postfix, n) ? _postfix : _type;
            else
                return digit_from_double(_postfix, d) ? _postfix : _type;
        }
        return digit_return(_type, n, d, i);
    }

    lexer_t clexer::next_alpha() {
        static const char *valid_id = "?-";
        sint i;
        for (i = index + 1; i < length && (isalnum(str[i]) || str[i] == '_' || strchr(valid_id, str[i])); i++);
        auto s = str.substr(index, i - index);
        bags._identifier = s;
        move(s.length());
        return l_identifier;
    }

    lexer_t clexer::next_space() {
        uint i, j;
        switch (str[index]) {
            case ' ':
            case '\t':
                // 查找连续的空格或Tab
                for (i = index + 1; i < length && (str[i] == ' ' || str[i] == '\t'); i++);
                bags._space = i - index;
                move(bags._space);
                return l_space;
            case '\r':
            case '\n':
                // 查找连续的'\n'或'\r\n'
                for (i = index, j = 0; i < length &&
                                       (str[i] == '\r' || (str[i] == '\n' ? ++j > 0 : false)); i++);
                bags._newline = j;
                move(i - index, bags._newline);
                return l_newline;
        }
        assert(!"space not match"); // cannot reach
        move(1);
        return l_error;
    }

    // 单字符转义
    static int escape(char c) {
        if (c >= '0' && c <= '9') {
            return c - '0';
        } else {
            switch (c) { // like \r, \n, ...
                case 'b':
                    return '\b';
                case 'f':
                    return '\f';
                case 'n':
                    return '\n';
                case 'r':
                    return '\r';
                case 't':
                    return '\t';
                case 'v':
                    return '\v';
                case '\'':
                    return '\'';
                case '\"':
                    return '\"';
                case '\\':
                    return '\\';
                default:
                    return -1;
            }
        }
    }

    lexer_t clexer::next_char() {
        // 提前判定 '\'' 及 '\\' 这两种特殊情况（向前看）
        if (local(1) == '\\' && local(3) == '\'') {
            auto c = local(2);
            auto esc = escape((char) c); // '\?'
            if (esc != -1) {
                bags._char = (char) esc;
                move(4);
                return l_char;
            }
            return record_error(e_invalid_char, 4);
        }
        uint i;
        // 寻找 '\'' 的右边界（限定）
        for (i = 1; index + i < length && str[index + i] != '\'' && i <= 4; i++);
        if (i == 1) { // ''
            return record_error(e_invalid_char, i + 1);
        }
        auto j = index + i;
        i++;
        if (j < length && str[j] == '\'') {
            if (str[index + 1] == '\\') {
                if (i == 3) { // '\'
                    return record_error(e_invalid_char, i);
                }
                // i 不可能为 4
                if (i == 5) { // '\x?'
                    if (str[index + 1] == '\\' && str[index + 2] == 'x') {
                        auto esc = hex2dec(str[index + 3]);
                        if (esc != -1) {
                            bags._char = (char) esc;
                            move(i);
                            return l_char;
                        }
                    }
                    return record_error(e_invalid_char, i);
                }
                // '\x??'
                if (str[index + 1] == '\\' && str[index + 2] == 'x') {
                    auto esc = hex2dec(str[index + 3]); // '\x?_'
                    if (esc != -1) {
                        bags._char = (char) esc;
                        esc = hex2dec(str[index + 4]); // '\x_?'
                        if (esc != -1) {
                            bags._char *= 0x10;
                            bags._char += (char) esc;
                            move(i);
                            return l_char;
                        }
                    }
                }
                return record_error(e_invalid_char, i);
            } else if (i == 3) { // '?'
                bags._char = str[index + 1];
                move((uint) i);
                return l_char;
            }
        }
        return record_error(e_invalid_char, 1);
    }

    lexer_t clexer::next_string() {
        sint i = index;
        auto prev = str[i];
        // 寻找非'\"'的第一个'"'
        for (i++; i < length && (prev == '\\' || (str[i]) != '"'); prev = str[i++]);
        auto j = i;
        if (j == length) { // " EOF
            return record_error(e_invalid_string, i - index);
        }
        std::stringstream ss;
        auto status = 1; // 状态机
        char c = 0;
        for (i = index + 1; i < j;) {
            switch (status) {
                case 1: { // 处理字符
                    if (str[i] == '\\') {
                        status = 2;
                    } else { // '?'
                        ss << str[i];
                    }
                    i++;
                }
                    break;
                case 2: { // 处理转义
                    if (str[i] == 'x') {
                        status = 3;
                        i++;
                    } else {
                        auto esc = escape(str[i]);
                        if (esc != -1) {
                            ss << (char) esc;
                            i++;
                            status = 1;
                        } else {
                            status = 0; // 失败
                        }
                    }
                }
                    break;
                case 3: { // 处理 '\x??' 前一位十六进制数字
                    auto esc = hex2dec(str[i]);
                    if (esc != -1) {
                        c = (char) esc;
                        status = 4;
                        i++;
                    } else {
                        status = 0; // 失败
                    }
                }
                    break;
                case 4: { // 处理 '\x??' 后一位十六进制数字
                    auto esc = hex2dec(str[i]);
                    if (esc != -1) {
                        c *= 10;
                        c += (char) esc;
                        ss << c;
                        status = 1;
                        i++;
                    } else {
                        ss << c;
                        status = 1;
                    }
                }
                    break;
                default: // 失败
                    bags._string = str.substr(index + 1, j - index - 1);
                    move(j - index + 1);
                    return l_string;
            }
        }
        if (status == 1) { // 为初态/终态
            bags._string = ss.str();
            move(j - index + 1);
            return l_string;
        }
        bags._string = str.substr(index + 1, j - index - 1);
        move(j - index + 1);
        return l_string;
    }

    lexer_t clexer::next_operator() {
        auto c = local();
        auto p = sinOp[c];
        if (p == 0) {
            return record_error(e_invalid_operator, 1);
        }
        if (p == op__end) {
            sint i;
            for (i = index + 1; i < length && sinOp[str[i]] == op__end &&
                !(isalnum(str[i]) || isspace(str[i])); i++);
            auto s = str.substr(index, i - index);
            bags._identifier = s;
            move(s.length());
            return l_identifier;
        }
        bags._operator = (operator_t) p;
        move(1);
        return l_operator;
    }

    int clexer::local() {
        if (index < length)
            return str[index];
        return -1;
    }

    int clexer::local(int offset) {
        if (index + offset < length)
            return str[index + offset];
        return -1;
    }

    void clexer::initMap() {
        std::fill(sinOp.begin(), sinOp.end(), op__end);
        for (auto i = op__start + 1; i < op__end; i++) {
            const auto &op = OP_STRING((operator_t) i);
            if (op.length() == 1) {
                sinOp[op[0]] = (operator_t) i; // 操作符第一位char映射
            }
        }
    }

    void clexer::reset() {
        index = 0;
        last_index = 0;

        type = l_none;
        line = 1;
        column = 1;
        last_line = 1;
        last_column = 1;

        records.clear();
    }
}
