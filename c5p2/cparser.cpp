//
// Project: cliblisp
// Author: bajdcc
//
#include <sstream>
#include "cparser.h"
#include "clexer.h"
#include "cast.h"

namespace clib {

    cparser::cparser(string_t str)
        : lexer(str) {}

    ast_node *cparser::parse() {
        // 清空词法分析结果
        lexer.reset();
        // 清空AST
        ast.reset();
        // 语法分析（递归下降）
        program();
        return ast.get_root();
    }

    ast_node *cparser::root() const {
        return ast.get_root();
    }

    void cparser::next() {
        lexer_t token;
        do {
            token = lexer.next();
            if (token == l_error) {
                auto err = lexer.recent_error();
                printf("[%04d:%03d] %-12s - %s\n",
                       err.line,
                       err.column,
                       ERROR_STRING(err.err).c_str(),
                       err.str.c_str());
            }
        } while (token == l_newline || token == l_space || token == l_error);
#if 0
        if (token != l_end) {
            printf("[%04d:%03d] %-12s - %s\n",
                   lexer.get_last_line(),
                   lexer.get_last_column(),
                   LEX_STRING(lexer.get_type()).c_str(),
                   lexer.current().c_str());
        }
#endif
    }

    void cparser::program() {
        next();
        ast.add_child(lambda(false));
        if (!lexer.is_type(l_end)) {
            error("incomplete call");
        }
    }

    void cparser::expect(bool flag, const string_t &info) {
        if (!flag) {
            error(info);
        }
    }

    void cparser::match_operator(operator_t type) {
        expect(lexer.is_operator(type), string_t("expect operator " + OPERATOR_STRING(type)));
        next();
    }

    void cparser::match_type(lexer_t type) {
        expect(lexer.is_type(type), string_t("expect type " + LEX_STRING(type)));
        next();
    }

    void cparser::match_number() {
        expect(lexer.is_number(), "expect number");
        next();
    }

    void cparser::match_integer() {
        expect(lexer.is_integer(), "expect integer");
        next();
    }

    void cparser::error(const string_t &info) {
        printf("[%04d:%03d] ERROR: %s\n", lexer.get_line(), lexer.get_column(), info.c_str());
        throw std::exception();
    }

    ast_node *cparser::lambda(bool paran) {
        if (paran) {
            match_operator(op_lparan);
            auto node = ast.new_node(ast_sexpr);
            while (!lexer.is_operator(op_rparan)) {
                cast::set_child(node, object());
            }
            match_operator(op_rparan);
            return node;
        } else {
            auto child = object();
            if (lexer.is_type(l_end)) {
                return child;
            }
            auto node = ast.new_node(ast_sexpr);
            cast::set_child(node, child);
            while (!lexer.is_type(l_end)) {
                cast::set_child(node, object());
            }
            return node;
        }
    }

    ast_node *cparser::object() {
        if (lexer.is_type(l_end)) { // 结尾
            error("unexpected token EOF of expression");
        }
        if (lexer.is_type(l_operator)) {
            if (lexer.is_operator(op_lparan)) {
                return lambda();
            }
            if (lexer.is_operator(op_quote)) {
                match_operator(op_quote);
                auto obj = object();
                if (obj->flag == ast_sexpr) {
                    obj->flag = ast_qexpr;
                    return obj;
                } else {
                    auto node = ast.new_node(ast_qexpr);
                    cast::set_child(node, obj);
                    return node;
                }
            }
            auto node = ast.new_node(ast_literal);
            ast.set_str(node, OP_STRING(lexer.get_operator()));
            match_type(l_operator);
            return node;
        }
        if (lexer.is_type(l_identifier)) {
            auto node = ast.new_node(ast_literal);
            ast.set_str(node, lexer.get_identifier());
            match_type(l_identifier);
            return node;
        }
        if (lexer.is_number()) {
            ast_node *node = nullptr;
            auto type = lexer.get_type();
            switch (type) {
#define DEFINE_NODE_INT(t) \
            case l_##t: \
                node = ast.new_node(ast_##t); \
                node->data._##t = lexer.get_##t(); \
                break;
                DEFINE_NODE_INT(char)
                DEFINE_NODE_INT(uchar)
                DEFINE_NODE_INT(short)
                DEFINE_NODE_INT(ushort)
                DEFINE_NODE_INT(int)
                DEFINE_NODE_INT(uint)
                DEFINE_NODE_INT(long)
                DEFINE_NODE_INT(ulong)
                DEFINE_NODE_INT(float)
                DEFINE_NODE_INT(double)
#undef DEFINE_NODE_INT
                default:
                    error("invalid number");
                    break;
            }
            match_number();
            return node;
        }
        if (lexer.is_type(l_string)) {
            auto node = ast.new_node(ast_string);
            ast.set_str(node, lexer.get_string());
            match_type(l_string);
            return node;
        }
        error("invalid type");
        return nullptr;
    }
}
