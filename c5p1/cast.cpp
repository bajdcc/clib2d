//
// Project: cliblisp
// Created by bajdcc
//

#include <cstring>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "cast.h"

namespace clib {

    cast::cast() {
        init();
    }

    void cast::init() {
        root = new_node(ast_root);
        current = root;
    }

    ast_node *cast::get_root() const {
        return root;
    }

    ast_node *cast::new_node(ast_t type) {
        if (nodes.available() < 64) {
            printf("AST ERROR: 'nodes' out of memory\n");
            throw std::exception();
        }
        auto node = nodes.alloc<ast_node>();
        memset(node, 0, sizeof(ast_node));
        node->flag = type;
        return node;
    }

    ast_node *cast::set_child(ast_node *node, ast_node *child) {
        child->parent = node;
        if (node->child == nullptr) { // 没有孩子
            node->child = child;
            child->prev = child->next = child;
        } else { // 有孩子，添加到末尾
            child->prev = node->child->prev;
            child->next = node->child;
            node->child->prev->next = child;
            node->child->prev = child;
        }
        return node;
    }

    ast_node *cast::set_sibling(ast_node *node, ast_node *sibling) {
        sibling->parent = node->parent;
        sibling->prev = node;
        sibling->next = node->next;
        node->next = sibling;
        return sibling;
    }

    int cast::children_size(ast_node *node) {
        if (!node || !node->child)
            return 0;
        node = node->child;
        auto i = node;
        auto n = 0;
        do {
            n++;
            i = i->next;
        } while (i != node);
        return n;
    }

    ast_node *cast::add_child(ast_node *node) {
        return set_child(current, node);
    }

    ast_node *cast::new_child(ast_t type, bool step) {
        auto node = new_node(type);
        set_child(current, node);
        if (step)
            current = node;
        return node;
    }

    ast_node *cast::new_sibling(ast_t type, bool step) {
        auto node = new_node(type);
        set_sibling(current, node);
        if (step)
            current = node;
        return node;
    }

    void cast::to(ast_to_t type) {
        switch (type) {
            case to_parent:
                current = current->parent;
                break;
            case to_prev:
                current = current->prev;
                break;
            case to_next:
                current = current->next;
                break;
            case to_child:
                current = current->child;
                break;
        }
    }

    void cast::set_str(ast_node *node, const string_t &str) {
        if (strings.available() < 64) {
            printf("AST ERROR: 'strings' out of memory\n");
            throw std::exception();
        }
        auto len = str.length();
        auto s = strings.alloc_array<char>(len + 1);
        memcpy(s, str.c_str(), len);
        s[len] = 0;
        node->data._string = s;
    }

    std::string cast::display_str(const char *str) {
        std::stringstream ss;
        for (auto c = str; *c != 0; c++) {
            if (isprint(*c)) {
                ss << *c;
            } else {
                if (*c == '\n')
                    ss << "\\n";
                else
                    ss << ".";
            }
        }
        return ss.str();
    }

    void cast::reset() {
        nodes.clear();
        strings.clear();
        init();
    }

    template<class T>
    static void ast_recursion(ast_node *node, int level, std::ostream &os, T f) {
        if (node == nullptr)
            return;
        auto i = node;
        if (i->next == i) {
            f(i, level, os);
            return;
        }
        f(i, level, os);
        i = i->next;
        while (i != node) {
            f(i, level, os);
            i = i->next;
        }
    }

    void cast::print(ast_node *node, int level, std::ostream &os) {
        if (node == nullptr)
            return;
        auto rec = [&](auto n, auto l, auto &os) { cast::print(n, l, os); };
        auto type = (ast_t) node->flag;
        switch (type) {
            case ast_root: // 根结点，全局声明
                ast_recursion(node->child, level, os, rec);
                break;
            case ast_env:
            case ast_sub:
            case ast_lambda:
                break;
            case ast_sexpr:
                os << '(';
                ast_recursion(node->child, level + 1, os, rec);
                os << ')';
                break;
            case ast_qexpr:
                os << '`';
                if (node->child == node->child->next) {
                    ast_recursion(node->child, level + 1, os, rec);
                } else {
                    os << '(';
                    ast_recursion(node->child, level + 1, os, rec);
                    os << ')';
                }
                break;
            case ast_literal:
                os << node->data._string;
                break;
            case ast_string:
                os << '"' << display_str(node->data._string) << '"';
                break;
            case ast_char:
                if (isprint(node->data._char))
                    os << '\'' << node->data._char << '\'';
                else if (node->data._char == '\n')
                    os << "'\\n'";
                else
                    os << "'\\x" << std::setiosflags(std::ios::uppercase) << std::hex
                       << std::setfill('0') << std::setw(2)
                       << (unsigned int) node->data._char << '\'';
                break;
            case ast_uchar:
                os << (unsigned int) node->data._uchar;
                break;
            case ast_short:
                os << node->data._short;
                break;
            case ast_ushort:
                os << node->data._ushort;
                break;
            case ast_int:
                os << node->data._int;
                break;
            case ast_uint:
                os << node->data._uint;
                break;
            case ast_long:
                os << node->data._long;
                break;
            case ast_ulong:
                os << node->data._ulong;
                break;
            case ast_float:
                os << node->data._float;
                break;
            case ast_double:
                os << node->data._double;
                break;
        }
        if (node->parent) {
            if ((node->parent->flag == ast_qexpr  || node->parent->flag == ast_sexpr) &&
                node->next != node->parent->child) {
                os << ' ';
            }
        }
    }

    ast_node *cast::index(ast_node *node, int index) {
        auto child = node->child;
        if (child) {
            if (child->next == child) {
                return index == 0 ? child : nullptr;
            }
            auto head = child;
            for (auto i = 0; i < index; ++i) {
                child = child->next;
                if (child == head)
                    return nullptr;
            }
            return child;
        }
        return nullptr;
    }

    ast_node *cast::index(ast_node *node, const string_t &index) {
        auto child = node->child;
        if (child) {
            if (child->next == child) {
                return index == child->child->data._string ? child : nullptr;
            }
            auto head = child;
            auto i = head;
            do {
                if (index == i->child->data._string)
                    return i->child->next;
                i = i->next;
            } while (i != head);
        }
        return nullptr;
    }

    std::tuple<ast_t, string_t> ast_list[] = {
        std::make_tuple(ast_root, "root"),
        std::make_tuple(ast_env, "env"),
        std::make_tuple(ast_sub, "sub"),
        std::make_tuple(ast_lambda, "lambda"),
        std::make_tuple(ast_sexpr, "S-exp"),
        std::make_tuple(ast_qexpr, "Q-exp"),
        std::make_tuple(ast_literal, "literal"),
        std::make_tuple(ast_string, "string"),
        std::make_tuple(ast_char, "char"),
        std::make_tuple(ast_uchar, "uchar"),
        std::make_tuple(ast_short, "short"),
        std::make_tuple(ast_ushort, "ushort"),
        std::make_tuple(ast_int, "int"),
        std::make_tuple(ast_uint, "uint"),
        std::make_tuple(ast_long, "long"),
        std::make_tuple(ast_ulong, "ulong"),
        std::make_tuple(ast_float, "float"),
        std::make_tuple(ast_double, "double"),
    };

    const string_t &cast::ast_str(ast_t type) {
        return std::get<1>(ast_list[type]);
    }
}
