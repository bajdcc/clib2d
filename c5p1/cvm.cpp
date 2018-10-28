//
// Project: cliblisp
// Created by bajdcc
//

#include <iostream>
#include <iomanip>
#include <cstring>
#include "cvm.h"
#include "cast.h"
#include "csub.h"

namespace clib {

    cvm::cvm() {
        builtin();
        set_free_callback();
    }

    void cvm::builtin() {
        global_env = val_obj(ast_env);
        global_env->val._env.env = new cval::cenv_t();
        global_env->val._env.parent = nullptr;
        mem.push_root(global_env);
#if SHOW_ALLOCATE_NODE
        printf("[DEBUG] ALLOC | addr: 0x%p, node: %-10s\n", global_env, cast::ast_str(global_env->type).c_str());
#endif
        builtin_init();
        mem.pop_root();
        mem.protect(global_env);
        builtin_load();
    }

    cval *cvm::val_obj(ast_t type) {
        auto v = mem.alloc<cval>();
        v->type = type;
        v->next = nullptr;
        return v;
    }

    cval *cvm::val_str(ast_t type, const char *str) {
        auto len = strlen(str);
        auto v = (cval *) mem.alloc(sizeof(cval) + len + 1);
        v->type = type;
        v->next = nullptr;
        v->val._string = ((char *) v) + sizeof(cval);
        strncpy((char *) v->val._string, str, len);
        return v;
    }

    cval *cvm::val_sub(const char *name, csub sub) {
        auto len = strlen(name);
        auto v = (cval *) mem.alloc(sizeof(cval) + len + 1);
        v->type = ast_sub;
        v->next = nullptr;
        auto str = ((char *) v) + sizeof(cval);
        strncpy(str, name, len);
        v->val._sub.vm = this;
        v->val._sub.sub = sub;
        return v;
    }

    cval *cvm::val_sub(cval *val) {
        auto name = ((char *) val) + sizeof(cval);
        auto sub = val_sub(name, val->val._sub.sub);
        sub->val._sub.vm = val->val._sub.vm;
        return sub;
    }

    cval *cvm::val_bool(bool flag) {
        auto v = val_obj(ast_int);
        v->val._int = flag ? 1 : 0;
        return v;
    }

    static cval **lambda_env(cval *val) {
        return (cval **)((char *)val + sizeof(cval));
    }

    cval *cvm::val_lambda(cval *param, cval *body, cval *env) {
        auto v = (cval *) mem.alloc(sizeof(cval) + sizeof(cval *));
        v->type = ast_lambda;
        v->next = nullptr;
        mem.push_root(v);
        v->val._lambda.param = copy(param);
        v->val._lambda.body = copy(body);
        if (env == global_env) {
            *lambda_env(v) = new_env(env);
        } else {
            auto _env = *lambda_env(v) = new_env(env->val._env.parent);
            mem.push_root(_env);
            auto &_new_env = *_env->val._env.env;
            for (auto &en : *env->val._env.env) {
                _new_env.insert(std::make_pair(en.first, copy(en.second)));
            }
            mem.pop_root();
        }
        mem.pop_root();
        return v;
    }

    static char *sub_name(cval *val) {
        return (char*)val + sizeof(cval);
    }

    uint cvm::children_size(cval *val) {
        if (!val || (val->type != ast_sexpr && val->type != ast_qexpr))
            return 0;
        return val->val._v.count;
    }

    cval *cvm::conv(ast_node *node, cval *env) {
        if (node == nullptr)
            return nullptr;
        auto type = (ast_t) node->flag;
        switch (type) {
            case ast_root: // 根结点，全局声明
                return conv(node->child, env);
            case ast_sexpr:
                if (!node->child)
                    error("S-exp: missing value");
                if (node->child->flag == ast_literal || node->child->flag == ast_sexpr) {
                    auto v = val_obj(type);
                    mem.push_root(v);
#if SHOW_ALLOCATE_NODE
                    printf("[DEBUG] ALLOC | addr: 0x%p, node: %-10s, count: %d\n", v, cast::ast_str(type).c_str(),
                           cast::children_size(node));
#endif
                    v->val._v.child = nullptr;
                    auto i = node->child;
                    auto local = conv(i, env);
                    v->val._v.child = local;
                    v->val._v.count = 1;
                    i = i->next;
                    while (i != node->child) {
                        v->val._v.count++;
                        local->next = conv(i, env);
                        local = local->next;
                        i = i->next;
                    }
                    mem.pop_root();
                    return v;
                } else {
                    error("S-exp: missing literal");
                }
                break;
            case ast_qexpr:
                if (!node->child) {
                    auto v = val_obj(type);
                    v->val._v.count = 0;
                    v->val._v.child = nullptr;
                    return v;
                } else  {
                    auto v = val_obj(type);
                    mem.push_root(v);
#if SHOW_ALLOCATE_NODE
                    printf("[DEBUG] ALLOC | addr: 0x%p, node: %-10s, count: %d\n", node, cast::ast_str(type).c_str(),
                           cast::children_size(node));
#endif
                    v->val._v.child = nullptr;
                    auto i = node->child;
                    auto local = conv(i, env);
                    v->val._v.child = local;
                    v->val._v.count = 1;
                    i = i->next;
                    while (i != node->child) {
                        v->val._v.count++;
                        local->next = conv(i, env);
                        local = local->next;
                        i = i->next;
                    }
                    mem.pop_root();
                    return v;
                }
            case ast_string: {
                auto v = val_str(type, node->data._string);
#if SHOW_ALLOCATE_NODE
                printf("[DEBUG] ALLOC | addr: 0x%p, node: %-10s, val: %s\n", v, cast::ast_str(type).c_str(),
                       v->val._string);
#endif
                return v;
            }
            case ast_literal: {
                auto v = val_str(type, node->data._string);
#if SHOW_ALLOCATE_NODE
                printf("[DEBUG] ALLOC | addr: 0x%p, node: %-10s, val: %s\n", v, cast::ast_str(type).c_str(),
                       v->val._string);
#endif
                return v;
            }
#if SHOW_ALLOCATE_NODE
#define DEFINE_VAL(t) \
            case ast_##t: { \
                auto v = val_obj(type); \
                v->val._##t = node->data._##t; \
                printf("[DEBUG] ALLOC | addr: 0x%p, node: %-10s, val: ", v, cast::ast_str(type).c_str()); \
                print(v, std::cout); \
                std::cout << std::endl; \
                return v; }
#else
#define DEFINE_VAL(t) \
            case ast_##t: { \
                auto v = val_obj(type); \
                v->val._##t = node->data._##t; \
                return v; }
#endif
            DEFINE_VAL(char)
            DEFINE_VAL(uchar)
            DEFINE_VAL(short)
            DEFINE_VAL(ushort)
            DEFINE_VAL(int)
            DEFINE_VAL(uint)
            DEFINE_VAL(long)
            DEFINE_VAL(ulong)
            DEFINE_VAL(float)
            DEFINE_VAL(double)
#undef DEFINE_VAL
            default:
                break;
        }
        error("invalid val type");
        return nullptr;
    }

    status_t cvm::call(csub fun, cval *val, cval *env, cval **ret) {
        auto frame = eval_mem.alloc<cframe>();
        memset(frame, 0, sizeof(cframe));
        frame->fun = fun;
        frame->val = val;
        frame->env = env;
        frame->ret = ret;
        eval_stack.push_back(frame);
        return s_call;
    }

    cval *cvm::run(ast_node *root) {
        mem.save_stack();
        auto val = conv(root, global_env); // 将AST转换为cval树
        cval *ret = nullptr;
        call(eval, val, global_env, &ret);
        // 自己实现调用栈
        while (!eval_stack.empty()) {
            auto frame = eval_stack.back();
            auto r = frame->fun(this, frame);
            if (r == s_ret) {
                eval_mem.free(frame);
                eval_stack.pop_back();
            }
        }
        assert(ret);
        eval_stack.clear();
        eval_mem.clear();
        eval_tmp.clear();
        return ret;
    }

    void cvm::error(const string_t &info) {
        printf("COMPILER ERROR: %s\n", info.c_str());
        throw std::exception();
    }

    void cvm::print(cval *val, std::ostream &os) {
        if (!val)
            return;
        switch (val->type) {
            case ast_root:
                break;
            case ast_env:
                break;
            case ast_lambda:
                os << "<lambda ";
                print(val->val._lambda.param, os);
                os << ' ';
                print(val->val._lambda.body, os);
                os << ">";
                break;
            case ast_sub:
                os << "<subroutine \"" << sub_name(val) << "\">";
                break;
            case ast_sexpr: {
                    os << '(';
                    auto head = val->val._v.child;
                    while (head) {
                        print(head, os);
                        head = head->next;
                    }
                    os << ')';
                }
                break;
            case ast_qexpr:
                if (val->val._v.count == 0) {
                    os << "nil";
                } else {
                    os << '`';
                    auto head = val->val._v.child;
                    if (val->val._v.count == 1) {
                        print(head, os);
                    } else {
                        os << '(';
                        while (head) {
                            print(head, os);
                            head = head->next;
                        }
                        os << ')';
                    }
                }
                break;
            case ast_literal:
                os << val->val._string;
                break;
            case ast_string:
                os << '"' << cast::display_str(val->val._string) << '"';
                break;
            case ast_char:
                if (isprint(val->val._char))
                    os << '\'' << val->val._char << '\'';
                else if (val->val._char == '\n')
                    os << "'\\n'";
                else
                    os << "'\\x" << std::setiosflags(std::ios::uppercase) << std::hex
                       << std::setfill('0') << std::setw(2)
                       << (unsigned int) val->val._char << '\'';
                break;
            case ast_uchar:
                os << (unsigned int) val->val._uchar;
                break;
            case ast_short:
                os << val->val._short;
                break;
            case ast_ushort:
                os << val->val._ushort;
                break;
            case ast_int:
                os << val->val._int;
                break;
            case ast_uint:
                os << val->val._uint;
                break;
            case ast_long:
                os << val->val._long;
                break;
            case ast_ulong:
                os << val->val._ulong;
                break;
            case ast_float:
                os << val->val._float;
                break;
            case ast_double:
                os << val->val._double;
                break;
        }
        if (val->next) {
            os << ' ';
        }
    }

    void cvm::gc() {
#if SHOW_ALLOCATE_NODE
        dump();
#endif
        mem.gc();
#if SHOW_ALLOCATE_NODE
        printf("[DEBUG] MEM   | Alive objects: %lu\n", mem.count());
#endif
    }

    cval *cvm::copy(cval *val) {
        cval *new_val{nullptr};
        switch (val->type) {
            case ast_root:
            case ast_env:
                error("not supported");
                break;
            case ast_lambda:
                new_val = val_lambda(val->val._lambda.param, val->val._lambda.body, *lambda_env(val));
                break;
            case ast_sub:
                new_val = val_sub(val);
                new_val->val._sub.vm = val->val._sub.vm;
                break;
            case ast_sexpr:
            case ast_qexpr:
                new_val = val_obj(val->type);
                new_val->val._v.count = val->val._v.count;
                if (new_val->val._v.count > 0) {
                    mem.push_root(new_val);
                    auto head = val->val._v.child;
                    new_val->val._v.child = copy(head);
                    if (val->val._v.count > 1) {
                        auto _head = new_val->val._v.child;
                        head = head->next;
                        while (head) {
                            _head->next = copy(head);
                            head = head->next;
                            _head = _head->next;
                        }
                    }
                    mem.pop_root();
                } else {
                    new_val->val._v.child = nullptr;
                }
                break;
            case ast_literal:
            case ast_string:
                new_val = val_str(val->type, val->val._string);
                break;
            case ast_char:
            case ast_uchar:
            case ast_short:
            case ast_ushort:
            case ast_int:
            case ast_uint:
            case ast_long:
            case ast_ulong:
            case ast_float:
            case ast_double:
                new_val = val_obj(val->type);
                std::memcpy((char *) &new_val->val, (char *) &val->val, sizeof(val->val));
                break;
        }
#if SHOW_ALLOCATE_NODE
        printf("[DEBUG] COPY  | addr: 0x%p, node: %-10s, val: ", new_val, cast::ast_str(val->type).c_str());
        print(val, std::cout);
        std::cout << std::endl;
#endif
        return new_val;
    }

    cval *cvm::calc_symbol(const char *sym, cval *env) {
        while (env) {
            auto &_env = *env->val._env.env;
            auto f = _env.find(sym);
            if (f != _env.end()) {
                return copy(f->second);
            }
            env = env->val._env.parent;
        }
        printf("invalid symbol: %s\n", sym);
        error("cannot find symbol");
        return nullptr;
    }

    cval *cvm::new_env(cval *env) {
        auto _env = val_obj(ast_env);
        _env->val._env.env = new cval::cenv_t();
        _env->val._env.parent = env;
        return _env;
    }

    void cvm::set_free_callback() {
#if SHOW_ALLOCATE_NODE
        mem.set_callback([](void *ptr) {
            cval *val = (cval *) ptr;
            printf("[DEBUG] GC    | free: 0x%p, node: %-10s, ", ptr, cast::ast_str(val->type).c_str());
            if (val->type == ast_sexpr || val->type == ast_qexpr) {
                printf("count: %lu\n", children_size(val));
            } else if (val->type == ast_literal) {
                printf("id: %s\n", val->val._string);
            } else if (val->type == ast_env) {
                printf("env: %d\n", val->val._env.env->size());
                delete val->val._env.env;
            } else if (val->type == ast_sub) {
                printf("name: %s\n", sub_name(val));
            } else {
                printf("val: ");
                print(val, std::cout);
                std::cout << std::endl;
            }
        });
        mem.set_dump_callback([](void *ptr, int level) {
            cval *val = (cval *) ptr;
            printf("[DEBUG] DUMP  | ");
            std::cout << std::setfill('_') << std::setw(level << 2) << "";
            printf("addr: 0x%p, node: %-10s, ", ptr, cast::ast_str(val->type).c_str());
            if (val->type == ast_sexpr || val->type == ast_qexpr) {
                printf("count: %lu\n", children_size(val));
            } else if (val->type == ast_literal) {
                printf("id: %s\n", val->val._string);
            } else if (val->type == ast_env) {
                printf("env: %d\n", val->val._env.env->size());
            } else if (val->type == ast_sub) {
                printf("name: %s\n", sub_name(val));
            } else {
                printf("val: ");
                print(val, std::cout);
                std::cout << std::endl;
            }
        });
#else
        mem.set_callback([](void *ptr) {
            cval *val = (cval *) ptr;
            if (val->type == ast_env) {
                delete val->val._env.env;
            }
        });
#endif
    }

    void cvm::save() {
        mem.save_stack();
    }

    void cvm::restore() {
        mem.restore_stack();
        eval_stack.clear();
        eval_mem.clear();
        eval_tmp.clear();
    }

    void cvm::dump() {
#if SHOW_ALLOCATE_NODE
        mem.dump(std::cout);
#endif
    }
}
