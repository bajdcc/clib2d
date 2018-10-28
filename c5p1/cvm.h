//
// Project: cliblisp
// Created by bajdcc
//

#ifndef CLIBLISP_CVM_H
#define CLIBLISP_CVM_H

#define VM_MEM (32 * 1024)
#define VM_EVAL (32 * 1024)
#define VM_TMP (32 * 1024)
#define SHOW_ALLOCATE_NODE 0

#include <vector>
#include "cast.h"
#include "memory_gc.h"

namespace clib {

    class cvm;
    struct cframe;

    enum status_t {
        s_ret,
        s_call,
        s_error,
    };

    using ctmp = void *;

    struct cval {
        using cenv_t = std::unordered_map<std::string, cval *>;
        using csub_t = status_t (*)(cvm *vm, cframe *frame);
        ast_t type;
        cval *next;
        union {
            struct {
                uint count;
                cval *child;
            } _v;
            struct {
                cval *parent;
                cenv_t *env;
            } _env;
            struct {
                void *vm;
                csub_t sub;
            } _sub;
            struct {
                cval *param;
                cval *body;
            } _lambda;
            const char *_string;
#define DEFINE_CVAL(t) LEX_T(t) _##t;
            DEFINE_CVAL(char)
            DEFINE_CVAL(uchar)
            DEFINE_CVAL(short)
            DEFINE_CVAL(ushort)
            DEFINE_CVAL(int)
            DEFINE_CVAL(uint)
            DEFINE_CVAL(long)
            DEFINE_CVAL(ulong)
            DEFINE_CVAL(float)
            DEFINE_CVAL(double)
#undef DEFINE_CVAL
        } val;
    };

    using cenv = cval::cenv_t;
    using csub = cval::csub_t;

    struct cframe {
        csub fun;
        cval *val, *env, **ret;
        void *arg;
    };

    class cvm {
    public:
        cvm();
        ~cvm() = default;

        cvm(const cvm &) = delete;
        cvm &operator=(const cvm &) = delete;

        friend class builtins;

        cval *run(ast_node *root);
        void gc();

        static void print(cval *val, std::ostream &os);

        void save();
        void restore();

        void error(const string_t &info);

        void dump();

    private:
        void builtin();
        void builtin_init();
        void builtin_load();
        cval *conv(ast_node *node, cval *env);

        status_t call(csub fun, cval *val, cval *env, cval **ret);

        int calc(int op, ast_t type, cval *r, cval *v, cval *env);
        cval *calc_op(int op, cval *val, cval *env);
        cval *calc_symbol(const char *sym, cval *env);
        cval *calc_sub(const char *sub, cval *val, cval *env);

        static status_t eval(cvm *vm, cframe *frame);
        static status_t eval_one(cvm *vm, cframe *frame);
        static status_t eval_child(cvm *vm, cframe *frame);

        cval *val_obj(ast_t type);
        cval *val_str(ast_t type, const char *str);
        cval *val_sub(const char *name, csub sub);
        cval *val_sub(cval *val);
        cval *val_bool(bool flag);
        cval *val_lambda(cval *param, cval *body, cval *env);

        cval *copy(cval *val);
        cval *new_env(cval *env);

        static uint children_size(cval *val);

        void set_free_callback();

    private:
        cval *global_env{nullptr};
        memory_pool_gc<VM_MEM> mem;
        std::vector<cframe *> eval_stack;
        memory_pool<VM_EVAL> eval_mem;
        memory_pool<VM_TMP> eval_tmp;
    };
}

#endif //CLIBLISP_CVM_H
