//
// Project: cliblisp
// Created by bajdcc
//

#ifndef CLIBLISP_CSUB_H
#define CLIBLISP_CSUB_H

#include "cvm.h"

namespace clib {
    struct cval;
    class cvm;

    class builtins {
    public:
        static status_t add(cvm *vm, cframe *frame);
        static status_t sub(cvm *vm, cframe *frame);
        static status_t mul(cvm *vm, cframe *frame);
        static status_t div(cvm *vm, cframe *frame);
        static status_t quote(cvm *vm, cframe *frame);
        static status_t list(cvm *vm, cframe *frame);
        static status_t car(cvm *vm, cframe *frame);
        static status_t cdr(cvm *vm, cframe *frame);
        static status_t cons(cvm *vm, cframe *frame);

        static status_t def(cvm *vm, cframe *frame);
        static status_t lambda(cvm *vm, cframe *frame);
        static status_t call_lambda(cvm *vm, cframe *frame);
        static status_t call_eval(cvm *vm, cframe *frame);

        static status_t lt(cvm *vm, cframe *frame);
        static status_t le(cvm *vm, cframe *frame);
        static status_t gt(cvm *vm, cframe *frame);
        static status_t ge(cvm *vm, cframe *frame);
        static status_t eq(cvm *vm, cframe *frame);
        static status_t ne(cvm *vm, cframe *frame);

        static status_t begin(cvm *vm, cframe *frame);
        static status_t _if(cvm *vm, cframe *frame);

        static status_t len(cvm *vm, cframe *frame);
        static status_t append(cvm *vm, cframe *frame);

        static status_t is_null(cvm *vm, cframe *frame);
        static status_t type(cvm *vm, cframe *frame);
        static status_t str(cvm *vm, cframe *frame);

        static status_t print(cvm *vm, cframe *frame);
    };
}

#endif //CLIBLISP_CSUB_H
