//
// Project: cliblisp
// Author: bajdcc
//
#ifndef CLIBLISP_PARSER_H
#define CLIBLISP_PARSER_H

#include "types.h"
#include "clexer.h"
#include "cast.h"

namespace clib {

    class cparser {
    public:
        explicit cparser(string_t str);
        ~cparser() = default;

        cparser(const cparser &) = delete;
        cparser &operator=(const cparser &) = delete;

        ast_node *parse();
        ast_node *root() const;

    private:
        void next();

        void program();
        ast_node *lambda(bool paran = true);
        ast_node *object();

    private:
        void expect(bool, const string_t &);
        void match_operator(operator_t);
        void match_type(lexer_t);
        void match_number();
        void match_integer();

        void error(const string_t &);

    private:
        lexer_t base_type{l_none};

    private:
        clexer lexer;
        cast ast;
    };
}
#endif //CLIBLISP_PARSER_H