//
// Project: cliblisp
// Created by bajdcc
//

#include <iostream>
#include <sstream>
#include <tuple>
#include "cparser.h"
#include "cvm.h"

#define TEST(a,b) std::make_tuple(a, b)

int main(int argc, char *argv[]) {
    clib::cvm vm;
    auto codes = std::vector<std::tuple<std::string, std::string>>{
            TEST("+ 1 2", "3"),
            TEST("* 1 2 3 4 5 6", "720"),
            TEST("- 8 4 2 9 8 ", "-15"),
            TEST(R"(+ "Hello" " " "world!")", R"("Hello world!")"),
            TEST("eval 5", "5"),
            TEST("eval `(+ 1 2)", "3"),
            TEST("eval (+ 1 2)", "3"),
            TEST("`a", "`a"),
            TEST("`(a b c)", "`(a b c)"),
            TEST(R"(+ "Project: " __project__ ", author: " __author__)", R"("Project: cliblisp, author: bajdcc")"),
            TEST("+", R"(<subroutine "+">)"),
            // tests for lis.py
            TEST(R"(quote (testing 1 2.0 -3.14e159))", "`(testing 1 2 -3.14e+159)"),
            TEST(R"(+ 2 2)", "4"),
            TEST(R"(+ (* 2 100) (* 1 10))", "210"),
            TEST(R"(if (> 6 5) `(+ 1 1) `(+ 2 2))", "2"),
            TEST(R"(if (< 6 5) `(+ 1 1) `(+ 2 2))", "4"),
            TEST(R"(def `x 3)", "3"),
            TEST(R"(x)", "3"),
            TEST(R"(+ x x)", "6"),
            TEST(R"(begin (def `x 1) (def `x (+ x 1)) (+ x 1))", "3"),
            TEST(R"((\ `(x) `(+ x x)) 5)", "10"),
            TEST(R"(def `twice (\ `(x) `(* 2 x)))", R"(<lambda `x `(* 2 x)>)"),
            TEST(R"(twice 5)", "10"),
            TEST(R"(def `compose (\ `(f g) `(\ `(x) `(f (g x)))))", R"(<lambda `(f g) `(\ `x `(f (g x)))>)"),
            TEST(R"((compose list twice) 5)", "`10"),
            TEST(R"(def `repeat (\ `(f) `(compose f f)))", "<lambda `f `(compose f f)>"),
            TEST(R"((repeat twice) 5)", "20"),
            TEST(R"((repeat (repeat twice)) 5)", "80"),
            TEST(R"(def `fact (\ `(n) `(if (<= n 1) `1 `(* n (fact (- n 1))))))",
                 R"(<lambda `n `(if (<= n 1) `1 `(* n (fact (- n 1))))>)"),
            TEST(R"(fact 3)", "6"),
            TEST(R"(fact 50)", "30414093201713378043612608166064768844377641568960512000000000000"),
            TEST(R"(fact 12)", "479001600"),
            TEST(R"(def `abs (\ `(n) `((if (> n 0) `+ `-) 0 n)))", "<lambda `n `((if (> n 0) `+ `-) 0 n)>"),
            TEST(R"(abs -3)", "3"),
            TEST(R"(list (abs -3) (abs 0) (abs 3))", "`(3 0 3)"),
            TEST(R"(def `combine (\ `(f)
                 `(\ `(x y)
                 `(if (null? x) `nil
                 `(f (list (car x) (car y))
                 ((combine f) (cdr x) (cdr y)))))))",
                 R"(<lambda `f `(\ `(x y) `(if (null? x) `nil `(f (list (car x) (car y)) ((combine f) (cdr x) (cdr y)))))>)"),
            TEST(R"(def `zip (combine cons))",
                    "<lambda `(x y) `(if (null? x) `nil `(f (list (car x) (car y)) ((combine f) (cdr x) (cdr y))))>"),
            TEST(R"(zip (list 1 2 3 4) (list 5 6 7 8))", "`(`(1 5) `(2 6) `(3 7) `(4 8))"),
            TEST(R"(def `riff-shuffle (\ `(deck) `(begin
                 (def `take (\ `(n seq) `(if (<= n 0) `nil `(cons (car seq) (take (- n 1) (cdr seq))))))
                 (def `drop (\ `(n seq) `(if (<= n 0) `seq `(drop (- n 1) (cdr seq)))))
                 (def `mid (\ `(seq) `(/ (len seq) 2)))
                 ((combine append) (take (mid deck) deck) (drop (mid deck) deck)))))",
                 R"(<lambda `deck `(begin)"
                 R"( (def `take (\ `(n seq) `(if (<= n 0) `nil `(cons (car seq) (take (- n 1) (cdr seq)))))))"
                 R"( (def `drop (\ `(n seq) `(if (<= n 0) `seq `(drop (- n 1) (cdr seq))))))"
                 R"( (def `mid (\ `seq `(/ (len seq) 2))))"
                 R"( ((combine append) (take (mid deck) deck) (drop (mid deck) deck)))>)"),
            TEST(R"(riff-shuffle (list 1 2 3 4 5 6 7 8))", "`(1 5 2 6 3 7 4 8)"),
            TEST(R"((repeat riff-shuffle) (list 1 2 3 4 5 6 7 8))",  "`(1 3 5 7 2 4 6 8)"),
            TEST(R"(riff-shuffle (riff-shuffle (riff-shuffle (list 1 2 3 4 5 6 7 8))))", "`(1 2 3 4 5 6 7 8)"),
            TEST(R"(def `apply (\ `(item L) `(eval (cons item L))))", "<lambda `(item L) `(eval (cons item L))>"),
            TEST(R"(apply + `(1 2 3))", "6"),
            TEST(R"(def `sum (\ `n `(if (< n 2) `1 `(+ n (sum (- n 1))))))",
                 "<lambda `n `(if (< n 2) `1 `(+ n (sum (- n 1))))>"),
            TEST(R"(sum 10)", "55"),
            TEST(R"(def `Y (\ `f `((\ `self `(f (\ `x `((self self) x)))) (\ `self `(f (\ `x `((self self) x)))))))",
                 R"(<lambda `f `((\ `self `(f (\ `x `((self self) x)))) (\ `self `(f (\ `x `((self self) x)))))>)"),
            TEST(R"(def `Y_fib (\ `f `(\ `n `(if (<= n 2) `1 `(+ (f (- n 1)) (f (- n 2)))))))",
                 R"(<lambda `f `(\ `n `(if (<= n 2) `1 `(+ (f (- n 1)) (f (- n 2)))))>)"),
            TEST(R"((Y Y_fib) 5)", "5"),
            TEST(R"((def `range (\ `(a b) `(if (== a b) `nil `(cons a (range (+ a 1) b))))))",
                "<lambda `(a b) `(if (== a b) `nil `(cons a (range (+ a 1) b)))>"),
            TEST(R"(range 1 10)", "`(1 2 3 4 5 6 7 8 9)"),
            TEST(R"(apply + (range 1 10))", "45"),
    };
    auto i = 0;
    auto failed = 0;
    std::stringstream ss;
    std::string ast, out;
    for (auto &code : codes) {
        vm.save();
        try {
            ast = std::get<0>(code);
            out.clear();
            clib::cparser p(ast);
            auto root = p.parse();
            auto val = vm.run(root);
            std::cout << "TEST #" << (++i) << "> ";
            ss.str("");
            clib::cast::print(root, 0, ss);
            ast = ss.str();
            ss.str("");
            clib::cvm::print(val, ss);
            out = ss.str();
            auto &right = std::get<1>(code);
            if (out == right) {
                std::cout << "[PASSED] " << ast << "  =>  " << out;
            } else {
                std::cout << "[ERROR ] " << ast << "  =>  " << out << "   REQUIRE: " << right;
                failed++;
            }
            std::cout << std::endl;
            vm.gc();
        } catch (const std::exception &e) {
            failed++;
            std::cout << "TEST #" << (++i) << "> [ERROR ] " << ast << std::endl;
            //printf("RUNTIME ERROR: %s\n", e.what());
            vm.restore();
            vm.gc();
        }
    }
    std::cout << "==== ALL TEST PASSED [" << (i - failed) << "/" << i << "] ====" << std::endl;
}
