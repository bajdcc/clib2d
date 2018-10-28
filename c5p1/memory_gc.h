//
// Project: cliblisp
// Author: bajdcc
//

#ifndef CLIBLISP_MEMORY_GC_H
#define CLIBLISP_MEMORY_GC_H

#include <functional>
#include <unordered_set>
#include <cassert>
#include <vector>
#include "memory.h"
#include "types.h"

#define SHOW_GC 1

namespace clib {

    template<size_t DefaultSize = default_allocator<>::DEFAULT_ALLOC_BLOCK_SIZE>
    class legacy_memory_gc {
    public:
        struct gc_header {
            gc_header *child;
            gc_header *next;
            gc_header *prev;
        };

        using memory_pool_t = memory_pool<DefaultSize>;
        using blk_t = typename memory_pool_t::block;
        static const auto BLOCK_MARK = memory_pool_t::BLOCK_MARK;
        static const auto GC_HEADER_SIZE = sizeof(gc_header);
        static const auto GC_BLOCK_SIZE = sizeof(blk_t);

        legacy_memory_gc() {
            stack_roots.push_back(&stack_head);
        }

        static gc_header *header(void *ptr) {
            return static_cast<gc_header *>((void *) (static_cast<char *>(ptr) - GC_HEADER_SIZE));
        }

        static void *data(void *ptr) {
            return static_cast<void *>(static_cast<char *>(ptr) + GC_HEADER_SIZE);
        }

        static blk_t *block(void *ptr) {
            return static_cast<blk_t *>((void *) (static_cast<char *>(ptr) - GC_BLOCK_SIZE));
        }

        static void set_marked(void *ptr, bool value) {
            auto blk = block(ptr);
            if (value) {
                blk->flag |= 1 << BLOCK_MARK;
            } else {
                blk->flag &= ~(1 << BLOCK_MARK);
            }
        }

        static uint is_marked(void *ptr) {
            auto blk = block(ptr);
            return (blk->flag & (1 << BLOCK_MARK)) != 0 ? 1 : 0;
        }

        template<class T>
        T *alloc() {
            return static_cast<T *>(alloc(sizeof(T)));
        }

        void *alloc(size_t size) {
            auto new_node = static_cast<gc_header *>((void *) memory.template alloc_array<char>(GC_HEADER_SIZE + size));
            assert(new_node);
            memset(new_node, 0, GC_HEADER_SIZE + size);
            auto &top = stack_roots.back();
            if (top->child) {
                new_node->prev = top->child->prev;
                new_node->prev->next = new_node;
                new_node->next = top->child;
                new_node->next->prev = new_node;
            } else {
                top->child = new_node;
                new_node->next = new_node->prev = new_node;
            }
            objects.push_back(new_node);
            return (void *) (static_cast<char *>((void *) new_node) + GC_HEADER_SIZE);
        }

        void push_root(void *ptr) {
            stack_roots.push_back(header(ptr));
        }

        void pop_root() {
            stack_roots.pop_back();
        }

        void link(void *parent, void *ptr) {
            auto _parent = header(parent);
            auto _ptr = header(ptr);
            _link(_parent, _ptr);
        }

        void unlink(void *parent, void *ptr) {
            auto _parent = header(parent);
            auto _ptr = header(ptr);
            _unlink(_parent, _ptr);
        }

        void unlink(void *ptr) {
            auto _parent = stack_roots.back();
            auto _ptr = header(ptr);
            _unlink(_parent, _ptr);
        }

        void protect(void *ptr) {
            roots.insert(header(ptr));
        }

        void unprotect(void *ptr) {
            roots.erase(ptr);
        }

        void gc() {
            mark();
            sweep();
        }

        size_t count() const {
            return objects.size();
        }

        void set_callback(std::function<void(void *)> callback) {
            gc_callback = callback;
        }

        void set_dump_callback(std::function<void(void *, int)> callback) {
            dump_callback = callback;
        }

        void save_stack() {
            saved_stack = stack_roots.size();
        }

        void restore_stack() {
            stack_roots.erase(stack_roots.begin() + saved_stack, stack_roots.end());
        }

        void dump(std::ostream &os) {
            memory.dump(os);
            dump_tree();
        }

    private:
        void mark_children(gc_header *ptr) {
            if (ptr->child) {
                auto i = ptr->child;
                set_marked(i, true);
                mark_children(i);
                i = i->next;
                while (i != ptr->child) {
                    set_marked(i, true);
                    mark_children(i);
                    i = i->next;
                }
            }
        }

        void _link(gc_header *parent, gc_header *ptr) {
            if (parent->child) {
                ptr->prev = parent->child->prev;
                ptr->prev->next = ptr;
                ptr->next = parent->child;
                ptr->next->prev = ptr;
            } else {
                parent->child = ptr;
                ptr->next = ptr->prev = ptr;
            }
        }

        void _unlink(gc_header *parent, gc_header *ptr) {
            if (parent->child) {
                auto i = parent->child;
                if (i->next == i) {
                    parent->child = nullptr;
                    return;
                }
                if (i == ptr) {
                    parent->child = i->next;
                    i->prev->next = parent->child;
                } else {
                    i = i->next;
                }
                while (i != parent->child) {
                    if (i->next == ptr) {
                        if (i->next->next == parent->child) {
                            i->next = parent->child;
                            parent->child->prev = i;
                        } else {
                            i->next->next->prev = i;
                            i->next = i->next->next;
                        }
                        break;
                    } else {
                        i = i->next;
                    }
                }
            }
        }

        void mark() {
            stack_roots.front()->child = nullptr;
            for (auto &root : roots) {
                set_marked(root, true);
                mark_children(root);
            }
            for (auto it = stack_roots.begin() + 1; it != stack_roots.end(); it++) {
                auto &root = *it;
                set_marked(root, true);
                mark_children(root);
            }
        }

        void sweep() {
            for (auto it = objects.begin(); it != objects.end();) {
                auto &obj = *it;
                if (is_marked(obj)) {
                    set_marked(obj, false);
                    it++;
                } else {
#if SHOW_GC
                    if (gc_callback)
                        gc_callback((void *) data((void *) obj));
#endif
                    memory.free(obj);
                    it = objects.erase(it);
                }
            }
        }

        void dump_children(gc_header *ptr, int level) {
            dump_callback(data(ptr), level);
            if (ptr->child) {
                auto i = ptr->child;
                dump_children(i, level + 1);
                i = i->next;
                while (i != ptr->child) {
                    dump_children(i, level + 1);
                    i = i->next;
                }
            }
        }

        void dump_tree() {
            if (!dump_callback)
                return;
            for (auto &root : roots) {
                dump_children(root, 0);
            }
            for (auto it = stack_roots.begin() + 1; it != stack_roots.end(); it++) {
                auto &root = *it;
                dump_children(root, 0);
            }
        }

    private:
        size_t saved_stack{0};
        gc_header stack_head{nullptr, nullptr, nullptr};
        std::function<void(void *)> gc_callback{[](void *) {}};
        std::function<void(void *, int)> dump_callback{[](void *, int) {}};
        std::vector<gc_header *> objects;
        std::vector<gc_header *> stack_roots;
        std::unordered_set<gc_header *> roots;
        memory_pool <DefaultSize> memory;
    };

    template<size_t DefaultSize = default_allocator<>::DEFAULT_ALLOC_BLOCK_SIZE>
    using memory_pool_gc = legacy_memory_gc<DefaultSize>;
}

#endif //CLIBLISP_MEMORY_GC_H
