// AVLMap.hpp
//
// An ordered key -> value map backed by an AVL tree (a self-balancing binary
// search tree, Adelson-Velsky and Landis, 1962).
//
// Who uses it: FlightGraph (include/fp/graph/FlightGraph.hpp) stores the
// "phone book" from airport code to internal id in one of these:
//     "BHO" -> 1432, "DEL" -> 2017, ...
//
// Why a balanced tree here and not a hash map. Be precise about this in an
// interview, because it is NOT about memory: both use O(n) space.
//   1. Ordered iteration. Codes come out sorted, so "list every airport" and
//      "autocomplete DE -> DED, DEL, DEN, ..." are a walk from a starting
//      key. A hash map scatters keys and cannot do prefix search.
//   2. Worst case guarantees. Every lookup is O(log n), always. A hash map is
//      O(1) on average but has no worst-case bound.
//   3. It is only used at the edges of the program (turning what the user
//      typed into an int). The algorithms never touch it; they work on plain
//      vectors indexed by those ints (see FlightGraph::neighbors).
//
// The AVL rule: for every node, the heights of its left and right subtrees
// differ by at most 1. That keeps the height under about 1.44 * log2(n), so
// find and insert are O(log n). After every insert we walk back up and fix
// any node that broke the rule using one or two "rotations":
//
//     right rotation at y (left side too tall):
//
//            y                x        before    after
//           / \              / \       ------    -----
//          x   C    --->    A   y      y on top  x on top
//         / \                  / \     B under x B under y
//        A   B                B   C
//
//   Order (A < x < B < y < C) is preserved, only the shape changes.
//
// Deliberately NOT implemented: erase. Airports are loaded once and never
// removed, and deletion is the most bug-prone part of an AVL tree. The "what
// if a hub shuts down" analysis blocks airports with a flag instead of
// deleting them (see src/analysis/HubAnalysis.cpp).

#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <memory>
#include <utility>

#include "fp/Errors.hpp"  // KeyNotFound

namespace fp {

template <typename K, typename V, typename Compare = std::less<K>>
class AVLMap {
private:
    struct Node {
        Node(K k, V v) : key(std::move(k)), value(std::move(v)) {}
        K key;
        V value;
        int height = 1;  // a leaf has height 1, an empty subtree has 0
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
    };

public:
    AVLMap() = default;

    // Insert a new key. Returns false (and leaves the old value alone) if the
    // key was already present. FlightGraph::addAirport uses the return value
    // to reject duplicate airport codes.
    bool insert(const K& key, V value) {
        bool inserted = false;
        insertAt(root_, key, std::move(value), /*overwrite=*/false, inserted);
        if (inserted) ++size_;
        return inserted;
    }

    // Insert, or replace the value if the key already exists.
    void insertOrAssign(const K& key, V value) {
        bool inserted = false;
        insertAt(root_, key, std::move(value), /*overwrite=*/true, inserted);
        if (inserted) ++size_;
    }

    // Returns a pointer to the value, or nullptr if the key is absent.
    // Plain binary search tree descent: O(height) = O(log n).
    const V* find(const K& key) const {
        const Node* node = root_.get();
        while (node) {
            if (less_(key, node->key)) {
                node = node->left.get();
            } else if (less_(node->key, key)) {
                node = node->right.get();
            } else {
                return &node->value;
            }
        }
        return nullptr;
    }

    V* find(const K& key) {
        return const_cast<V*>(static_cast<const AVLMap&>(*this).find(key));
    }

    // Like find, but throws KeyNotFound instead of returning nullptr.
    const V& at(const K& key) const {
        const V* value = find(key);
        if (!value) {
            throw KeyNotFound("AVLMap::at: key not found");
        }
        return *value;
    }

    bool contains(const K& key) const { return find(key) != nullptr; }
    std::size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }

    // Height of the whole tree (0 when empty). Tests use it to check that the
    // tree really stays logarithmic even when keys arrive already sorted,
    // which is the worst case for an unbalanced BST.
    int height() const { return heightOf(root_.get()); }

    // Visit every (key, value) in ascending key order.
    template <typename Fn>
    void forEach(Fn fn) const {
        walkFrom(root_.get(), nullptr, fn);
    }

    // Visit (key, value) pairs in ascending order starting at the first key
    // that is >= `from`. `fn` returns true to keep going, false to stop.
    // This is the building block for prefix search: start at the prefix and
    // stop at the first key that no longer starts with it.
    // Cost: O(log n + number of keys visited).
    template <typename Fn>
    void forEachFrom(const K& from, Fn fn) const {
        walkFrom(root_.get(), &from, fn);
    }

    // Debug helper for tests: verifies BST ordering, the AVL balance rule and
    // the stored heights on every node.
    bool checkInvariants() const { return check(root_.get(), nullptr, nullptr) >= 0; }

private:
    static int heightOf(const Node* node) { return node ? node->height : 0; }

    static void updateHeight(Node& node) {
        node.height = 1 + std::max(heightOf(node.left.get()), heightOf(node.right.get()));
    }

    // Positive: left side taller. Negative: right side taller.
    static int balanceOf(const Node& node) {
        return heightOf(node.left.get()) - heightOf(node.right.get());
    }

    // See the right-rotation picture in the header comment.
    static void rotateRight(std::unique_ptr<Node>& y) {
        std::unique_ptr<Node> x = std::move(y->left);
        y->left = std::move(x->right);  // B moves under y
        updateHeight(*y);
        x->right = std::move(y);        // y goes under x
        updateHeight(*x);
        y = std::move(x);               // x becomes the subtree root
    }

    // Mirror image of rotateRight.
    static void rotateLeft(std::unique_ptr<Node>& x) {
        std::unique_ptr<Node> y = std::move(x->right);
        x->right = std::move(y->left);
        updateHeight(*x);
        y->left = std::move(x);
        updateHeight(*y);
        x = std::move(y);
    }

    // Restore the AVL rule at `node` after one of its subtrees grew by one.
    // The four classic cases:
    //   left-left   -> rotate right
    //   left-right  -> rotate left at the child, then right at the node
    //   right-right -> rotate left
    //   right-left  -> rotate right at the child, then left at the node
    static void rebalance(std::unique_ptr<Node>& node) {
        updateHeight(*node);
        int balance = balanceOf(*node);
        if (balance > 1) {
            if (balanceOf(*node->left) < 0) {
                rotateLeft(node->left);
            }
            rotateRight(node);
        } else if (balance < -1) {
            if (balanceOf(*node->right) > 0) {
                rotateRight(node->right);
            }
            rotateLeft(node);
        }
    }

    // Recursive insert. Recursion depth is the tree height, i.e. O(log n),
    // so there is no stack overflow risk.
    void insertAt(std::unique_ptr<Node>& node, const K& key, V&& value, bool overwrite,
                  bool& inserted) {
        if (!node) {
            node = std::make_unique<Node>(key, std::move(value));
            inserted = true;
            return;
        }
        if (less_(key, node->key)) {
            insertAt(node->left, key, std::move(value), overwrite, inserted);
        } else if (less_(node->key, key)) {
            insertAt(node->right, key, std::move(value), overwrite, inserted);
        } else {
            if (overwrite) node->value = std::move(value);
            return;  // key already present, shape unchanged
        }
        rebalance(node);
    }

    // In-order walk that skips everything smaller than *from (if given).
    // Returns false once fn asked to stop, so callers higher up stop too.
    template <typename Fn>
    bool walkFrom(const Node* node, const K* from, Fn& fn) const {
        if (!node) return true;
        bool nodeInRange = (from == nullptr) || !less_(node->key, *from);
        if (nodeInRange) {
            // Only worth going left when this node is in range; otherwise the
            // whole left subtree is smaller than `from` too.
            if (!walkFrom(node->left.get(), from, fn)) return false;
            if (!fn(node->key, node->value)) return false;
        }
        return walkFrom(node->right.get(), from, fn);
    }

    // Returns the subtree height, or -1 if any invariant is broken.
    // lo/hi are the open bounds every key in this subtree must respect.
    int check(const Node* node, const K* lo, const K* hi) const {
        if (!node) return 0;
        if (lo && !less_(*lo, node->key)) return -1;
        if (hi && !less_(node->key, *hi)) return -1;
        int lh = check(node->left.get(), lo, &node->key);
        int rh = check(node->right.get(), &node->key, hi);
        if (lh < 0 || rh < 0) return -1;
        if (lh - rh > 1 || rh - lh > 1) return -1;
        if (node->height != 1 + std::max(lh, rh)) return -1;
        return node->height;
    }

    std::unique_ptr<Node> root_;
    std::size_t size_ = 0;
    Compare less_{};
};

}  // namespace fp
