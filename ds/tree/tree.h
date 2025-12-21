#pragma once

#include "../../ds/headers.hpp"

template <typename T>
class Tree {
private:
    struct TreeNode {
        T data;
        std::shared_ptr<TreeNode> left;
        std::shared_ptr<TreeNode> right;
        std::weak_ptr<TreeNode> parent;
        
        TreeNode(const T& value) : data(value), left(nullptr), right(nullptr) {}
    };
    
    using NodePtr = std::shared_ptr<TreeNode>;
    NodePtr root;
    size_t node_count;
    
    // Recursive insertion in BST
    NodePtr insert(NodePtr node, const T& value, NodePtr parent = nullptr) {
        if (!node) {
            auto new_node = std::make_shared<TreeNode>(value);
            new_node->parent = parent;
            node_count++;
            return new_node;
        }
        
        if (value < node->data) {
            node->left = insert(node->left, value, node);
        } else if (value > node->data) {
            node->right = insert(node->right, value, node);
        }
        // If value equals node->data, don't insert (no duplicates)
        
        return node;
    }
    
    // Recursive search
    NodePtr search(NodePtr node, const T& value) const {
        if (!node || node->data == value) {
            return node;
        }
        
        if (value < node->data) {
            return search(node->left, value);
        } else {
            return search(node->right, value);
        }
    }
    
    // Find minimum value node
    NodePtr find_min(NodePtr node) const {
        if (!node) return nullptr;
        
        while (node->left) {
            node = node->left;
        }
        return node;
    }
    
    // Recursive deletion
    NodePtr remove(NodePtr node, const T& value) {
        if (!node) return node;
        
        if (value < node->data) {
            node->left = remove(node->left, value);
        } else if (value > node->data) {
            node->right = remove(node->right, value);
        } else {
            // Node to be deleted found
            node_count--;
            
            // Case 1: Node has no children
            if (!node->left && !node->right) {
                return nullptr;
            }
            
            // Case 2: Node has one child
            if (!node->left) {
                if (node->right) {
                    node->right->parent = node->parent.lock();
                }
                return node->right;
            }
            if (!node->right) {
                if (node->left) {
                    node->left->parent = node->parent.lock();
                }
                return node->left;
            }
            
            // Case 3: Node has two children
            NodePtr successor = find_min(node->right);
            node->data = successor->data;
            node->right = remove(node->right, successor->data);
            node_count++; // Compensate for the decrement that will happen in recursive call
        }
        
        return node;
    }
    
    // Calculate height
    int height(NodePtr node) const {
        if (!node) return -1;
        
        int left_height = height(node->left);
        int right_height = height(node->right);
        
        return 1 + std::max(left_height, right_height);
    }
    
    // Check if tree is balanced
    bool is_balanced(NodePtr node, int& height) const {
        if (!node) {
            height = -1;
            return true;
        }
        
        int left_height, right_height;
        bool left_balanced = is_balanced(node->left, left_height);
        bool right_balanced = is_balanced(node->right, right_height);
        
        height = 1 + std::max(left_height, right_height);
        
        return left_balanced && right_balanced && 
               std::abs(left_height - right_height) <= 1;
    }

public:
    Tree() : root(nullptr), node_count(0) {}
    
    // Insert a value into the binary search tree
    void insert(const T& value) {
        root = insert(root, value);
    }
    
    // Search for a value in the tree
    bool contains(const T& value) const {
        return search(root, value) != nullptr;
    }
    
    // Remove a value from the tree
    void remove(const T& value) {
        root = remove(root, value);
    }
    
    // Get inorder traversal (sorted order for BST)
    void inorder(NodePtr node, std::vector<T>& result) const {
        if (!node) return;
        
        inorder(node->left, result);
        result.push_back(node->data);
        inorder(node->right, result);
    }
    
    std::vector<T> inorder() const {
        std::vector<T> result;
        inorder(root, result);
        return result;
    }
    
    // Get preorder traversal
    void preorder(NodePtr node, std::vector<T>& result) const {
        if (!node) return;
        
        result.push_back(node->data);
        preorder(node->left, result);
        preorder(node->right, result);
    }
    
    std::vector<T> preorder() const {
        std::vector<T> result;
        preorder(root, result);
        return result;
    }
    
    // Get postorder traversal
    void postorder(NodePtr node, std::vector<T>& result) const {
        if (!node) return;
        
        postorder(node->left, result);
        postorder(node->right, result);
        result.push_back(node->data);
    }
    
    std::vector<T> postorder() const {
        std::vector<T> result;
        postorder(root, result);
        return result;
    }
    
    // Breadth-First Search traversal (level order)
    std::vector<T> bfs() const {
        if (!root) return {};
        
        std::vector<T> result;
        std::queue<NodePtr> q;
        
        q.push(root);
        
        while (!q.empty()) {
            NodePtr current = q.front();
            q.pop();
            result.push_back(current->data);
            
            if (current->left) q.push(current->left);
            if (current->right) q.push(current->right);
        }
        
        return result;
    }
    
    // Iterative Depth-First Search traversal (preorder)
    std::vector<T> dfs_iterative() const {
        if (!root) return {};
        
        std::vector<T> result;
        std::stack<NodePtr> s;
        
        s.push(root);
        
        while (!s.empty()) {
            NodePtr current = s.top();
            s.pop();
            result.push_back(current->data);
            
            // Push right first so left is processed first
            if (current->right) s.push(current->right);
            if (current->left) s.push(current->left);
        }
        
        return result;
    }
    
    // Find minimum value in the tree
    T find_min() const {
        if (!root) {
            throw std::runtime_error("Tree is empty");
        }
        
        NodePtr min_node = find_min(root);
        return min_node->data;
    }
    
    // Find maximum value in the tree
    T find_max() const {
        if (!root) {
            throw std::runtime_error("Tree is empty");
        }
        
        NodePtr current = root;
        while (current->right) {
            current = current->right;
        }
        return current->data;
    }
    
    // Get height of the tree
    int height() const {
        return height(root);
    }
    
    // Check if tree is balanced
    bool is_balanced() const {
        int height_val;
        return is_balanced(root, height_val);
    }
    
    // Check if tree is a valid BST
    bool is_valid_bst() const {
        std::vector<T> inorder_result = inorder();
        
        for (size_t i = 1; i < inorder_result.size(); ++i) {
            if (inorder_result[i] <= inorder_result[i-1]) {
                return false;
            }
        }
        return true;
    }
    
    // Get all leaves of the tree
    std::vector<T> get_leaves() const {
        std::vector<T> leaves;
        
        if (!root) return leaves;
        
        std::queue<NodePtr> q;
        q.push(root);
        
        while (!q.empty()) {
            NodePtr current = q.front();
            q.pop();
            
            // Check if it's a leaf node
            if (!current->left && !current->right) {
                leaves.push_back(current->data);
            }
            
            if (current->left) {
                q.push(current->left);
            }
            if (current->right) {
                q.push(current->right);
            }
        }
        
        return leaves;
    }
    
    // Get nodes at a specific level (0-indexed)
    std::vector<T> get_level(int level) const {
        std::vector<T> result;
        
        if (!root || level < 0) return result;
        
        std::queue<std::pair<NodePtr, int>> q;
        q.push({root, 0});
        
        while (!q.empty()) {
            auto [node, current_level] = q.front();
            q.pop();
            
            if (current_level == level) {
                result.push_back(node->data);
            } else if (current_level < level) {
                if (node->left) {
                    q.push({node->left, current_level + 1});
                }
                if (node->right) {
                    q.push({node->right, current_level + 1});
                }
            }
        }
        
        return result;
    }
    
    // Clear the entire tree
    void clear() {
        root = nullptr;
        node_count = 0;
    }
    
    // Tree properties
    size_t size() const { return node_count; }
    bool is_empty() const { return root == nullptr; }
    
    // Display the tree (for debugging) - simple level-order format
    void print_tree() const {
        if (!root) {
            std::cout << "Tree is empty" << std::endl;
            return;
        }
        
        std::queue<NodePtr> q;
        q.push(root);
        int level = 0;
        
        while (!q.empty()) {
            int level_size = q.size();
            std::cout << "Level " << level << ": ";
            
            for (int i = 0; i < level_size; ++i) {
                NodePtr current = q.front();
                q.pop();
                
                if (current) {
                    std::cout << current->data << " ";
                    q.push(current->left);
                    q.push(current->right);
                } else {
                    std::cout << "null ";
                }
            }
            
            std::cout << std::endl;
            level++;
            
            // Stop if all remaining nodes are null
            bool has_non_null = false;
            auto temp_q = q;
            while (!temp_q.empty()) {
                if (temp_q.front() != nullptr) {
                    has_non_null = true;
                    break;
                }
                temp_q.pop();
            }
            if (!has_non_null) break;
        }
    }
};
