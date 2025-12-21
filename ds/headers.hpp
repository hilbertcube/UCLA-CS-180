#ifndef GENERAL_HEADERS
#define GENERAL_HEADERS

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <algorithm>
#include <stack>
#include <queue>
#include <deque>
#include <list>
#include <utility>
#include <type_traits>
#include <functional>
#include <iomanip>
#include <cmath>
#include <numeric>
#include <bitset>
#include <tuple>
#include <climits>
#include <cassert>
#include <sstream>
#include <cctype>
#include <optional>
#include <stdint.h>

using namespace std;

// ——— Sequence containers ———
template<typename T>
ostream& operator<<(ostream& os, const vector<T>& v) {
    os << '[';
    bool first = true;
    for (auto const& e : v) {
        if (!first) os << ", ";
        first = false;
        os << e;
    }
    os << ']';
    return os;
}

template<typename A, typename B>
ostream& operator<<(ostream& os, const pair<A, B>& p) {
    os << '(' << p.first << ", " << p.second << ')';
    return os;
}

template<typename T>
ostream& operator<<(ostream& os, const deque<T>& d) {
    os << '[';
    bool first = true;
    for (auto const& e : d) {
        if (!first) os << ", ";
        first = false;
        os << e;
    }
    os << ']';
    return os;
}

template<typename T>
ostream& operator<<(ostream& os, const list<T>& l) {
    os << '[';
    bool first = true;
    for (auto const& e : l) {
        if (!first) os << ", ";
        first = false;
        os << e;
    }
    os << ']';
    return os;
}

// ——— Set-like containers ———

template<typename T>
ostream& operator<<(ostream& os, const set<T>& s) {
    os << '{';
    bool first = true;
    for (auto const& e : s) {
        if (!first) os << ", ";
        first = false;
        os << e;
    }
    os << '}';
    return os;
}

template<typename T>
ostream& operator<<(ostream& os, const multiset<T>& s) {
    os << '{';
    bool first = true;
    for (auto const& e : s) {
        if (!first) os << ", ";
        first = false;
        os << e;
    }
    os << '}';
    return os;
}

template<typename T>
ostream& operator<<(ostream& os, const unordered_set<T>& s) {
    os << '{';
    bool first = true;
    for (auto const& e : s) {
        if (!first) os << ", ";
        first = false;
        os << e;
    }
    os << '}';
    return os;
}

template<typename T>
ostream& operator<<(ostream& os, const unordered_multiset<T>& s) {
    os << '{';
    bool first = true;
    for (auto const& e : s) {
        if (!first) os << ", ";
        first = false;
        os << e;
    }
    os << '}';
    return os;
}

// ——— Map-like containers ———

template<typename K, typename V>
ostream& operator<<(ostream& os, const map<K,V>& m) {
    os << '{';
    bool first = true;
    for (auto const& kv : m) {
        if (!first) os << ", ";
        first = false;
        os << kv.first << ": " << kv.second;
    }
    os << '}';
    return os;
}

template<typename K, typename V>
ostream& operator<<(ostream& os, const multimap<K,V>& m) {
    os << '{';
    bool first = true;
    for (auto const& kv : m) {
        if (!first) os << ", ";
        first = false;
        os << kv.first << ": " << kv.second;
    }
    os << '}';
    return os;
}

template<typename K, typename V>
ostream& operator<<(ostream& os, const unordered_map<K,V>& m) {
    os << '{';
    bool first = true;
    for (auto const& kv : m) {
        if (!first) os << ", ";
        first = false;
        os << kv.first << ": " << kv.second;
    }
    os << '}';
    return os;
}

template<typename K, typename V>
ostream& operator<<(ostream& os, const unordered_multimap<K,V>& m) {
    os << '{';
    bool first = true;
    for (auto const& kv : m) {
        if (!first) os << ", ";
        first = false;
        os << kv.first << ": " << kv.second;
    }
    os << '}';
    return os;
}

// ——— Adapter containers ———

template<typename T, typename Container>
ostream& operator<<(ostream& os, stack<T,Container> s) {
    os << "stack[";
    bool first = true;
    while (!s.empty()) {
        if (!first) os << ", ";
        first = false;
        os << s.top();
        s.pop();
    }
    os << ']';
    return os;
}

template<typename T, typename Container>
ostream& operator<<(ostream& os, queue<T,Container> q) {
    os << "queue[";
    bool first = true;
    while (!q.empty()) {
        if (!first) os << ", ";
        first = false;
        os << q.front();
        q.pop();
    }
    os << ']';
    return os;
}

template<typename T, typename Container, typename Compare>
ostream& operator<<(ostream& os, priority_queue<T,Container,Compare> pq) {
    os << "priority_queue[";
    bool first = true;
    while (!pq.empty()) {
        if (!first) os << ", ";
        first = false;
        os << pq.top();
        pq.pop();
    }
    os << ']';
    return os;
}

void print_bool(bool input) {
    const string GREEN = "\033[32m";  // ANSI code for green
    const string RED   = "\033[31m";  // ANSI code for red
    const string RESET = "\033[0m";   // Reset color

    if (input)
        cout << GREEN << "True" << RESET << endl;
    else
        cout << RED << "False" << RESET << endl;
}

#endif