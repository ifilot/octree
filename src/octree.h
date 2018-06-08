 /***********************************************************************************
 #   This file is part of octree.                                                   #
 #                                                                                  #
 #   MIT License                                                                    #
 #                                                                                  #
 #   Copyright (c) 2018 Ivo Filot <ivo@ivofilot.nl>                                 #
 #                                                                                  #
 #   Permission is hereby granted, free of charge, to any person obtaining a copy   #
 #   of this software and associated documentation files (the "Software"), to deal  #
 #   in the Software without restriction, including without limitation the rights   #
 #   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      #
 #   copies of the Software, and to permit persons to whom the Software is          #
 #   furnished to do so, subject to the following conditions:                       #
 #                                                                                  #
 #   The above copyright notice and this permission notice shall be included in all #
 #   copies or substantial portions of the Software.                                #
 #                                                                                  #
 #   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     #
 #   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       #
 #   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    #
 #   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         #
 #   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  #
 #   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  #
 #   SOFTWARE.                                                                      #
 #                                                                                  #
 #***********************************************************************************/

#ifndef _OCTREE_H
#define _OCTREE_H

#include <vector>
#include <iostream>
#include <unordered_set>

#include "octreetypes.h"

template <class T>
class OctreeNode {

private:
    double cx;
    double cy;
    double cz;

    double x;
    double y;
    double z;

    std::vector<T*> objects;
    std::vector<double> pos;

    OctreeNode* parent = nullptr;
    OctreeNode* children[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

    unsigned int level;
    bool leaf = true;

public:
    OctreeNode(OctreeNode* _parent,
               double _cx, double _cy, double _cz,
               double _x, double _y, double _z,
               unsigned int _level);

    void split();

    void add(T* object, double _px, double _py, double _pz);

    void print();

    OctreeNode* get_child(unsigned int o) const {
        return this->children[o];
    }

    OctreeNode* get_parent() const {
        return this->parent;
    }

    unsigned int get_type() const;

    OctreeNode* find_node(double _px, double _py, double _pz);

    std::vector<OctreeNode*> find_neighbors() const;

    OctreeNode* find_gteq_neighbor_face(unsigned int i) const;

    OctreeNode* find_gteq_neighbor_edge(unsigned int i) const;

    OctreeNode* find_gteq_neighbor_vertex(unsigned int i) const;

    ~OctreeNode();

    // getters
    inline double get_cx() const {
        return this->cx;
    }

    inline double get_cy() const {
        return this->cy;
    }

    inline double get_cz() const {
        return this->cz;
    }

    inline bool is_leaf() const {
        return this->leaf;
    }

    inline const std::vector<T*>& get_objects() const {
        return this->objects;
    }

private:
    bool adj(unsigned int i, unsigned int o) const;

    unsigned int reflect(unsigned int i, unsigned int o) const;

    unsigned int common_face(unsigned int i, unsigned int o) const;

    unsigned int common_edge(unsigned int i, unsigned int o) const;
};

template <class T>
class Octree {

private:
    OctreeNode<T>* root = nullptr;

    double cx;
    double cy;
    double cz;

    double x;
    double y;
    double z;

public:
    Octree(double _x, double _y, double _z);

    void add(T* object, double _px, double _py, double _pz);

    inline void print() {
        this->root->print();
    }

    inline OctreeNode<T>* find_node(double _px, double _py, double _pz) {
        return this->root->find_node(_px, _py, _pz);
    }
};

#include "octree.cpp"

#endif // _OCTREE_H
