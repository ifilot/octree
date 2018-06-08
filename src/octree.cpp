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

#include "octree.h"

#ifndef _OCTREE_IMPL
#define _OCTREE_IMPL

template <class T>
Octree<T>::Octree(double _x, double _y, double _z) :
    cx(_x/2),
    cy(_y/2),
    cz(_z/2),
    x(_x),
    y(_y),
    z(_z) {

    this->root = new OctreeNode<T>(nullptr,
                                   this->cx, this->cy, this->cz,
                                   this->x, this->y, this->z,
                                   0);
}

template <class T>
void Octree<T>::add(T* object, double _px, double _py, double _pz) {
    this->root->find_node(_px, _py, _pz)->add(object, _px, _py, _pz);
}

template <class T>
OctreeNode<T>::OctreeNode(OctreeNode* _parent,
                          double _cx, double _cy, double _cz,
                          double _x, double _y, double _z,
                          unsigned int _level) :
    parent(_parent),
    cx(_cx),
    cy(_cy),
    cz(_cz),
    x(_x),
    y(_y),
    z(_z),
    level(_level) {

    this->pos.reserve(3 * 16);
    this->objects.reserve(16);
}

template <class T>
void OctreeNode<T>::split() {
    std::cout << "Split at level: " << this->level << std::endl;
    if(this->children[0] != nullptr) {
        return;
    }

    // set node to leaf node
    this->leaf = false;

    const double nx = this->x / 2.0;
    const double ny = this->y / 2.0;
    const double nz = this->z / 2.0;
    const unsigned int ll = this->level + 1;

    this->children[OT_LDB] = new OctreeNode(this, this->cx - nx / 2.0, this->cy - ny / 2.0, this->cz - nz / 2.0, nx, ny, nz, ll);
    this->children[OT_LDF] = new OctreeNode(this, this->cx - nx / 2.0, this->cy + ny / 2.0, this->cz - nz / 2.0, nx, ny, nz, ll);
    this->children[OT_LUB] = new OctreeNode(this, this->cx - nx / 2.0, this->cy - ny / 2.0, this->cz + nz / 2.0, nx, ny, nz, ll);
    this->children[OT_LUF] = new OctreeNode(this, this->cx - nx / 2.0, this->cy + ny / 2.0, this->cz + nz / 2.0, nx, ny, nz, ll);
    this->children[OT_RDB] = new OctreeNode(this, this->cx + nx / 2.0, this->cy - ny / 2.0, this->cz - nz / 2.0, nx, ny, nz, ll);
    this->children[OT_RDF] = new OctreeNode(this, this->cx + nx / 2.0, this->cy + ny / 2.0, this->cz - nz / 2.0, nx, ny, nz, ll);
    this->children[OT_RUB] = new OctreeNode(this, this->cx + nx / 2.0, this->cy - ny / 2.0, this->cz + nz / 2.0, nx, ny, nz, ll);
    this->children[OT_RUF] = new OctreeNode(this, this->cx + nx / 2.0, this->cy + ny / 2.0, this->cz + nz / 2.0, nx, ny, nz, ll);

    // migrate objects
    for(unsigned int i=0; i<this->objects.size(); i++) {
        this->find_node(this->pos[i*3], this->pos[i*3+1], this->pos[i*3+2])->add(this->objects[i], this->pos[i*3], this->pos[i*3+1], this->pos[i*3+2]);
    }

    this->objects.clear();
    this->pos.clear();
}

template <class T>
void OctreeNode<T>::add(T* object, double _px, double _py, double _pz) {
    if(this->leaf) {
        this->objects.push_back(object);
        this->pos.push_back(_px);
        this->pos.push_back(_py);
        this->pos.push_back(_pz);

        if(this->objects.size() >= 16) {
            this->split();
        }
    }
}

template <class T>
void OctreeNode<T>::print() {
    for(unsigned int j=0; j<this->level; j++) {
        std::cout << "\t";
    }
    std::cout << "(" << this->level << ") " << this->cx << "  " << this->cy << "  " << this->cz << "  " << this->get_type() << std::endl;

    if(!this->leaf) {
        for(unsigned int i=0; i<8; i++) {
            this->children[i]->print();
        }
    }
}

template <class T>
unsigned int OctreeNode<T>::get_type() const {
    if(this->get_parent() == nullptr) {
        return OT_ROOT;
    }

    for(unsigned int i=0; i<8; i++) {
        if(this->get_parent()->get_child(i) == this) {
            return i;
        }
    }
}

template <class T>
OctreeNode<T>* OctreeNode<T>::find_node(double _px, double _py, double _pz) {
    if(this->leaf) {
        return this;
    }

    if(_px < this->cx) { // L
        if(_pz < this->cz) { // D
            if(_py < this->cy) { // B
                return this->children[OT_LDB]->find_node(_px, _py, _pz);
            } else { // F
                return this->children[OT_LDF]->find_node(_px, _py, _pz);
            }
        } else { // U
            if(_py < this->cy) { // B
                return this->children[OT_LUB]->find_node(_px, _py, _pz);
            } else { // F
                return this->children[OT_LUF]->find_node(_px, _py, _pz);
            }
        }
    } else { // R
        if(_pz < this->cz) { // D
            if(_py < this->cy) { // B
                return this->children[OT_RDB]->find_node(_px, _py, _pz);
            } else { // F
                return this->children[OT_RDF]->find_node(_px, _py, _pz);
            }
        } else { // U
            if(_py < this->cy) { // B
                return this->children[OT_RUB]->find_node(_px, _py, _pz);
            } else { // F
                return this->children[OT_RUF]->find_node(_px, _py, _pz);
            }
        }
    }
}

template <class T>
OctreeNode<T>* OctreeNode<T>::find_eq_neighbor_face(unsigned int i) const {
    const unsigned int type = this->get_type();
    if(this->adj(i, type)) {
        return this->get_parent()->find_eq_neighbor_face(i)->get_child(this->reflect(i, type));
    } else {
        return this->get_parent()->get_child(this->reflect(i, type));
    }
}

template <class T>
OctreeNode<T>* OctreeNode<T>::find_gteq_neighbor_face(unsigned int i) const {
    OctreeNode<T>* q;
    const unsigned int type = this->get_type();

    if(this->parent != nullptr && this->adj(i, type)) {
        q = this->parent->find_gteq_neighbor_face(i);
    } else {
        q = this->parent;
    }

    if(q != nullptr && !q->is_leaf()) {
        return q->get_child(this->reflect(i, type));
    } else {
        return q;
    }
}

template <class T>
OctreeNode<T>* OctreeNode<T>::find_gteq_neighbor_edge(unsigned int i) const {
    OctreeNode<T>* q;
    const unsigned int type = this->get_type();

    if(this->parent == nullptr) {
        q = nullptr;
    } else if(this->adj(i, type)) {
        q = this->parent->find_gteq_neighbor_edge(i);
    } else if(this->common_face(i, type) != OT_D_UNKNOWN) {
        q = this->parent->find_gteq_neighbor_face(this->common_face(i, type));
    } else {
        q = parent;
    }

    if(q != nullptr && !q->is_leaf()) {
        return q->get_child(this->reflect(i, type));
    } else {
        return q;
    }
}

template <class T>
OctreeNode<T>::~OctreeNode() {
    if(!this->leaf) {
        delete[] this->children;
    }
}

template <class T>
bool OctreeNode<T>::adj(unsigned int i, unsigned int o) const {
    static const bool table[18][8] = {
        {1,1,1,1,0,0,0,0},
        {0,0,0,0,1,1,1,1},
        {1,1,0,0,1,1,0,0},
        {0,0,1,1,0,0,1,1},
        {1,0,1,0,1,0,1,0},
        {0,1,0,1,0,1,0,1},
        {1,1,0,0,0,0,0,0},
        {0,0,1,1,0,0,0,0},
        {1,0,1,0,0,0,0,0},
        {0,1,0,1,0,0,0,0},
        {0,0,0,0,1,1,0,0},
        {0,0,0,0,0,0,1,1},
        {0,0,0,0,1,0,1,0},
        {0,0,0,0,0,1,0,1},
        {1,0,0,0,1,0,0,0},
        {0,1,0,0,0,1,0,0},
        {0,0,1,0,0,0,1,0},
        {0,0,0,1,0,0,0,1}
    };

    return table[i][o];
}

template <class T>
unsigned int OctreeNode<T>::reflect(unsigned int i, unsigned int o) const {
    static const unsigned int table[20][8] = {
        {OT_RDB, OT_RDF, OT_RUB, OT_RUF, OT_LDB, OT_LDF, OT_LUB, OT_LUF}, // L
        {OT_RDB, OT_RDF, OT_RUB, OT_RUF, OT_LDB, OT_LDF, OT_LUB, OT_LUF}, // R
        {OT_LUB, OT_LUF, OT_LDB, OT_LDF, OT_RUB, OT_RUF, OT_RDB, OT_RDF}, // D
        {OT_LUB, OT_LUF, OT_LDB, OT_LDF, OT_RUB, OT_RUF, OT_RDB, OT_RDF}, // U
        {OT_LDF, OT_LDB, OT_LUF, OT_LUB, OT_RDF, OT_RDB, OT_RUF, OT_RUB}, // B
        {OT_LDF, OT_LDB, OT_LUF, OT_LUB, OT_RDF, OT_RDB, OT_RUF, OT_RUB}, // F
        {OT_RUB, OT_RUF, OT_RDB, OT_RDF, OT_LUB, OT_LUF, OT_LDB, OT_LDF}, // LD
        {OT_RUB, OT_RUF, OT_RDB, OT_RDF, OT_LUB, OT_LUF, OT_LDB, OT_LDF}, // LU
        {OT_RDF, OT_RDB, OT_RUF, OT_RUB, OT_LDF, OT_LDB, OT_LUF, OT_LUB}, // LB
        {OT_RDF, OT_RDB, OT_RUF, OT_RUB, OT_LDF, OT_LDB, OT_LUF, OT_LUB}, // LF
        {OT_RUB, OT_RUF, OT_RDB, OT_RDF, OT_LUB, OT_LUF, OT_LDB, OT_LDF}, // RD
        {OT_RUB, OT_RUF, OT_RDB, OT_RDF, OT_LUB, OT_LUF, OT_LDB, OT_LDF}, // RU
        {OT_RDF, OT_RDB, OT_RUF, OT_RUB, OT_LDF, OT_LDB, OT_LUF, OT_LUB}, // RB
        {OT_RDF, OT_RDB, OT_RUF, OT_RUB, OT_LDF, OT_LDB, OT_LUF, OT_LUB}, // RF
        {OT_LUF, OT_LUB, OT_LDF, OT_LDB, OT_RUF, OT_RUB, OT_RDF, OT_RDB}, // DB
        {OT_LUF, OT_LUB, OT_LDF, OT_LDB, OT_RUF, OT_RUB, OT_RDF, OT_RDB}, // DF
        {OT_LUF, OT_LUB, OT_LDF, OT_LDB, OT_RUF, OT_RUB, OT_RDF, OT_RDB}, // UB
        {OT_LUF, OT_LUB, OT_LDF, OT_LDB, OT_RUF, OT_RUB, OT_RDF, OT_RDB}  // UF
    };

    return table[i][o];
}

template <class T>
unsigned int OctreeNode<T>::common_face(unsigned int i, unsigned int o) const {
    static const unsigned int table[12][8] = {
        {OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_L, OT_D_L, OT_D_D, OT_D_D, OT_D_UNKNOWN, OT_D_UNKNOWN}, // LD
        {OT_D_L, OT_D_L, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_U, OT_D_U}, // LU
        {OT_D_UNKNOWN, OT_D_L, OT_D_UNKNOWN, OT_D_L, OT_D_B, OT_D_UNKNOWN, OT_D_B, OT_D_UNKNOWN}, // LB
        {OT_D_L, OT_D_UNKNOWN, OT_D_L, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_F, OT_D_UNKNOWN, OT_D_F}, // LF
        {OT_D_D, OT_D_D, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_R, OT_D_R}, // RD
        {OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_U, OT_D_U, OT_D_R, OT_D_R, OT_D_UNKNOWN, OT_D_UNKNOWN}, // RU
        {OT_D_B, OT_D_UNKNOWN, OT_D_B, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_R, OT_D_UNKNOWN, OT_D_R}, // RB
        {OT_D_UNKNOWN, OT_D_F, OT_D_UNKNOWN, OT_D_F, OT_D_R, OT_D_UNKNOWN, OT_D_R, OT_D_UNKNOWN}, // RF
        {OT_D_UNKNOWN, OT_D_D, OT_D_B, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_D, OT_D_B, OT_D_UNKNOWN}, // DB
        {OT_D_D, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_F, OT_D_D, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_F}, // DF
        {OT_D_B, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_U, OT_D_B, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_U}, // UB
        {OT_D_UNKNOWN, OT_D_F, OT_D_U, OT_D_UNKNOWN, OT_D_UNKNOWN, OT_D_F, OT_D_U, OT_D_UNKNOWN}  // UF
    };

    return table[i-6][o];
}

#endif // _OCTREE_IMPL
