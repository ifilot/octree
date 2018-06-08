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

/*
 * Octree implementation based on the following article:
 *    Neighbor Finding in Images Represented by Octrees
 *    Hanan Samet
 *    Computer vision, graphics, and image processing 46, 367-386 (1989)
 *    DOI: https://doi.org/10.1016/0734-189X(89)90038-8
 */

/**
 * @brief      Class for octree node.
 *
 * @tparam     T     object class
 */
template <class T>
class OctreeNode {

private:
    double cx;      //!< center position x
    double cy;      //!< center position y
    double cz;      //!< center position z

    double x;       //!< width of the cell
    double y;       //!< breadth of the cell
    double z;       //!< height of the cell

    std::vector<T*> objects;    //!< vector of pointers to objects
    std::vector<double> pos;    //!< positions of the objects

    OctreeNode* parent = nullptr;   //!< pointer to parent
    OctreeNode* children[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr}; //!< pointer to children

    unsigned int level;     //!< level of the node
    bool leaf = true;       //!< whether node is a leaf

public:
    /**
     * @brief      Constructs the object.
     *
     * @param      _parent  pointer to parent
     * @param[in]  _cx      center position x
     * @param[in]  _cy      center position y
     * @param[in]  _cz      center position z
     * @param[in]  _x       width of the cell
     * @param[in]  _y       breadth of the cell
     * @param[in]  _z       height of the cell
     * @param[in]  _level   The level
     */
    OctreeNode(OctreeNode* _parent,
               double _cx, double _cy, double _cz,
               double _x, double _y, double _z,
               unsigned int _level);

    /**
     * @brief      split the cell into 8 octants
     */
    void split();

    /**
     * @brief      add object to node
     *
     * @param      object  pointer to object
     * @param[in]  _px     object position x
     * @param[in]  _py     object position y
     * @param[in]  _pz     object position z
     */
    void add(T* object, double _px, double _py, double _pz);

    /**
     * @brief      get child node given octant position
     *
     * @param[in]  o     the octant position
     *
     * @return     pointer to child node
     */
    OctreeNode* get_child(unsigned int o) const {
        return this->children[o];
    }

    /**
     * @brief      get the parent node
     *
     * @return     pointer to parent node
     */
    OctreeNode* get_parent() const {
        return this->parent;
    }

    /**
     * @brief      get the octant type
     *
     * @return     octant type
     */
    unsigned int get_type() const;

    /**
     * @brief      find node given position
     *
     * @param[in]  _px   position x
     * @param[in]  _py   position y
     * @param[in]  _pz   position z
     *
     * @return     pointer to node
     */
    OctreeNode* find_node(double _px, double _py, double _pz);

    /**
     * @brief      find neighbors
     *
     * @return     vector holding pointers to neighbor nodes
     */
    std::vector<OctreeNode*> find_neighbors() const;

    /**
     * @brief      find face neighbor (equal or larger in size) in direction i
     *
     * @param[in]  i     direction i
     *
     * @return     pointer to face neighbor
     */
    OctreeNode* find_gteq_neighbor_face(unsigned int i) const;

    /**
     * @brief      find edge neighbor (equal or larger in size) in direction i
     *
     * @param[in]  i     direction i
     *
     * @return     pointer to edge neighbor
     */
    OctreeNode* find_gteq_neighbor_edge(unsigned int i) const;

    /**
     * @brief      find vertex neighbor (equal or larger in size) in direction i
     *
     * @param[in]  i     direction i
     *
     * @return     pointer to vertex neighbor
     */
    OctreeNode* find_gteq_neighbor_vertex(unsigned int i) const;

    /**
     * @brief      print the tree
     */
    void print();

    /**
     * @brief      Destroys the object.
     */
    ~OctreeNode();

    // getters

    /**
     * @brief      get node center x
     *
     * @return     node center x
     */
    inline double get_cx() const {
        return this->cx;
    }

    /**
     * @brief      get node center y
     *
     * @return     node center y
     */
    inline double get_cy() const {
        return this->cy;
    }

    /**
     * @brief      get node center z
     *
     * @return     node center z
     */
    inline double get_cz() const {
        return this->cz;
    }

    /**
     * @brief      determines if node is leaf
     *
     * @return     True if leaf, False otherwise.
     */
    inline bool is_leaf() const {
        return this->leaf;
    }

    /**
     * @brief      get the objects of the node
     *
     * @return     vector of pointer to objects
     */
    inline const std::vector<T*>& get_objects() const {
        return this->objects;
    }

private:

    /***********************************************************
     *
     * AUXILIARY FUNCTIONS
     *
     ***********************************************************/

    /**
     * @brief      check if octant is adjacent to direction
     *
     * @param[in]  i     direction
     * @param[in]  o     octant
     *
     * @return     true if adjacent, false otherwise
     */
    bool adj(unsigned int i, unsigned int o) const;

    /**
     * @brief      determines neighboring octant label in direction i
     *
     * @param[in]  i     direction
     * @param[in]  o     octant
     *
     * @return     octant label
     */
    unsigned int reflect(unsigned int i, unsigned int o) const;

    /**
     * @brief      determines label of face common to o and neighboring node in direction i
     *
     * @param[in]  i     direction
     * @param[in]  o     octant
     *
     * @return     face label
     */
    unsigned int common_face(unsigned int i, unsigned int o) const;

    /**
     * @brief      determines label of edge common to o and neighboring node in
     *             direction i
     *
     * @param[in]  i     direction
     * @param[in]  o     octant
     *
     * @return     edge label
     */
    unsigned int common_edge(unsigned int i, unsigned int o) const;
};

/**
 * @brief      Class for octree.
 *
 * @tparam     T     object type
 */
template <class T>
class Octree {

private:
    OctreeNode<T>* root = nullptr;  //!< pointer to root node

    double cx;                      //!< octree center x
    double cy;                      //!< octree center y
    double cz;                      //!< octree center z

    double x;                       //!< octree widht
    double y;                       //!< octree breadth
    double z;                       //!< octree height

public:
    /**
     * @brief      Octree constructor
     *
     * @param[in]  _px     width of principal cell
     * @param[in]  _py     breadth of principal cell
     * @param[in]  _pz     height of principal cell
     *
     */
    Octree(double _x, double _y, double _z);

    /**
     * @brief      add object to the tree
     *
     * @param      object  pointer to object
     * @param[in]  _px     x position
     * @param[in]  _py     y position
     * @param[in]  _pz     z position
     *
     */
    void add(T* object, double _px, double _py, double _pz);

    /**
     * @brief      print the tree to std::cout
     */
    inline void print() {
        this->root->print();
    }

    /**
     * @brief      find a node in the tree given position
     *
     * @param[in]  _px   x position
     * @param[in]  _py   y position
     * @param[in]  _pz   z position
     *
     * @return     pointer to node
     */
    inline OctreeNode<T>* find_node(double _px, double _py, double _pz) {
        return this->root->find_node(_px, _py, _pz);
    }
};

#include "octree.cpp"

#endif // _OCTREE_H
