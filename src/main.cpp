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

#include <string>
#include <random>
#include <boost/lexical_cast.hpp>

#include "octree.h"

template class Octree<std::string>;

int main() {
    Octree<std::string> octree(10, 10, 10);

    std::uniform_real_distribution<double> unif(0.0, 10.0);
    std::default_random_engine re;

    for(unsigned int i=0; i<100; i++) {
        octree.add(new std::string(boost::lexical_cast<std::string>(i)), unif(re), unif(re), unif(re));
    }

    octree.print();

    auto nd = octree.find_node(6.15,1.20,6.15);
    std::cout << nd->get_cx() << "  " << nd->get_cy() << "  " << nd->get_cz() << std::endl;
    std::cout << nd->get_type() << std::endl;

    auto nd2 = nd->find_gteq_neighbor_face(OT_D_L);
    std::cout << nd2->get_cx() << "  " << nd2->get_cy() << "  " << nd2->get_cz() << std::endl;
    std::cout << nd2->get_type() << std::endl;

    std::cout << "Done" << std::endl;

    return 0;
}
