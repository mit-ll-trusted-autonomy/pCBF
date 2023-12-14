/**
* DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
*
* This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001.
* Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of
* the Under Secretary of Defense for Research and Engineering.
*
* © 2023 Massachusetts Institute of Technology.
*
* The software/firmware is provided to you on an As-Is basis
*
* Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014).
* Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above.
* Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
*/


/*
Generalizes how agents' states are stored.

This is kept in a separate class so that changing it for all CBFs
can be done by editing one file.
*/

#ifndef STATE_CONTAINER_HEADER
#define STATE_CONTAINER_HEADER

#include <array>
#include <iostream>

template <typename T, int N>
class VectorContainer {

   public:
    VectorContainer() { values = std::array<T, N>(); };
    VectorContainer(std::array<T, N> _array) { values = std::move(_array); }

    // Getter
    T operator[](int idx) const {
        // To remove bounds checking, use state[idx]
        return values.at(idx);
    }
    // Setter
    T &operator[](int idx) {
        return values.at(idx);
    }

	// TODO: Figure out how to make this private but accessible
	// by subclasses of CBF_Parent
    std::array<T, N> values;

    /*
    TODO: Change this to std::vector at some point?
    std::arrays can be more easily inlined I think.
    */
};


// Convenience functions for printing, etc.
template<typename T, int N>
std::ostream& operator << (std::ostream &os, const VectorContainer<T,N> &container) {
	os << "[";
	for (auto element : container.values) {
		os << element << ", ";
	}
	os << "]";
	return os;
}


#endif