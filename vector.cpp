
// Created March 11, 2020
// Testing using a LILO data buffer

#include <iostream>
#include <sstream>
#include <vector>

int main () {

    char aC[] = { 'a', 'b', 'c', 'd', 'e', 'f', 'g'};
    int size = 3;

    std::vector<char> vCs;
    int index = 0;
    while (index < size) {
	vCs.push_back (aC[index++]);
    }

    for ( ;index <= sizeof(aC)/sizeof(char); ) {

	// Let's see what the buffer holds now
	int s=0;
	std::stringstream sst;
	while (s < size) {
	    sst << vCs[s++] << " ";
	}
	std::cout << sst.str() << std::endl;
	
	// Shuffle the elements one spo back
	for (std::vector<char>::iterator it=vCs.begin()+1; it< vCs.end(); it++) {
	    (*(it-1)) = (*it);
	}
	// Insert a new data item
	*(vCs.end()-1) = aC[index++];
    }

    return (0);
}
