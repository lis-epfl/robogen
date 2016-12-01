#ifndef PARSE_HELPERS_H
#define PARSE_HELPERS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <stack>
#include <queue>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include "utils/RobogenUtils.h"
#include "brain/NeuralNetwork.h"

namespace robogen{

/**
 * Helper function for decoding a part line of a robot text file.
 * @return true if successful read
 */

bool isLineEmpty(std::string line);


bool robotTextFileReadPartLine(std::ifstream &file, unsigned int &indent,
		unsigned int &slot,
		char &type, std::string &id, unsigned int &orientation,
		std::vector<double> &params);

/**
 * Helper function for decoding a weight line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadWeightLine(std::ifstream &file, std::string &from,
		int &fromIoId, std::string &to, int &toIoId, double &value);

/**
 * Helper function to translate type string to code
 */
void parseTypeString(std::string typeString, unsigned int &type);
/**
 * Helper function for decoding an add-neuron line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadAddNeuronLine(std::ifstream &file, std::string &partId,
		unsigned int &type);

/**
 * Helper function for decoding a brain param line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadParamsLine(std::ifstream &file, std::string &node,
		int &ioId,  unsigned int &type, std::vector<double> &params);

}

#endif //PARSE_HELPERS_H