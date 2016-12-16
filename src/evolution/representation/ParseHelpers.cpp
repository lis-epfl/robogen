#include"ParseHelpers.h"

namespace robogen{

/**
 * Helper function for decoding a part line of a robot text file.
 * @return true if successful read
 */

bool isLineEmpty(std::string line) {
	static const boost::regex spacex("^\\s*$");
	return boost::regex_match(line.c_str(), spacex);
}

bool robotTextFileReadPartLine(std::ifstream &file, unsigned int &indent,
		unsigned int &slot,
		char &type, std::string &id, unsigned int &orientation,
		std::vector<double> &params) {
	// match (0 or more tabs)(digit) (type) (id) (orientation) (parameters)
	static const boost::regex rx(
	 "^(\\t*)(\\d) ([A-Z]|(?:[A-Z][a-z]*)+) ([^\\s]+) (\\d)([ \\d\\.-]*)\\s*$"
			);
	boost::cmatch match;
	std::string line;
	RobogenUtils::safeGetline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)) {
		// match[0]:whole string, match[1]:tabs, match[2]:slot, match[3]:type,
		// match[4]:id, match[5]:orientation, match[6]:parameters
		indent = match[1].length();
		slot = std::atoi(match[2].first);

		if (match[3].str().length() == 1) {
			type = match[3].first[0];
			if( PART_TYPE_MAP.count(type) == 0) {
				std::cerr << "Invalid body part type: " << type
						<< std::endl;
				exitRobogen(EXIT_FAILURE);
			}
		} else if( INVERSE_PART_TYPE_MAP.count(match[3].str()) == 0) {
			std::cerr << "Invalid body part type: " << match[3].str()
					<< std::endl;
			exitRobogen(EXIT_FAILURE);
		} else {
			type = INVERSE_PART_TYPE_MAP.at(match[3].str());
		}
		id = std::string(match[4]);
		orientation = std::atoi(match[5].first);
		double param;
		std::stringstream ss(match[6]);
		params.clear();

		std::vector<double> rawParams;
		while (ss >> param) {
			rawParams.push_back(param);
		}
		if (rawParams.size()
				!= PART_TYPE_PARAM_COUNT_MAP.at(PART_TYPE_MAP.at(type))) {
			std::cerr << "Error reading body part from text file.\n"
					<< PART_TYPE_MAP.at(type) << " requires "
					<< PART_TYPE_PARAM_COUNT_MAP.at(PART_TYPE_MAP.at(type))
					<< " params, but " << rawParams.size()
					<< " were received\n";
			exitRobogen(EXIT_FAILURE);
			//return false;
		}

		// Do not normalize arity param
		unsigned int iParam0;
		if(PART_TYPE_IS_VARIABLE_ARITY_MAP.at(PART_TYPE_MAP.at(type)))
			iParam0 = 1;
		else
			iParam0 = 0;

		for (unsigned int i = 0; i < rawParams.size(); i++){
			std::pair<double, double> ranges = PART_TYPE_PARAM_RANGE_MAP.at(
					std::make_pair(PART_TYPE_MAP.at(type), i));
			double rawParamValue = rawParams[i];
			if (rawParamValue < ranges.first || rawParamValue > ranges.second) {
				std::cerr << "Error reading body part from text file.\n"
						<< PART_TYPE_MAP.at(type) << " requires param " << i
						<< " to be in [" << ranges.first << ", "
						<< ranges.second << "], but " << rawParamValue
						<< " was received\n";
				//return false;
				exitRobogen(EXIT_FAILURE);
			}
			if(i<iParam0)
			{
				params.push_back(rawParams[i]);
			}
			else
			{
				//add param in [0,1]
				params.push_back((fabs(ranges.first - ranges.second) < 1e-6) ? 0 :
						(rawParamValue - ranges.first)
								/ (ranges.second - ranges.first));
			}
		}

		return true;
	} else {
		// additional info if poor formatting, i.e. line not empty

		if (!isLineEmpty(line)) {
			std::cerr << "Error reading body part from text file. Received:\n"
					<< line << "\nbut expected format:\n"
					<< "<0 or more tabs><slot index digit> "
					<< "<part type character OR CamelCase string> "
							"<part id string> <orientation digit> "
					<< "<evt. parameters>"
					<< std::endl;
			exitRobogen(EXIT_FAILURE);
		}
		return false;
	}
}

/**
 * Helper function for decoding a weight line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadWeightLine(std::ifstream &file, std::string &from,
		int &fromIoId, std::string &to, int &toIoId, double &value) {

	static const boost::regex rx(
			"^([^\\s]+) (\\d+) ([^\\s]+) (\\d+) (-?\\d*\\.?\\d*)\\s*$");
	boost::cmatch match;
	std::string line;
	RobogenUtils::safeGetline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)) {
		// match[0]:whole string, match[1]:from, match[2]:from IO id,
		// match[3]:to, match[4]:to IO id, match[5]:value
		from.assign(match[1]);
		fromIoId = std::atoi(match[2].first);
		to.assign(match[3]);
		toIoId = std::atoi(match[4].first);
		value = std::atof(match[5].first);
		return true;
	} else {
		// additional info if poor formatting, i.e. line not empty
		if(!isLineEmpty(line)) {
			std::cerr << "Error reading weight from text file. Received:\n"
					<< line << "\nbut expected format:\n"
					<< "<source part id string> <source part io id> "
						"<destination part id string> <destination part io id> "
						"<weight>" << std::endl;
			exitRobogen(EXIT_FAILURE);
		}
		return false;
	}
}

/**
 * Helper function to translate type string to code
 */
void parseTypeString(std::string typeString, unsigned int &type) {
	boost::to_lower(typeString);
	if(typeString == "simple")
		type = NeuronRepresentation::SIMPLE;
	else if(typeString== "sigmoid" || typeString == "logistic")
		type = NeuronRepresentation::SIGMOID;
	else if(typeString == "ctrnn_sigmoid")
		type = NeuronRepresentation::CTRNN_SIGMOID;
	else if(typeString == "oscillator")
		type = NeuronRepresentation::OSCILLATOR;
	else {
		std::cerr << "Invalid neuron type: " << typeString << std::endl;
		exitRobogen(EXIT_FAILURE);
	}
}

/**
 * Helper function for decoding an add-neuron line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadAddNeuronLine(std::ifstream &file, std::string &partId,
		unsigned int &type) {

	static const boost::regex rx("^([^\\s]+) ([^\\s]+)\\s*$");
	boost::cmatch match;
	std::string line;
	RobogenUtils::safeGetline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)) {
		// match[0]:whole string, match[1]:partId match[2]:type string
		partId.assign(match[1]);
		std::string typeString = match[2];
		parseTypeString(typeString, type);
		return true;
	} else {
		// additional info if poor formatting, i.e. line not empty
		if(!isLineEmpty(line)) {
			std::cerr << "Error reading hidden neuron descriptor from text "
					<< "file. Received:\n"
					<< line << "\nbut expected format:\n"
					<< "<part id string> <type string>" << std::endl;
			exitRobogen(EXIT_FAILURE);
		}
		return false;
	}
}

/**
 * Helper function for decoding a brain param line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadParamsLine(std::ifstream &file, std::string &node,
		int &ioId,  unsigned int &type, std::vector<double> &params) {

	static const boost::regex
		generalRx("^([^\\s]+) (\\d+) ([^\\s]+)((?: -?\\d*\\.?\\d*)+)\\s*$");

	static const boost::regex biasRx("^([^\\s]+) (\\d+) (-?\\d*\\.?\\d*)\\s*$");
	boost::cmatch match;
	std::string line;
	RobogenUtils::safeGetline(file, line);
	if (boost::regex_match(line.c_str(), match, generalRx)) {
		node.assign(match[1]);
		ioId = std::atoi(match[2].first);
		std::string typeString = match[3];
		parseTypeString(typeString, type);
		std::string paramsString = match[4];
		boost::trim(paramsString);
		std::vector<std::string> strs;
		boost::split(strs, paramsString, boost::is_any_of(" "));
		for (unsigned int i=0; i<strs.size(); i++) {
			params.push_back(std::atof(strs[i].c_str()));
		}
		return true;
	} else if (boost::regex_match(line.c_str(), match, biasRx)) {
		for (unsigned int i=0; i < match.size(); i++) {
			std::cout << i << " " << match[i] << std::endl;
		}
		// match[0]:whole string, match[1]:node, match[2]:ioId, match[3]:value
		node.assign(match[1]);
		ioId = std::atoi(match[2].first);
		type = NeuronRepresentation::SIGMOID;
		params.push_back(std::atof(match[3].first));
		return true;
	} else {
		// additional info if poor formatting, i.e. line not empty
		if (!isLineEmpty(line) ) {
			std::cerr << "Error reading brain params from text file. "
					<< "Received:\n"
					<< line << "\nbut expected either format:\n"
					<< "<part id string> <part io id> <bias>\nor\n"
					<< "<part id string> <part io id> <neuron type> "
					<< "<param> <param> ..."
					<< std::endl;
			exitRobogen(EXIT_FAILURE);
		}
		return false;
	}
}

}