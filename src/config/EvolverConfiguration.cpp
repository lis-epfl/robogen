/*
 * @(#) EvolverConfiguration.cpp   1.0   Sep 2, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014 Titus Cieslewski
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */

#include <iostream>
#include <sstream>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

#include "config/EvolverConfiguration.h"
#include "PartList.h"

namespace robogen {

const std::string
EvolverConfiguration::BodyMutationOperatorsProbabilityCodes[] = {
			"pSubtreeRemove", "pSubtreeDuplicate", "pSubtreeSwap",
			"pNodeInsert", "pNodeRemove", "pParameterModify",
			"pOrientationChange", "pSensorSwap", "pLinkChange", "pActivePassive"
	};

// helper function to parse bounds options

bool parseBounds(std::string value, double &min, double &max) {
	boost::cmatch match;
	static const boost::regex boundsRegex(
					"^(-?\\d*[\\d\\.]\\d*):(-?\\d*[\\d\\.]\\d*)$");
	// match[0]:whole string, match[1]:min, match[2]:max
	if (!boost::regex_match(value.c_str(), match, boundsRegex)){
		std::cout << "Supplied bounds argument \"" <<
				value <<
				"\" does not match pattern <min>:<max>" << std::endl;
		return false;
	}
	min = std::atof(match[1].first);
	max = std::atof(match[2].first);
	if (min > max) {
		std::cout << "supplied min " << min << " is greater than supplied max "
				<< max << std::endl;
		return false;
	}

	return true;
}




/**
 * Parses options from given conf file.
 */
bool EvolverConfiguration::init(std::string configFileName) {

	this->confFileName = configFileName;

	// cleanse operator probabilities before reading in (not specified = 0)
	memset(bodyOperatorProbability, 0, sizeof(bodyOperatorProbability));
	maxBodyMutationAttempts = 100; //seems like a reasonable default
	maxBodyParts = 100000; //some unreasonably large value if max not set
	// boost-parse options
	boost::program_options::options_description desc(
			"Allowed options for Evolution Config File");

	std::vector<std::string> allowedBodyPartTypeStrings;

	//defaults - TODO add more
	useBrainSeed = false;

	minBrainPhaseOffset = -1;
	maxBrainPhaseOffset = 1;

	minBrainAmplitude = 0;
	maxBrainAmplitude = 1;

	// DON'T AUTO-INDENT THE FOLLOWING ON ECLIPSE:
	desc.add_options()
		("referenceRobotFile",
				boost::program_options::value<std::string>(&referenceRobotFile),
				"Path to the reference robot file")
		("simulatorConfFile",
				boost::program_options::value<std::string>(&simulatorConfFile)
				->required(), "Path to simulator configuration file")
		("mu",
				boost::program_options::value<unsigned int>(&mu)
				->required(), "Number of parents")
		("lambda",
				boost::program_options::value<unsigned int>(&lambda)
				->required(), "Number of offspring")
		("numGenerations",
				boost::program_options::value<unsigned int>(&numGenerations)
				->required(), "Amount of generations to be evaluated")
		("selection",
				boost::program_options::value<std::string>()->required(),
				"Type of selection strategy: deterministic-tournament")
		("tournamentSize",
				boost::program_options::value<unsigned int>(&tournamentSize),
				"Amount of participants in deterministic Tournament")
		("replacement",
				boost::program_options::value<std::string>()->required(),
				"Type of replacement strategy: comma or plus")
		("evolutionMode",
				boost::program_options::value<std::string>()->required(),
				"Mode of evolution: brain or full")
		("useBrainSeed",
				boost::program_options::value<bool>(&useBrainSeed),
				"Set true to continue evolving from provided brain instead of "\
				"re-initialing.")
		("pBrainMutate", boost::program_options::value<double>
				(&pBrainMutate),"Probability of mutation for any single brain "\
				"parameter")
		("brainSigma", boost::program_options::value<std::string>(),
				"Sigma of all brain parameter mutations")
		("brainBounds", boost::program_options::value<std::string>(),
				"Bounds of brain weights and biases. Format: min:max")
		("weightSigma", boost::program_options::value<std::string>(),
				"Sigma of brain weight mutation")
		("weightBounds", boost::program_options::value<std::string>(),
				"Bounds of brain weights. Format: min:max")
		("biasSigma", boost::program_options::value<std::string>(),
				"Sigma of brain bias mutation")
		("biasBounds", boost::program_options::value<std::string>(),
				"Bounds of brain biases. Format: min:max")
		("tauSigma", boost::program_options::value<double>(&brainTauSigma),
				"Sigma of brain tau mutation")
		("tauBounds", boost::program_options::value<std::string>(),
				"Bounds of brain taus. Format: min:max")
		("periodSigma", boost::program_options::value<double>(&brainPeriodSigma),
				"Sigma of brain period mutation (defined in terms of seconds)")
		("periodBounds", boost::program_options::value<std::string>(),
				"Bounds of brain period (defined in terms of seconds). Format: min:max")
		("phaseOffsetSigma", boost::program_options::value<double>(&brainPhaseOffsetSigma),
				"Sigma of brain phase offset mutation (defined in terms of # periods).")
		("phaseOffsetBounds", boost::program_options::value<std::string>(),
				"Bounds of brain phase offset (defined in terms of # periods). Format: min:max")
		("amplitudeSigma", boost::program_options::value<double>(&brainAmplitudeSigma),
				"Sigma of brain amplitude mutation (relative to [0,1]).")
		("amplitudeBounds", boost::program_options::value<std::string>(),
				"Bounds of brain amplitude (must be a sub-interval of [0,1]). Format: min:max")
		("numInitialParts", boost::program_options::value<std::string>(),
				"Number of initial body parts (not "\
				"including core component). Format: min:max")
		("maxBodyMutationAttempts",
				boost::program_options::value<unsigned int>(
				&maxBodyMutationAttempts),
				"Max number of body mutation attempts")
		("pBrainCrossover",
				boost::program_options::value<double>(
				&pBrainCrossover), "Probability of crossover among brains")
		("socket", boost::program_options::value<std::vector<std::string> >()
				->required(),	"Sockets to be used to connect to the server")
		("addBodyPart",
				boost::program_options::value<std::vector<std::string> >(
				&allowedBodyPartTypeStrings),
				"Parts to be used in body evolution")
		("maxBodyParts",
				boost::program_options::value<unsigned int>(&maxBodyParts),
				"Maximum number of body parts.")
		("bodyParamSigma", boost::program_options::value<double>(&bodyParamSigma),
				"Sigma of body param mutation (all params in [0,1])")
		("evolutionaryAlgorithm",
				boost::program_options::value<std::string>(),
				"EA: Basic or HyperNEAT")
		("neatParamsFile",
				boost::program_options::value<std::string>(&neatParamsFile),
				"File for NEAT/HyperNEAT specific params");
	// generate body operator probability options from contraptions in header
	for (unsigned i=0; i<NUM_BODY_OPERATORS; ++i){
		desc.add_options()(
			BodyMutationOperatorsProbabilityCodes[i].c_str(),
			boost::program_options::value<double>(
			&bodyOperatorProbability[i]),"Probability of given operator");
	}
	boost::program_options::variables_map vm;

	if (confFileName == "help") {
		desc.print(std::cout);
		return true;
	}


	try{
		boost::program_options::store(
					boost::program_options::parse_config_file<char>(
							confFileName.c_str(), desc, true), vm);
		boost::program_options::notify(vm);
	}
	catch (boost::program_options::error& e) {
		std::cout << "Error while processing options: " << e.what() <<
				std::endl;
		return false;
	}

	// parse selection type
	if (vm["selection"].as<std::string>() == "deterministic-tournament"){
		selection = DETERMINISTIC_TOURNAMENT;
	}
	else {
		std::cout << "Specified selection strategy \"" <<
				vm["selection"].as<std::string>() <<
				"\" unknown. Options are \"deterministic-tournament\" or ..."
				 << std::endl;
		return false;
	}

	// parse replacement type
	if (vm["replacement"].as<std::string>() == "comma"){
		replacement = COMMA_REPLACEMENT;
	}
	else if (vm["replacement"].as<std::string>() == "plus"){
		replacement = PLUS_REPLACEMENT;
	}
	else {
		std::cout << "Specified replacement strategy \"" <<
				vm["replacement"].as<std::string>() <<
				"\" unknown. Options are \"comma\" or \"plus\"" << std::endl;
		return false;
	}

	// parse evolution mode
	if (vm["evolutionMode"].as<std::string>() == "brain"){
		evolutionMode = BRAIN_EVOLVER;
		if (referenceRobotFile.compare("") == 0) {
			std::cout << "evolutionMode is \"brain\" but no referenceRobotFile"
					<< " was provided" << std::endl;
			return false;
		}
	}
	else if (vm["evolutionMode"].as<std::string>() == "full"){
		evolutionMode = FULL_EVOLVER;
	}
	else {
		std::cout << "Specified evolution mode \"" <<
				vm["evolutionMode"].as<std::string>() <<
				"\" unknown. Options are \"brain\" or \"full\"" << std::endl;
		return false;
	}

	// parse brain bounds.
	if(vm.count("brainBounds") > 0){
		if(!parseBounds(vm["brainBounds"].as<std::string>(), minBrainWeight,
				maxBrainWeight))
			return false;
		minBrainBias = minBrainWeight;
		maxBrainBias = maxBrainWeight;
	} else if(vm.count("weightBounds") > 0 && vm.count("biasBounds") > 0) {
		if(!parseBounds(vm["weightBounds"].as<std::string>(), minBrainWeight,
				maxBrainWeight))
			return false;
		if(!parseBounds(vm["biasBounds"].as<std::string>(), minBrainBias,
				maxBrainBias))
			return false;
	} else {
		std::cout << "Must supply either brainBounds or (weightBounds and " <<
				"biasBounds) (and bounds for other brain params)" << std::endl;
		return false;
	}
	if(vm.count("brainSigma") > 0){
		brainWeightSigma = atof(vm["brainSigma"].as<std::string>().c_str());
		brainBiasSigma = brainWeightSigma;
	} else if(vm.count("weightSigma") > 0 && vm.count("biasSigma") > 0) {
		brainWeightSigma = atof(vm["weightSigma"].as<std::string>().c_str());
		brainBiasSigma = atof(vm["biasSigma"].as<std::string>().c_str());
	} else {
		std::cout << "Must supply either brainSigma or (weightSigma and biasSigma)"
				<< " ( and sigmas for other brain params)" << std::endl;
	}

	if(vm.count("tauBounds") > 0) {
		if(!parseBounds(vm["tauBounds"].as<std::string>(), minBrainTau,
				maxBrainTau))
			return false;
	}

	if(vm.count("periodBounds") > 0) {
		if(!parseBounds(vm["periodBounds"].as<std::string>(), minBrainPeriod,
				maxBrainPeriod))
			return false;
	}

	if(vm.count("phaseOffsetBounds") > 0) {
		if(!parseBounds(vm["phaseOffsetBounds"].as<std::string>(),
				minBrainPhaseOffset, maxBrainPhaseOffset))
			return false;
	}

	if(vm.count("amplitudeBounds") > 0) {
		if(!parseBounds(vm["amplitudeBounds"].as<std::string>(),
				minBrainAmplitude, maxBrainAmplitude))
			return false;
		if (minBrainAmplitude < 0) {
			std::cout << "Amplitude cannot be less than 0" << std::endl;
			return false;
		}
		if (maxBrainAmplitude > 1) {
			std::cout << "Amplitude cannot be greater than 1" << std::endl;
			return false;
		}
	}

	boost::cmatch match;
	static const boost::regex initPartsRegex(
				"^(\\d+):(\\d+)$");
	if (vm.count("numInitialParts") > 0) {
		if (!boost::regex_match(vm["numInitialParts"].as<std::string>().c_str(),
				match, initPartsRegex)){
			std::cout << "Supplied numInitialParts argument \"" <<
					vm["numInitialParts"].as<std::string>() <<
					"\" does not match pattern <min>:<max>" << std::endl;
			return false;
		}
		minNumInitialParts = std::atof(match[1].first);
		maxNumInitialParts = std::atof(match[2].first);
	}


	// parse sockets. The used regex is not super-restrictive, but we count
	// on the TcpSocket to find the error... else:
	// http://www.regular-expressions.info/examples.html
	static const boost::regex socketRegex("^([\\d\\.]*):(\\d*)$");
	std::vector<std::string> encSocket =
			vm["socket"].as<std::vector<std::string> >();
	sockets.clear();
	for (unsigned int i = 0; i<encSocket.size(); i++){
		// match[0]:whole string, match[1]:IP, match[2]:port
		if (!boost::regex_match(encSocket[i].c_str(), match, socketRegex)){
			std::cout << "Supplied socket argument \"" << encSocket[i] <<
					"\" does not match pattern <ip address>:<port>" <<
					std::endl;
			return false;
		}
		sockets.push_back(std::pair<std::string, int>(std::string(match[1]),
				std::atoi(match[2].first)));
	}

	// now that everything is parsed, we verify configuration validity
	// ===================================

	// - if selection is deterministic tournament, 1 <= tournamentSize <= mu
	if (selection == DETERMINISTIC_TOURNAMENT && (tournamentSize < 1 ||
			tournamentSize > mu)){
		std::cout << "Specified tournament size should be between 1 and mu, "\
				"but is " << tournamentSize << std::endl;
		return false;
	}

	// - if replacement is comma, lambda must exceed mu
	if (replacement == COMMA_REPLACEMENT && lambda < mu){
		std::cout << "If replacement is comma, lambda must be bigger than mu,"\
				"but lambda is " << lambda << " and mu is " << mu << std::endl;
		return false;
	}

	// - brain bounds max must > min
	if (maxBrainWeight < minBrainWeight){
		std::cout << "Minimum brain bound " << minBrainWeight << " exceeds "\
				"maximum " << maxBrainWeight << std::endl;
		return false;
	}

	// - 0. <= probabilities <= 1.
	if (pBrainMutate > 1. || pBrainMutate < 0.){
		std::cout << "Brain mutation probability parameter " << pBrainMutate <<
				" not between 0 and 1!" << std::endl;
		return false;
	}
	if (pBrainCrossover > 1. || pBrainCrossover < 0.){
		std::cout << "Brain crossover probability parameter " << pBrainCrossover
				<< " not between 0 and 1!" << std::endl;
		return false;
	}
	for(unsigned i=0; i<NUM_BODY_OPERATORS; ++i){
		if (bodyOperatorProbability[i] > 1. || bodyOperatorProbability[i] < 0.){
			std::cout << BodyMutationOperatorsProbabilityCodes[i] <<
					bodyOperatorProbability[i]	<< " not between 0 and 1!" <<
					std::endl;
			return false;
		}
	}

	// - sigma needs to be positive
	if (brainWeightSigma < 0.){
		std::cout << "Weight sigma (" << brainWeightSigma << ") must be positive" <<
				std::endl;
		return false;
	}

	if (brainBiasSigma < 0.){
		std::cout << "Bias sigma (" << brainBiasSigma << ") must be positive" <<
				std::endl;
		return false;
	}

	if (evolutionMode == FULL_EVOLVER) { // otherwise none of this matters
		// allows specifying body parts either as string or as char

		// If only one allowedBodyPartType, and matches "All" then
		// add all of them

		if (allowedBodyPartTypeStrings.size() == 1) {

			std::string lowerCaseBodyPart = allowedBodyPartTypeStrings[0];
			std::transform(lowerCaseBodyPart.begin(), lowerCaseBodyPart.end(),
					lowerCaseBodyPart.begin(), ::tolower);

			if (lowerCaseBodyPart.compare("all") == 0) {

				// Add all body parts
				allowedBodyPartTypeStrings.clear();

				for(std::map<char, std::string>::const_iterator
						it = PART_TYPE_MAP.begin();
						it != PART_TYPE_MAP.end(); ++it) {
					allowedBodyPartTypeStrings.push_back(it->second);
				}

			}

		}

		for(unsigned int i = 0; i<allowedBodyPartTypeStrings.size(); i++) {
			std::string bodyPartType =  allowedBodyPartTypeStrings[i];
			char type;
			if (bodyPartType.length() == 1) {
				type = bodyPartType[0];
				if( PART_TYPE_MAP.count(type) == 0) {
					std::cout << "Invalid body part type: " << type
							<< std::endl;
					return false;
				}
			} else if( INVERSE_PART_TYPE_MAP.count(bodyPartType) == 0) {
				std::cout << "Invalid body part type: " << bodyPartType
						<< std::endl;
				return false;
			} else {
				type = INVERSE_PART_TYPE_MAP.at(bodyPartType);
			}

			// don't want to be able to add more core components
			if(type != INVERSE_PART_TYPE_MAP.at(PART_TYPE_CORE_COMPONENT)) {
				allowedBodyPartTypes.push_back(type);
			}
		}

		// need at least one body part to be able to add
		if ( allowedBodyPartTypes.size() == 0) {
			std::cout << "If evolving bodies then need to define at least " <<
					"one allowed body part to add." << std::endl;
			return false;
		}
	}



	// convert file names to absolute paths, so that will use one relative to
	// directory where the evolver conf file is

	const boost::filesystem::path confFilePath(this->confFileName);

	if ( referenceRobotFile.compare("") != 0 ) {
		const boost::filesystem::path referenceRobotFilePath(
				referenceRobotFile);
		if (!referenceRobotFilePath.is_absolute()) {
			const boost::filesystem::path absolutePath =
					boost::filesystem::absolute(referenceRobotFilePath,
							confFilePath.parent_path());
			referenceRobotFile = absolutePath.string();
		}
	}

	if ( simulatorConfFile.compare("") != 0 ) {
		const boost::filesystem::path simulatorConfFilePath(
				simulatorConfFile);
		if (!simulatorConfFilePath.is_absolute()) {
			const boost::filesystem::path absolutePath =
					boost::filesystem::absolute(simulatorConfFilePath,
							confFilePath.parent_path());
			simulatorConfFile = absolutePath.string();
		}
	}


	// ---------------------------------------
	// HyperNEAT stuff -- hardcoded for now
	// TODO make configurable
	// ---------------------------------------

	evolutionaryAlgorithm = BASIC;
	if ( vm.count("evolutionaryAlgorithm") > 0 ) {
		if (vm["evolutionaryAlgorithm"].as<std::string>().compare("HyperNEAT")
				== 0) {

			evolutionaryAlgorithm = HYPER_NEAT;
			neatParams.PopulationSize = mu;


			//defaults
			neatParams.WeightDiffCoeff = 0.4;
			neatParams.DynamicCompatibility = true;
			neatParams.CompatTreshold = 3.0;
			neatParams.YoungAgeTreshold = 15;
			neatParams.SpeciesMaxStagnation = 15;
			neatParams.OldAgeTreshold = 35;
			neatParams.MinSpecies = 4; //5;
			neatParams.MaxSpecies = 18; //25;
			neatParams.RouletteWheelSelection = false;
			neatParams.RecurrentProb = 0;
			neatParams.OverallMutationRate = 0.8;

			neatParams.MutateWeightsProb = 0.90;

			neatParams.WeightMutationMaxPower = 2.5;
			neatParams.WeightReplacementMaxPower = 5.0;
			neatParams.MutateWeightsSevereProb = 0.5;
			neatParams.WeightMutationRate = 0.25;

			neatParams.MaxWeight = 8;

			neatParams.MutateAddNeuronProb = 0.03;
			neatParams.MutateAddLinkProb = 0.05;
			neatParams.MutateRemLinkProb = 0.001;

			neatParams.MinActivationA  = 1.0;
			neatParams.MaxActivationA  = 6.0;

			neatParams.ActivationFunction_SignedSigmoid_Prob = 1.0;
			neatParams.ActivationFunction_UnsignedSigmoid_Prob = 0.0;
			neatParams.ActivationFunction_Tanh_Prob = 1.0;
			neatParams.ActivationFunction_TanhCubic_Prob = 0.0;
			neatParams.ActivationFunction_SignedStep_Prob = 0.0;
			neatParams.ActivationFunction_UnsignedStep_Prob = 0.0;
			neatParams.ActivationFunction_SignedGauss_Prob = 1.0;
			neatParams.ActivationFunction_UnsignedGauss_Prob = 0.0;
			neatParams.ActivationFunction_Abs_Prob = 0.0;
			neatParams.ActivationFunction_SignedSine_Prob = 1.0;
			neatParams.ActivationFunction_UnsignedSine_Prob = 0.0;
			neatParams.ActivationFunction_Linear_Prob = 1.0;


			if ( neatParamsFile.compare("") != 0 ) {
				const boost::filesystem::path neatParamsFilePath(
						neatParamsFile);
				if (!neatParamsFilePath.is_absolute()) {
					const boost::filesystem::path absolutePath =
							boost::filesystem::absolute(neatParamsFilePath,
									confFilePath.parent_path());
					neatParamsFile = absolutePath.string();
				}

				if (neatParams.Load(neatParamsFile.c_str()) < 0) {
					std::cout << "Problem parsing neatParams file." <<
							std::endl;
					return false;
				}
			}
		}
	}





	return true;
}

}
