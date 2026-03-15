#ifndef CNAV_MYSTYLE_LEGACY_TRACE_EXPORT_H_
#define CNAV_MYSTYLE_LEGACY_TRACE_EXPORT_H_

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <string>
#include <vector>

#include "../src/Vector2.h"

struct LegacyTraceCandidateAction {
	int action_index;
	RVO::Vector2 intended_velocity;
	float total_reward;
};

struct LegacyTraceConfig {
	bool enabled;
	std::string output_path;
	int max_steps;
	unsigned int seed;
};

inline int readTraceEnvInt(const char *name, int fallback)
{
	const char *raw = std::getenv(name);
	if (raw == NULL || raw[0] == '\0') {
		return fallback;
	}
	char *end = NULL;
	const long parsed = std::strtol(raw, &end, 10);
	if (end == raw) {
		return fallback;
	}
	return static_cast<int>(parsed);
}

inline unsigned int readTraceEnvSeed(const char *name, unsigned int fallback)
{
	const int parsed = readTraceEnvInt(name, static_cast<int>(fallback));
	if (parsed < 0) {
		return fallback;
	}
	return static_cast<unsigned int>(parsed);
}

inline LegacyTraceConfig loadLegacyTraceConfig()
{
	const char *outputPath = std::getenv("CNAV_TRACE_OUTPUT");
	const bool enabled = outputPath != NULL && outputPath[0] != '\0';
	LegacyTraceConfig config = {enabled, "", 0, 1u};
	if (enabled) {
		config.output_path = outputPath;
		config.max_steps = readTraceEnvInt("CNAV_TRACE_STEPS", 0);
		config.seed = readTraceEnvSeed("CNAV_TRACE_SEED", 1u);
	}
	return config;
}

inline void writeJsonFloat(std::ostream &output, float value)
{
	output << std::setprecision(9) << value;
}

inline void writeJsonVector(std::ostream &output, const RVO::Vector2 &value)
{
	output << "[";
	writeJsonFloat(output, value.x());
	output << ", ";
	writeJsonFloat(output, value.y());
	output << "]";
}

inline void writeJsonIntList(std::ostream &output, const std::vector<int> &values)
{
	output << "[";
	for (size_t index = 0; index < values.size(); ++index) {
		if (index > 0) {
			output << ", ";
		}
		output << values[index];
	}
	output << "]";
}

inline void writeLegacyTraceRecord(
	std::ostream &output,
	int stepIndex,
	float globalTime,
	int agentIndex,
	const std::string &agentName,
	bool actionUpdated,
	const std::vector<int> &rankedNeighbors,
	const RVO::Vector2 &communicatedIntendedVelocity,
	int chosenActionIndex,
	const RVO::Vector2 &chosenIntendedVelocity,
	const RVO::Vector2 &outputVelocity,
	const std::vector<LegacyTraceCandidateAction> &candidateActions,
	bool hasNextAction,
	int nextActionIndex,
	const RVO::Vector2 &nextChosenIntendedVelocity)
{
	output << "{";
	output << "\"step_index\": " << stepIndex << ", ";
	output << "\"global_time\": ";
	writeJsonFloat(output, globalTime);
	output << ", ";
	output << "\"agent_index\": " << agentIndex << ", ";
	output << "\"agent_name\": \"" << agentName << "\", ";
	output << "\"action_updated\": " << (actionUpdated ? "true" : "false") << ", ";
	output << "\"ranked_neighbors\": ";
	writeJsonIntList(output, rankedNeighbors);
	output << ", ";
	output << "\"communicated_intended_velocity\": ";
	writeJsonVector(output, communicatedIntendedVelocity);
	output << ", ";
	output << "\"chosen_action_index\": " << chosenActionIndex << ", ";
	output << "\"chosen_intended_velocity\": ";
	writeJsonVector(output, chosenIntendedVelocity);
	output << ", ";
	output << "\"output_velocity\": ";
	writeJsonVector(output, outputVelocity);
	output << ", ";
	output << "\"candidate_actions\": [";
	for (size_t index = 0; index < candidateActions.size(); ++index) {
		if (index > 0) {
			output << ", ";
		}
		output << "{";
		output << "\"action_index\": " << candidateActions[index].action_index << ", ";
		output << "\"intended_velocity\": ";
		writeJsonVector(output, candidateActions[index].intended_velocity);
		output << ", ";
		output << "\"goal_progress_reward\": null, ";
		output << "\"constrained_reduction_reward\": null, ";
		output << "\"total_reward\": ";
		writeJsonFloat(output, candidateActions[index].total_reward);
		output << "}";
	}
	output << "]";
	output << ", ";
	output << "\"next_action_index\": ";
	if (hasNextAction) {
		output << nextActionIndex;
	} else {
		output << "null";
	}
	output << ", ";
	output << "\"next_chosen_intended_velocity\": ";
	if (hasNextAction) {
		writeJsonVector(output, nextChosenIntendedVelocity);
	} else {
		output << "null";
	}
	output << "}";
	output << std::endl;
}

#endif
