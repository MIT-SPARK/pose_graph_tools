#pragma once

#include <vector>
#include "pose_graph_tools/bow_query.h"

namespace pose_graph_tools {

struct BoWQueries {
unsigned int destination_robot_id;
std::vector<BoWQuery> queries;   
};

}  // namespace pose_graph_tools