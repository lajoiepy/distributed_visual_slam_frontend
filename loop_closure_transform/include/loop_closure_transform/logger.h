#include "ros/ros.h"
#include "loop_closure_transform/FindMatches.h"
#include "loop_closure_transform/ReceiveSeparators.h"
#include "stdio.h"
#include <boost/range.hpp>

void log_find_matches_query(loop_closure_transform::FindMatches::Request &req);
void log_find_matches_answer(loop_closure_transform::FindMatches::Response &res);

void log_receive_separators_query(loop_closure_transform::ReceiveSeparators::Request &req);
void log_receive_separators_answer(loop_closure_transform::ReceiveSeparators::Response &res);