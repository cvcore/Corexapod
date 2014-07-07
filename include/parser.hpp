/*
 * parser.hpp
 *
 *  Created on: Jun 22, 2014
 *      Author: core
 */

#ifndef PARSER_HPP_
#define PARSER_HPP_

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <sstream>
#include <streambuf>
#include <map>
#include <utility>
#include <cstdlib>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_lexeme.hpp>
#include <boost/spirit/include/qi_alternative.hpp>
#include <boost/spirit/include/qi_char_.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_container.hpp>
#include <boost/fusion/adapted/struct.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/bind.hpp>
#include <Eigen/Dense>

extern "C" {
#include <unistd.h>
}

#include "engine.hpp"

namespace qi = boost::spirit::qi;

const float PI = 3.14159265359f;

namespace hex {

struct Group {
	char groupType_;
	std::vector<int> members_;
	char moveType_;
	float x_, y_, z_;
//	Eigen::Vector3f position_;
};

struct Line {
	std::vector<Group> group_;
	int time_;
	bool empty_, absolute_;
};

struct Block {
	std::string methodName_;
	std::vector<Line> line_;
};

struct ActionScript {
	std::vector<Block> block_;
};


class Parser {
public:
	Parser(const char* scriptPath, Hexapod& hexapod);
	bool act(const std::string& methodName);
	friend std::ostream& operator<<(std::ostream& os, const Parser& p);
	void parseLine(const Line& line) const;
	std::string parseSocket(const std::string& socketStr);

private:
	void buildIndex();

	Hexapod& _hexapod;
	ActionScript _as;
	std::map<std::string, const Block&> _index;
};

}

#endif /* PARSER_HPP_ */
