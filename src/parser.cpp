/*
 * parser.cpp
 *
 *  Created on: Jun 21, 2014
 *      Author: core
 */

#include "parser.hpp"

BOOST_FUSION_ADAPT_STRUCT(
	hex::Group,
	(char, groupType_)
	(std::vector<int>, members_)
	(char, moveType_)
	(float, x_)
	(float, y_)
	(float, z_)
//	(Eigen::Vector3f, position_)
)

BOOST_FUSION_ADAPT_STRUCT(
	hex::Line,
	(std::vector<hex::Group>, group_)
	(int, time_)
)

BOOST_FUSION_ADAPT_STRUCT(
	hex::Block,
	(std::string, methodName_)
	(std::vector<hex::Line>, line_)
)

BOOST_FUSION_ADAPT_STRUCT(
	hex::ActionScript,
	(std::vector<hex::Block>, block_)
)

namespace hex {

template <typename Iterator>
struct ActionScriptParser : qi::grammar<Iterator, ActionScript(), qi::space_type> {
	ActionScriptParser() : ActionScriptParser::base_type(as_) {
		using namespace qi;
		using boost::spirit::ascii::char_;
		using boost::phoenix::push_back;
		using boost::phoenix::ref;
		using boost::phoenix::at_c;
		float x = 0.f, y = 0.f, z = 0.f;

		mem_ %= '[' >> (int_ % ',') >> ']';
//		vec3f_ = float_[ref(x) = _1] >> ',' >> float_[ref(y) = _1] >> ',' >> float_[ref(z) = _1] >> eps[_val = Eigen::Vector3f(ref(x), ref(y), ref(z))];

		grp_ = ( char_('L')[at_c<0>(_val) = qi::_1] >> mem_[at_c<1>(_val) = qi::_1] | char_('B')[at_c<0>(_val) = qi::_1] )
				>> char_[at_c<2>(_val) = qi::_1]
				>> '(' >> float_[at_c<3>(_val) = qi::_1 ] >> ',' >> float_[at_c<4>(_val) = qi::_1 ] >> ',' >> float_[at_c<5>(_val) = qi::_1 ] >> ')';

		line_ = +grp_[push_back(at_c<0>(_val), qi::_1)] >> char_('T') >> int_[at_c<1>(_val) = qi::_1];

		blk_ = +(char_ - "{")[at_c<0>(_val) += _1]
		         >> '{'
		         >> +line_[push_back(at_c<1>(_val), _1)]
		         >> '}';
		as_ = *blk_[push_back(at_c<0>(_val), _1)];
	}

	qi::rule<Iterator, Group(), qi::space_type> grp_;
	qi::rule<Iterator, std::vector<int>(), qi::space_type> mem_;
	qi::rule<Iterator, Line(), qi::space_type> line_;
	qi::rule<Iterator, Block(), qi::space_type> blk_;
	qi::rule<Iterator, ActionScript(), qi::space_type> as_;
//	qi::rule<Iterator, Eigen::Vector3f(), qi::space_type> vec3f_;

};

Parser::Parser(const char* scriptPath, Hexapod& hexapod) : _hexapod(hexapod) {
	namespace qi = boost::spirit::qi;
	if(scriptPath == NULL) {
		std::cout << "[Parser] Empty file path.\n";
		return;
	}
	std::ifstream fin(scriptPath, std::ios::in | std::ios::binary);
	std::string strin((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
	std::string::const_iterator begin(strin.begin()), end(strin.end());
	ActionScriptParser<std::string::const_iterator> parser;
	if(!qi::phrase_parse(begin, end, parser, qi::space, _as) || begin != end)
		std::cout << "[Parser] Parse file: " << scriptPath << " failed.\n";
	this->buildIndex();
}

void Parser::buildIndex() {
	std::vector<Block>::const_iterator it;
	for(it = _as.block_.begin(); it != _as.block_.end(); it++) {
		_index.insert(std::make_pair(it->methodName_, *it));
	}
	// for each

}

std::ostream& operator<<(std::ostream& os, const Parser& p) {
	os << "Parsed: " << p._as.block_.size() << " blocks.\n";
	std::vector<Block>::const_iterator bit;
	for(bit = p._as.block_.begin(); bit != p._as.block_.end(); bit++) {
		os << bit->methodName_ << ": " << bit->line_.size() << " lines.\n";
		std::vector<Line>::const_iterator lit;
		for(lit = bit->line_.begin(); lit != bit->line_.end(); lit++) {
			os << '\t';
			std::vector<Group>::const_iterator git;
			for(git = lit->group_.begin(); git != lit->group_.end(); git++) {
				os << git->groupType_ << '[';
				for_each(git->members_.begin(), git->members_.end(), os << boost::lambda::_1 << ' ');
				os << "] " << git->moveType_ << '(' << git->x_ << ',' << git->y_ << ',' << git->z_ << ") | ";

			}
			os << 'T' << lit->time_ << '\n';
		}
	}
	return os;
}

}
