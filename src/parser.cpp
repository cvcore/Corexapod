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
	(bool, empty_)
	(bool, absolute_)
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
		using qi::_1;
		float x = 0.f, y = 0.f, z = 0.f;

		mem_ %= '[' >> (int_ % ',') >> ']';
//		vec3f_ = float_[ref(x) = _1] >> ',' >> float_[ref(y) = _1] >> ',' >> float_[ref(z) = _1] >> eps[_val = Eigen::Vector3f(ref(x), ref(y), ref(z))];

		grp_ = (( char_("L")[at_c<0>(_val) = qi::_1] >> mem_[at_c<1>(_val) = qi::_1] | char_('B')[at_c<0>(_val) = qi::_1] )
				>> char_[at_c<2>(_val) = qi::_1]
				>> '(' >> float_[at_c<3>(_val) = qi::_1 ] >> ',' >> float_[at_c<4>(_val) = qi::_1 ] >> ',' >> float_[at_c<5>(_val) = qi::_1 ] >> ')')
			    | char_("V")[at_c<0>(_val) = qi::_1] >> mem_[at_c<1>(_val) = qi::_1] >> char_[at_c<2>(_val) = qi::_1] >> float_[at_c<3>(_val) = qi::_1];

		line_ = (lit("NOP")[at_c<2>(_val) = true] |
				+grp_[push_back(at_c<0>(_val), qi::_1), at_c<2>(_val) = false]) >> char_('T') >> int_[at_c<1>(_val) = qi::_1] >> -lit('$')[at_c<3>(_val) = true];

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

Parser::Parser(const char* scriptPath, Hexapod& hexapod) : _hexapod(hexapod), autoenable_(false), _last(0), finish_(false) {
	namespace qi = boost::spirit::qi;
	if(scriptPath == NULL) {
		std::cout << "[Parser] Empty file path.\n";
		return;
	}
	std::ifstream fin(scriptPath, std::ios::in | std::ios::binary);
	if(!fin) {
		std::cout << "[Parser] File open error.\n";
		return;
	}
	std::string strin((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
	std::string::const_iterator begin(strin.begin()), end(strin.end());
	ActionScriptParser<std::string::const_iterator> parser;
	if(!qi::phrase_parse(begin, end, parser, qi::space, _as) || begin != end)
		std::cout << "[Parser] Parse file: " << scriptPath << " failed.\n";
	this->buildIndex();
	std::srand(std::time(0)); //feed rand generator
}

void Parser::buildIndex() {
	std::vector<Block>::const_iterator it;
	for(it = _as.block_.begin(); it != _as.block_.end(); it++) {
		_index.insert(std::pair<std::string, const Block&>(it->methodName_, *it));
	}
	// for each

}

std::ostream& operator<<(std::ostream &os, const Line& line) {
	std::vector<Group>::const_iterator git;
	for(git = line.group_.begin(); git != line.group_.end(); git++) {
		os << git->groupType_ << '[';
		for_each(git->members_.begin(), git->members_.end(), os << boost::lambda::_1 << ' ');
		os << "] " << git->moveType_ << '(' << git->x_ << ',' << git->y_ << ',' << git->z_ << ") | ";

	}
	os << 'T' << line.time_ << '\n';
	return os;
}

std::ostream& operator<<(std::ostream& os, const Parser& p) {
	os << "Parsed: " << p._as.block_.size() << " blocks.\n";
	std::vector<Block>::const_iterator bit;
	for(bit = p._as.block_.begin(); bit != p._as.block_.end(); bit++) {
		os << bit->methodName_ << ": " << bit->line_.size() << " lines.\n";
		std::vector<Line>::const_iterator lit;
		for(lit = bit->line_.begin(); lit != bit->line_.end(); lit++) {
			os << '\t';
			os << *lit;
		}
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const Plane& p) {
	os << "[Debug][Plane]" <<'\n';
	os << "\t Front:" << p.front_.x() << ' ' << p.front_.y() << ' ' << p.front_.z() << '\n'
		<<"\t Normal:" << p.normal_.x() << ' ' << p.normal_.y() << ' ' << p.normal_.z() << '\n'
		<<"\t Origin:" << p.origin_.x() << ' ' << p.origin_.y() << ' ' << p.origin_.z() << '\n';
	return os;
}

bool Parser::act(const std::string& methodName) {
	if(_index.count(methodName) == 0) {
		std::cout << "[Parser] Method name not exist.\n";
		return false;
	}
	Block block = _index.at(methodName);
	for_each(block.line_.begin(), block.line_.end(), boost::bind(&Parser::parseLine, this, boost::lambda::_1));
	return true;
}

void Parser::randomAct() {
//	int methodNo = std::rand() % (_index.size());
	_last = (_last + 1) % _index.size();
	int methodNo  = _last;
	std::cout << "[Parser] Auto: " << _last << '\n';
	std::map<std::string, const Block&>::const_iterator it = _index.begin();
	for(; methodNo > 0; methodNo--, it++);
	this->act(it->first);
}

std::string Parser::parseSocket(const std::string& socketStr) {
	std::stringstream ss;
	if(socketStr == "front") {
		_hexapod.moveLinear(Eigen::Vector3f(65, 0, 0), 1000, 3);
		ss << "success";
	} else if(socketStr == "back") {
		_hexapod.moveLinear(Eigen::Vector3f(-65, 0, 0), 1000, 3);
		ss << "success";
	} else if(socketStr == "up") {
		if(_hexapod.base_.origin_.z() < 210) {
			_hexapod.base_.translate(_hexapod.base_.origin_ + Eigen::Vector3f(0, 0, 20), 400);
			_hexapod.syncServoWithDelay(400);
		}
		ss << "success";
	} else if(socketStr == "down") {
		if(_hexapod.base_.origin_.z() > 110) {
			_hexapod.base_.translate(_hexapod.base_.origin_ + Eigen::Vector3f(0, 0, -20), 400);
			_hexapod.syncServoWithDelay(400);
		}
		ss << "success";
	} else if(socketStr == "left") {
		_hexapod.moveAngular(PI / 10, 1000, 2);
		ss << "success";
	} else if(socketStr == "right") {
		_hexapod.moveAngular(-PI / 10, 1000, 2);
		ss << "success";
	} else if(socketStr == "turnLeft") {
		_hexapod.moveLinear(Eigen::Vector3f(40, 40, 0), 1000, 3);
		ss << "success";
	} else if(socketStr == "turnRight") {
		_hexapod.moveLinear(Eigen::Vector3f(40, -40, 0), 1000, 3);
		ss << "success";
	} else if(socketStr == "autoenable") { // todo
		ss << "success";
		autoenable_ = true;
	} else if(socketStr == "autodisable") {
		ss << "success";
		autoenable_ = false;
	} else if(socketStr == "poweroff") {
		ss << "success";
		//_hexapod.power_.logicPower(poweroff);
		finish_ = true;
	} else if(socketStr == "reboot") {
		ss << "success";
		_hexapod.power_.logicPower(restart);
		finish_ = true;
	} else if(socketStr == "status") {
		ss << "{\"totalUseTime\":\"" << _hexapod.getTotalUseTime() << "\"," <<
			  "\"powerCycle\":" << _hexapod.getPowerCycle() << "," <<
			  "\"battery\":\"" << _hexapod.power_.readBatteryPercentage() << "%\"}";
		std::cout << "[Parser][Debug] " << ss.str() << '\n';
	} else {
		this -> act(socketStr);
		ss << "success";
	}
	return ss.str();
}

void Parser::parseLine(const Line& line) const {
	std::vector<Group>::const_iterator it;
	bool baseLine = false;
	bool servoLine = false;
	std::cout << "[Parser][Debug] " << line << '\n';
	std::cout << _hexapod.base_;
	for(it = line.group_.begin(); it != line.group_.end(); it++) {
		Eigen::Vector3f newVec(it->x_, it->y_, it->z_);
		switch(it->groupType_) {
		case 'B' :
			baseLine = true;
			switch(it->moveType_) {
			case 'N':
				_hexapod.base_.rotateNorm(newVec, line.time_);
				_hexapod.syncServoWithDelay(line.time_);
				break;
			case 'F':
				_hexapod.base_.rotateFront(_hexapod.base_.tfVector(newVec), line.time_);
				_hexapod.syncServoWithDelay(line.time_);
				break;
			case 'O':
				if(!line.absolute_) {
					newVec = _hexapod.base_.tfVector(newVec);
					newVec = newVec + _hexapod.base_.origin_;
				} else { //absolute value only to z axis
					newVec(0) = _hexapod.base_.origin_(0);
					newVec(1) = _hexapod.base_.origin_(1);
				}
				_hexapod.base_.translate(newVec, line.time_);
				_hexapod.syncServoWithDelay(line.time_);
				break;
			case 'f':
				_hexapod.rotateBodyAround(newVec, line.time_);
				break;
			default:
				std::cout << "[Parser] Invalid move type.\n";
			}
			break;

		case 'L':
			switch(it->moveType_) {
			case 'S':
				_hexapod.base_.stepGroup(_hexapod.base_.tfVector(newVec), line.time_, it->members_, 40.f);
				break;
			case 'M':
				//if not absolute
				_hexapod.base_.addRelMovementGroup(_hexapod.base_.tfVector(newVec), line.time_, it->members_);
				break;
			}
			break;

		case 'V':
			servoLine = true;
			switch(it->moveType_) {
			case 'A':
				int legIdx = it->members_[0], servoIdx = it->members_[1];
				Servo* pServo = _hexapod.base_.getServo(legIdx, servoIdx);
				pServo->setAngle(it->x_ / 180.f * PI);
				pServo->setActTime(line.time_);
			}
			break;
		}
	}
	if(servoLine) {
		_hexapod.syncServoWithDelay(line.time_);
	} else if(!baseLine) {
		_hexapod.parseMovement();
		const int allMembers[6] = {0, 1, 2, 3, 4, 5};
		std::vector<int> allMembersVec(allMembers, allMembers + 6);
		_hexapod.base_.resetMovementGroup(allMembersVec);
	}
}

}
