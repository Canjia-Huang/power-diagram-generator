#ifndef OPTIONS_HPP_
#define OPTIONS_HPP_

#include <map>

namespace PowerDiagramGenerator {
	class Options {
	public:
		bool visual;
		bool exact;
		bool voronoi;
		bool bounding;
		bool showHelp;
		double r;
		std::string output_path;
	public:
		Options()
		: visual(false)
		, exact(false)
		, voronoi(false)
		, bounding(false)
		, showHelp(false)
		, r(0)
		, output_path() 
		{
			boolArgsMap = {
				{"-vis", &visual},
				{"-exact", &exact},
				{"-voronoi", &voronoi},
				{"-bounding", &bounding},
				{"-help", &showHelp}, {"-h", &showHelp},
			};
			doubleArgsMap = {
				{"-r", &r},
			};
			stringArgsMap = {
				{"-output", &output_path}, {"-o", &output_path}
			};
		}

		void showOptions(std::ostream& o) const {
			o << "\tOPTIONS:\n";
			o << "\t--help|-h: show this help\n";
			o << "\n";
		}

		bool parseAnOption(int& i, int argc, char const* const* argv) {
			if (strlen(argv[i]) == 0) return true;

			std::string option(argv[i]);
			std::string arg;

			auto boolIt = boolArgsMap.find(option);
			if (boolIt != boolArgsMap.end()) {
				*boolIt->second = true;
				return true;
			}

			std::size_t found = option.find('=');
			if (found != std::string::npos) {
				arg = option.substr(found + 1);
				option = option.substr(0, found);
			}

			auto intIt = intArgsMap.find(option);
			if (intIt != intArgsMap.end()) {
				if (arg.empty() && i + 1 >= argc) return false;
				*intIt->second = atoi(arg.empty() ? argv[++i] : arg.c_str());
				return true;
			}

			auto doubleIt = doubleArgsMap.find(option);
			if (doubleIt != doubleArgsMap.end()) {
				if (arg.empty() && i + 1 >= argc) return false;
				*doubleIt->second = atoi(arg.empty() ? argv[++i] : arg.c_str());
				return true;
			}

			auto stringIt = stringArgsMap.find(option);
			if (stringIt != stringArgsMap.end()) {
				if (arg.empty() && i + 1 >= argc) return false;
				*stringIt->second = arg.empty() ? argv[++i] : arg.c_str();
				return true;
			}

			return false;
		}

		int parseOptions(int argc, char const* const* argv) {
			for (int i = 1; i < argc; ++i) {
				if (parseAnOption(i, argc, argv) == false) return i;
			}
			return argc;
		}
	private:
		std::map<std::string, bool*> boolArgsMap;
		std::map<std::string, int*> intArgsMap;
		std::map<std::string, double*> doubleArgsMap;
		std::map<std::string, std::string*> stringArgsMap;
	};

	Options& getOptions() {
		static Options s_options;
		return s_options;
	}
} // namespace PowerDiagramGenerator

#endif
