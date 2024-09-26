#ifndef OPTIONS_HPP_
#define OPTIONS_HPP_

#define OUTPUT_PATH			"D://C_Project//power-diagram//data//"

#include <map>

namespace PowerDiagramGenerator {
	class Options {
	public:
		bool showHelp;
		bool exact;
		bool voronoi;
		bool bounding;
		bool timecount;
		double r;
		std::string output_path;
		std::vector<std::string> outputList;
		std::vector<std::string> visualList;
	public:
		Options()
		: showHelp(false)
		, exact(false)
		, voronoi(false)
		, bounding(false)
		, timecount(false)
		, r(0)
		, output_path(OUTPUT_PATH)
		{
			boolArgsMap = {
				{"-help", &showHelp}, {"-h", &showHelp},
				{"-exact", &exact},
				{"-voronoi", &voronoi}, {"-vor", &voronoi},
				{"-bounding", &bounding}, {"-box", &bounding},
				{"-timecount", &timecount}, {"-tc", &timecount}
			};
			intArgsMap = {
			};
			doubleArgsMap = {
				{"-r", &r},
			};
			stringArgsMap = {
				{"-path", &output_path}, {"-op", &output_path}, {"-p", &output_path},
			};
		}

		void showOptions(std::ostream& o) const {
			o << "\tOPTIONS:\n";
			o << "\t-help|-h: show this help\n";
			o << "\t-exact: exactly compute the power diagram, it may result in an increase in computational overhead\n";
			o << "\t-voronoi|-vor: compute the voronoi diagram\n";
			o << "\t-bounding|-box: initialize the initial box for each point as the bounding box of the entire point cloud\n";
			o << "\t-r=[double]: set the radius of the initial box for each point [default 0]\n";
			o << "\n";
			o << "\t---------- output ----------\n";
			o << "\t-output|-o=type: generate the output of the power diagram in the form of the specified data type\n";
			o << "\t\tconnection|c: the connectivity relationships between the points\n";
			o << "\n";
			o << "\t---------- visual ----------\n";
			o << "\t-visual|-vis|-v=type: generate the visual output of the power diagram in the form of the specified data type.obj\n";
			o << "\t\twireframe|wire|w: the wireframe of power cells\n";
			o << "\t\tsolid|s: the solid of power cells\n";
			o << "\t\tconnection|c: the connectivity relationships between the points\n";
			o << "\n";
			o << "\t-path|-op|-p=[string]: set the output path [default ..//..//data//]\n";
			o << "\t-timecount|-tc: Output the time count information\n";
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

			if ((option == "-output" || option == "-o") && (!arg.empty() || i + 1 < argc)) {
				outputList.push_back(arg.empty() ? argv[++i] : arg.c_str());
				return true;
			}
			if ((option == "-visual" || option == "-vis" || option == "-v") && (!arg.empty() || i + 1 < argc)) {
				visualList.push_back(arg.empty() ? argv[++i] : arg.c_str());
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
